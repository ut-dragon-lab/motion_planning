// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <squeeze_navigation/planner/base_plugin.h>

#include <differential_kinematics/planner_core.h>
#include <aerial_motion_planning_msgs/multilink_state.h>

#include <pluginlib/class_loader.h>
/* special cost plugin for cartesian constraint */
#include <differential_kinematics/cost/cartesian_constraint.h>
/* special constraint plugin for collision avoidance */
#include <differential_kinematics/constraint/collision_avoidance.h>

/* utils */
#include <tf_conversions/tf_kdl.h>
#include <fnmatch.h>

enum Phase
  {
    PHASE1 = 1,
    PHASE2,
    PHASE3,
    PHASE4,
  };

namespace squeeze_motion_planner
{
  using namespace differential_kinematics;
  using CostContainer = std::vector<boost::shared_ptr<cost::Base> >;
  using ConstraintContainer = std::vector<boost::shared_ptr<constraint::Base> >;

  class DifferentialKinematics: public Base
  {
  public:
    DifferentialKinematics() {}
    ~DifferentialKinematics(){}

    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp, boost::shared_ptr<aerial_robot_model::transformable::RobotModel> robot_model_ptr)
    {
      Base::initialize(nh, nhp, robot_model_ptr);

      /* setup the planner core */
      planner_core_ptr_ = boost::shared_ptr<Planner> (new Planner(nh, nhp, robot_model_ptr_));
      planner_core_ptr_->registerUpdateFunc(std::bind(&DifferentialKinematics::updatePinchPoint, this));
      baselink_name_ = robot_model_ptr_->getBaselinkName();

      /* heuristic: serial model from link1 to linkN */
      /* CAUTION: getChain() doesn't include root_segment, please set the parent of root_segment */
      robot_model_ptr_->getTree().getChain(std::string("root"), std::string("link") + std::to_string(robot_model_ptr_->getRotorNum()), squeeze_chain_);


      /* base vars */
      phase_ = PHASE1;
      reference_point_ratio_ = 1.0;

      nhp_.param("delta_pinch_length", delta_pinch_length_, 0.02); // [m]
      nhp_.param("debug", debug_, true);

      /* set start pose */
      geometry_msgs::Pose pose;
      nhp_.param("start_state_x", pose.position.x, 0.0);
      nhp_.param("start_state_y", pose.position.y, 0.5);
      nhp_.param("start_state_z", pose.position.z, 0.0);
      double r, p, y;
      nhp_.param("start_state_roll", r, 0.0);
      nhp_.param("start_state_pitch", p, 0.0);
      nhp_.param("start_state_yaw", y, 0.0);
      pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(r,p,y);
      /* getTree().getNrOfJoints() = link_joint + gimbal + rotor */
      KDL::JntArray joint_state(robot_model_ptr_->getTree().getNrOfJoints());
      for(int i = 0; i < robot_model_ptr_->getLinkJointNames().size(); i++)
        nhp_.param(std::string("start_") + robot_model_ptr_->getLinkJointNames().at(i), joint_state(robot_model_ptr_->getLinkJointIndices().at(i)), 0.0);
      start_state_.setStatesFromRoot(robot_model_ptr_, pose, joint_state);

      /* set goal pose */
      nhp_.param("goal_state_x", pose.position.x, 0.0);
      nhp_.param("goal_state_y", pose.position.y, 0.5);
      nhp_.param("goal_state_z", pose.position.z, 0.0);
      nhp_.param("goal_state_roll", r, 0.0);
      nhp_.param("goal_state_pitch", p, 0.0);
      nhp_.param("goal_state_yaw", y, 0.0);
      pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(r,p,y);
      for(int i = 0; i < robot_model_ptr_->getLinkJointNames().size(); i++)
        nhp_.param(std::string("goal_") + robot_model_ptr_->getLinkJointNames().at(i), joint_state(robot_model_ptr_->getLinkJointIndices().at(i)), 0.0);
      goal_state_.setStatesFromRoot(robot_model_ptr_, pose, joint_state);

      /* get opening center frame from rosparam */
      nhp_.param("openning_pos_x", pose.position.x, 0.0);
      nhp_.param("openning_pos_y", pose.position.y, 0.0);
      nhp_.param("openning_pos_z", pose.position.z, 0.0);
      nhp_.param("openning_roll", r, 0.0);
      nhp_.param("openning_pitch", p, 0.0);
      nhp_.param("openning_yaw", y, 0.0);

      tf::pointMsgToTF(pose.position, openning_center_frame_.getOrigin());
      openning_center_frame_.setRotation(tf::createQuaternionFromRPY(r, p, y));

      setCollisionEnv();

      env_collision_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/env_collision", 1);
    }

    void setCollisionEnv()
    {
     /* hard-coding to set env */
      double wall_thickness;
      nhp_.param("wall_thickness", wall_thickness, 0.05);
      if(planner_core_ptr_->getMultilinkType() == motion_type::SE2)
        {
          /* setup env */
          double openning_width, env_width, env_length, openning_yaw;
          nhp_.param("openning_width", openning_width, 0.7);
          nhp_.param("openning_yaw", openning_yaw, 0.0);
          nhp_.param("env_width", env_width, 4.0);
          nhp_.param("env_length", env_length, 6.0);

          /* openning side wall(s) */
          visualization_msgs::Marker wall;
          wall.type = visualization_msgs::Marker::CUBE;
          wall.action = visualization_msgs::Marker::ADD;
          wall.header.frame_id = "world";
          wall.color.g = 1;
          wall.color.a = 1;

          wall.scale.z = 3;

          wall.scale.x = env_length;
          wall.scale.y = wall_thickness;

          wall.id = 1;
          tf::poseTFToMsg(tf::Transform(openning_center_frame_.getRotation(),
                                        openning_center_frame_ * tf::Vector3(0, -env_width / 2, 0)),
                          wall.pose);
          env_collision_.markers.push_back(wall);
          wall.id = 2;
          tf::poseTFToMsg(tf::Transform(openning_center_frame_.getRotation(),
                                        openning_center_frame_ * tf::Vector3(0, env_width / 2, 0)),
                          wall.pose);
          env_collision_.markers.push_back(wall);


          wall.scale.x = wall_thickness;
          wall.scale.y = env_width / 2 - openning_width / 2;
          wall.id = 3;
          tf::poseTFToMsg(tf::Transform(openning_center_frame_.getRotation(),
                                        openning_center_frame_ * tf::Vector3(0, env_width/4 + openning_width/4, 0)),
                           wall.pose);
          env_collision_.markers.push_back(wall);

          wall.id = 4;
          tf::poseTFToMsg(tf::Transform(openning_center_frame_.getRotation(),
                                        openning_center_frame_ * tf::Vector3(0, -env_width/4 - openning_width/4, 0)),
                          wall.pose);
          env_collision_.markers.push_back(wall);
        }
        // {
        //   /* setup env */
        //   double openning_width, env_width, env_length, openning_yaw;
        //   nhp_.param("openning_width", openning_width, 0.7);
        //   nhp_.param("openning_yaw", openning_yaw, 0.0);
        //   nhp_.param("env_width", env_width, 4.0);
        //   nhp_.param("env_length", env_length, 6.0);

        //   /* openning side wall(s) */
        //   visualization_msgs::Marker wall;
        //   wall.type = visualization_msgs::Marker::CUBE;
        //   wall.action = visualization_msgs::Marker::ADD;
        //   wall.header.frame_id = "world";
        //   wall.color.g = 1;
        //   wall.color.a = 1;

        //   wall.scale.z = 3;

        //   wall.scale.x = env_length;
        //   wall.scale.y = wall_thickness;

        //   wall.id = 1;
        //   tf::poseTFToMsg(tf::Transform(openning_center_frame_.getRotation(),
        //                                 openning_center_frame_ * tf::Vector3(0, -env_width / 2, -0.4)),
        //                   wall.pose);
        //   //env_collision_.markers.push_back(wall);
        //   wall.id = 2;
        //   tf::poseTFToMsg(tf::Transform(openning_center_frame_.getRotation(),
        //                                 openning_center_frame_ * tf::Vector3(0, env_width / 2, -0.4)),
        //                   wall.pose);
        //   env_collision_.markers.push_back(wall);


        //   wall.scale.x = wall_thickness;
        //   wall.scale.y = env_width / 2;
        //   wall.scale.z = 0.5;
        //   wall.id = 3;
        //   tf::poseTFToMsg(tf::Transform(openning_center_frame_.getRotation(),
        //                                 openning_center_frame_ * tf::Vector3(0,  env_width/4, -0.4)),
        //                    wall.pose);
        //   env_collision_.markers.push_back(wall);

        //   wall.id = 4;
        //   tf::poseTFToMsg(tf::Transform(openning_center_frame_.getRotation(),
        //                                 openning_center_frame_ * tf::Vector3(0, -env_width/4, -0.4)),
        //                   wall.pose);
        //   env_collision_.markers.push_back(wall);

        //   wall.scale.y = env_width / 2 - openning_width / 2;
        //   wall.scale.z = 1;
        //   wall.id = 5;
        //   tf::poseTFToMsg(tf::Transform(openning_center_frame_.getRotation(),
        //                                 openning_center_frame_ * tf::Vector3(0, env_width/4 + openning_width/4, 0.75-0.4)),
        //                    wall.pose);
        //   env_collision_.markers.push_back(wall);

        //   wall.id = 6;
        //   tf::poseTFToMsg(tf::Transform(openning_center_frame_.getRotation(),
        //                                 openning_center_frame_ * tf::Vector3(0, -env_width/4 - openning_width/4, 0.75-0.4)),
        //                   wall.pose);
        //   env_collision_.markers.push_back(wall);
          
        //   wall.scale.x = env_length;
        //   wall.scale.y = env_width;
        //   wall.scale.z = wall_thickness;
        //   wall.id = 7;
        //   tf::poseTFToMsg(tf::Transform(openning_center_frame_.getRotation(),
        //                                 openning_center_frame_ * tf::Vector3(0, 0, 1.5-0.4)),
        //                   wall.pose);
        //   env_collision_.markers.push_back(wall);

        // }
      else if(planner_core_ptr_->getMultilinkType() == motion_type::SE3)
        {
          /* setup env */
          double openning_width, env_width, env_length, ceiling_offset;
          nhp_.param("openning_width", openning_width, 0.8);
          nhp_.param("ceiling_offset", ceiling_offset, 0.6);
          nhp_.param("env_width", env_width, 6.0);

          /* openning side wall(s) */
          visualization_msgs::Marker wall;
          wall.type = visualization_msgs::Marker::CUBE;
          wall.action = visualization_msgs::Marker::ADD;
          wall.header.frame_id = "world";
          wall.color.b = 1;
          wall.color.r = 1;
          wall.color.a = 0.8;
          wall.pose.orientation.w = 1;
          wall.scale.z = wall_thickness;

          wall.id = 1;
          tf::poseTFToMsg(tf::Transform(openning_center_frame_.getRotation(),
                                         openning_center_frame_ * tf::Vector3(env_width/4 + openning_width/4, 0, 0)),
                           wall.pose);
          wall.scale.x = env_width / 2 - openning_width / 2;
          wall.scale.y = env_width;
          env_collision_.markers.push_back(wall);

          wall.id = 2;
          tf::poseTFToMsg(tf::Transform(openning_center_frame_.getRotation(),
                                         openning_center_frame_ * tf::Vector3(-env_width/4 - openning_width/4, 0, 0)),
                           wall.pose);
          env_collision_.markers.push_back(wall);

          wall.id = 3;
          tf::poseTFToMsg(tf::Transform(openning_center_frame_.getRotation(),
                                         openning_center_frame_ * tf::Vector3(0, env_width/4 + openning_width/4, 0)),
                           wall.pose);
          wall.scale.x = openning_width;
          wall.scale.y = env_width / 2 - openning_width / 2;
          env_collision_.markers.push_back(wall);

          wall.id = 4;
          tf::poseTFToMsg(tf::Transform(openning_center_frame_.getRotation(),
                                         openning_center_frame_ * tf::Vector3(0, -env_width/4 - openning_width/4, 0)),
                           wall.pose);
          env_collision_.markers.push_back(wall);

          wall.id = 5;
          wall.scale.x = env_width;
          wall.scale.y = env_width;
          tf::poseTFToMsg(tf::Transform(openning_center_frame_.getRotation(),
                                         openning_center_frame_ * tf::Vector3(0, 0, ceiling_offset)),
                           wall.pose);
          env_collision_.markers.push_back(wall);
        }
    }

    bool corePlan()
    {
      /* reset */
      discrete_path_.clear();

      /* declare the differential kinemtiacs const */
      pluginlib::ClassLoader<cost::Base>  cost_plugin_loader("differential_kinematics", "differential_kinematics::cost::Base");
      CostContainer cost_container;

      XmlRpc::XmlRpcValue costs;
      nhp_.getParam("differential_kinematics_cost", costs);

      for(auto cost: costs)
        {
          ROS_INFO_STREAM("squeezing based on differential kinematics, add cost: " << cost.first);

          bool orientation = true;
          cost_container.push_back(cost_plugin_loader.createInstance("differential_kinematics_cost/" + cost.first));

          /* sequeezing cartesian error constraint (cost) */
          if(cost.first == std::string("cartesian_constraint"))
            {
              orientation = false;
              cartersian_constraint_ = boost::reinterpret_pointer_cast<cost::CartersianConstraint>(cost_container.back());
              // NOTE: dynamic_pointer_cast include "undefined symbol" problem, so choose reinterpret_pointer_cast
            }

          cost_container.back()->initialize(nh_, nhp_, planner_core_ptr_, "differential_kinematics_cost/" + cost.first, orientation, true /* full_body */);
        }

      /* defualt reference point is the openning center */
      tf::Transform target_frame = openning_center_frame_;
      tf::Matrix3x3 squeeze_direct = openning_center_frame_.getBasis();
      if (planner_core_ptr_->getMultilinkType() == motion_type::SE2)
        {
          squeeze_direct *= tf::Matrix3x3(tf::createQuaternionFromRPY(0, M_PI/2, 0));
        }
      target_frame.setOrigin(target_frame.getOrigin() + squeeze_direct * tf::Vector3(0, 0, delta_pinch_length_));
      cartersian_constraint_->setTargetFrame(target_frame);
      ROS_WARN("target frame:[%f, %f, %f]", target_frame.getOrigin().x(),
               target_frame.getOrigin().y(), target_frame.getOrigin().z());

      /* declare the differential kinemtiacs constraint */
      pluginlib::ClassLoader<constraint::Base>  constraint_plugin_loader("differential_kinematics", "differential_kinematics::constraint::Base");
      ConstraintContainer constraint_container;

      XmlRpc::XmlRpcValue constraints;
      nhp_.getParam("differential_kinematics_constraint", constraints);
      for(auto constraint: constraints)
        {
          ROS_INFO_STREAM("squeezing based on differential kinematics, add constraint: " << constraint.first);
          constraint_container.push_back(constraint_plugin_loader.createInstance("differential_kinematics_constraint/" +  constraint.first));
          constraint_container.back()->initialize(nh_, nhp_, planner_core_ptr_, "differential_kinematics_constraint/" + constraint.first, true /* orientation */, true /* full_body */);

          /* speical case for collision avoidance */
          if(constraint.first ==  std::string("collision_avoidance"))
            {
              /*-- set collision env --*/
              setCollisionEnv();
              boost::reinterpret_pointer_cast<constraint::CollisionAvoidance>(constraint_container.back())->setEnv(env_collision_);
              // NOTE: dynamic_pointer_cast include "undefined symbol" problem, so choose reinterpret_pointer_cast
            }
        }

      /* reset the init joint(joint) state the init root pose for planner */
      tf::Transform root_pose;
      tf::poseMsgToTF(start_state_.getRootPoseConst(), root_pose);
      planner_core_ptr_->setTargetRootPose(root_pose);
      planner_core_ptr_->setTargetJointVector(start_state_.getJointStateConst());

      /* reset for pinch strategy */
      phase_ = PHASE1;
      reference_point_ratio_ = 1.0;

      /* start the planning */
      if(planner_core_ptr_->solver(cost_container, constraint_container, debug_verbose_))
        {
          /* set the correct base link ( which is not root_link = link1), to be suitable for the control system */
          robot_model_ptr_->setBaselinkName(baselink_name_);

          for(int index = 0; index < planner_core_ptr_->getRootPoseSequence().size(); index++)
            {

              geometry_msgs::Pose root_pose;
              tf::poseKDLToMsg(planner_core_ptr_->getRootPoseSequence().at(index), root_pose);
              discrete_path_.push_back(MultilinkState(robot_model_ptr_, root_pose, planner_core_ptr_->getJointStateSequence().at(index)));
           }

          /* Temporary: add several extra point after squeezing */
          /* simply only change the target altitude of the CoG frame */
          for(int i = 1; i <= 5; i++) // 5 times
            {
              MultilinkState robot_state;
              tf::Transform end_state;
              tf::transformKDLToTF(planner_core_ptr_->getRootPoseSequence().back(), end_state);
              end_state.getOrigin() += openning_center_frame_.getBasis() * tf::Vector3(0, 0, 0.01 * i);
              geometry_msgs::Pose root_pose;
              tf::poseTFToMsg(end_state, root_pose);

              robot_state.setStatesFromRoot(robot_model_ptr_, root_pose,
                                            planner_core_ptr_->getJointStateSequence().back());
              discrete_path_.push_back(robot_state);
            }
          ROS_WARN("total path length: %d", (int)discrete_path_.size());

          cartersian_constraint_.reset(); // necessary for PluginLoader unloading
          return true;
        }

      /* cannot solve */
      cartersian_constraint_.reset(); // necessary for PluginLoader unloading
      return false;
    }

    bool loadPath() { return false;}

    void checkCollision(MultilinkState state) {}

    void visualizeFunc()
    {
      /* collsion publishment */
      env_collision_pub_.publish(env_collision_);
    }

  private:

    ros::Publisher env_collision_pub_;

    boost::shared_ptr<Planner> planner_core_ptr_;

    /* path */
    MultilinkState start_state_;
    MultilinkState goal_state_;

    double delta_pinch_length_; //to propagate the pinch action

    bool debug_;
    Phase phase_;
    double reference_point_ratio_;

    KDL::Chain squeeze_chain_;

    boost::shared_ptr<cost::CartersianConstraint> cartersian_constraint_;
    tf::Transform squeeze_pivot_frame_;

    visualization_msgs::MarkerArray env_collision_;

    bool updatePinchPoint()
    {
      std::string prefix = std::string("[Pinch Strategy][phase") + std::to_string(phase_) + std::string("]");
      if(debug_) ROS_INFO_STREAM(prefix << " update");
      /* phase 1: tail link approaches the openning before squeeze */
      /* phase 2: tail link starts to squeeze openning  */
      /* phase 3: intermediate link squeezes openning */
      /* phase 4: root link close to openning => finish */

      if(phase_ == PHASE4) return true;

      tf::Transform pivot_frame = openning_center_frame_;
      if (planner_core_ptr_->getMultilinkType() == motion_type::SE2)
        {
          pivot_frame.setRotation(pivot_frame.getRotation()
                                  * tf::createQuaternionFromRPY(0, M_PI/2, 0));
        }
 std::cout<<"aa "<<planner_core_ptr_->getTargetRootPose<tf::Transform>().getOrigin().x()<<" "<<planner_core_ptr_->getTargetRootPose<tf::Transform>().getOrigin().y()<<" "<<planner_core_ptr_->getTargetRootPose<tf::Transform>().getOrigin().z()<<std::endl;
          if(phase_ < PHASE3)
      /* check whether can shift PHASE4 */
      if((pivot_frame.inverse() * planner_core_ptr_->getTargetRootPose<tf::Transform>()).getOrigin().z() > -0.01)
        {
         
            {
              ROS_ERROR_STREAM(prefix << " correct phase should be " << PHASE3);
              return false;
            }

          ROS_INFO_STREAM(prefix << " shift to phase 4 to finish squeezing");
          cartersian_constraint_->setReferenceFrame(std::string("link1"), KDL::Frame::Identity());
          phase_ = PHASE4;
          return true;
        }

      /* update robot model (maybe duplicated, but necessary) */
      if (planner_core_ptr_->getMultilinkType() == motion_type::SE3)
        {
          robot_model_ptr_->setCogDesireOrientation(planner_core_ptr_->getTargetRootPose<KDL::Frame>().M);
        }
      robot_model_ptr_->updateRobotModel(planner_core_ptr_->getTargetJointVector<KDL::JntArray>());

      /* fullFK */
      auto full_fk_result = planner_core_ptr_->getRobotModelPtr()->fullForwardKinematics(planner_core_ptr_->getTargetJointVector<KDL::JntArray>());

      tf::Transform prev_seg_tf;
      double prev_seg_z;
      std::string prev_seg_name = squeeze_chain_.segments.front().getName();

      for(const auto& segment: squeeze_chain_.segments)
        {
          tf::Transform curr_seg_tf;
          tf::transformKDLToTF(full_fk_result.at(segment.getName()), curr_seg_tf);
          double curr_seg_z = (pivot_frame.inverse() * planner_core_ptr_->getTargetRootPose<tf::Transform>() * curr_seg_tf).getOrigin().z();

          /* check whether can shift PHASE3 */
          if(curr_seg_z > 0)
            {
              if(phase_ == PHASE1)
                {
                  ROS_ERROR_STREAM(prefix << " correct phase should be " << PHASE2 << " or " << PHASE3);
                  return false;
                }

              if (phase_ = PHASE2) reference_point_ratio_ = 1;

              phase_ = PHASE3;

              // TODO: use reference_point_ratio_,
              //       but we have to reset reference_point_ratio_ once the seg is changed
              double ref_point_ratio = fabs(prev_seg_z) / (fabs(prev_seg_z) + curr_seg_z);
              if(ref_point_ratio < reference_point_ratio_) reference_point_ratio_ = ref_point_ratio;
              double seg_length = (curr_seg_tf.getOrigin() - prev_seg_tf.getOrigin()).length();

              KDL::Frame target_frame = KDL::Frame(KDL::Vector(seg_length * ref_point_ratio, 0, 0));
              cartersian_constraint_->setReferenceFrame(prev_seg_name, target_frame);

              if(debug_)
                {
                  ROS_INFO_STREAM(prefix
                                  << " upper seg: " << segment.getName()
                                  << " cross seg: " << prev_seg_name
                                  << " upper point z: " << curr_seg_z
                                  << " lower point_z: " << prev_seg_z
                                  << " ref_point_ratio: " << ref_point_ratio
                                  << " seg_length: " << seg_length
                                  << " reference_point_ratio_: " << reference_point_ratio_);
                }

              return true;
            }

          prev_seg_tf = curr_seg_tf;
          prev_seg_z = curr_seg_z;
          prev_seg_name = segment.getName();
        }

      /* tail of the robot */
      tf::Transform tail_seg_tf;
      tf::transformKDLToTF(full_fk_result.at(squeeze_chain_.segments.back().getName()),tail_seg_tf);
      tf::Vector3 tail_pos_in_root_link
        = tail_seg_tf * tf::Vector3(robot_model_ptr_->getLinkLength(), 0, 0);
      double tail_z
        = (pivot_frame.inverse()
           * planner_core_ptr_->getTargetRootPose<tf::Transform>() * tail_pos_in_root_link).z();

      if (tail_z < 0)
        {
          /* phase 1 */
          if(phase_ > PHASE1)
            {
              ROS_ERROR_STREAM(prefix << "correct phase should be " <<  PHASE1);
              return false;
            }

          if(debug_) ROS_INFO_STREAM(prefix << " the tail link does not pass");
          ROS_INFO_STREAM(prefix << " tail pos z : " << tail_z);

          std::string target_name = squeeze_chain_.segments.back().getName();
          KDL::Frame target_frame = KDL::Frame(KDL::Vector(robot_model_ptr_->getLinkLength(), 0, 0));
          cartersian_constraint_->setReferenceFrame(target_name, target_frame);
        }
      else
        {
          /* phase 2 */
          if(phase_ > PHASE2)
            {
              ROS_ERROR_STREAM(prefix << "correct phase should be " << PHASE2);
              return false;
            }

          phase_ = PHASE2;

          double ref_point_ratio = fabs(prev_seg_z) / (fabs(prev_seg_z) + tail_z);
          if(ref_point_ratio < reference_point_ratio_) reference_point_ratio_ = ref_point_ratio;

          std::string target_name = squeeze_chain_.segments.back().getName();
          KDL::Frame target_frame = KDL::Frame(KDL::Vector(robot_model_ptr_->getLinkLength() * reference_point_ratio_, 0, 0));
          cartersian_constraint_->setReferenceFrame(target_name, target_frame);
          if(debug_) ROS_INFO_STREAM(prefix << " tail_z: " << tail_z << " previous_z:" << prev_seg_z << " new_ratio: " << ref_point_ratio << " reference_point_ratio_" << reference_point_ratio_);
        }

      return true;
    }

    void setInitState(const MultilinkState& state) { start_state_ = state; }

  };
};

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(squeeze_motion_planner::DifferentialKinematics, squeeze_motion_planner::Base);
