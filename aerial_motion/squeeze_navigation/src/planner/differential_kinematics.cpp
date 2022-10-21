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
    CASE1,
    CASE2_1,
    CASE2_2,
    CASE3,
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

    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp, boost::shared_ptr<aerial_robot_model::RobotModel> robot_model_ptr)
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
      phase_ = CASE1;
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
      ROS_WARN("model: %d", planner_core_ptr_->getMultilinkType());

      /* get opening center frame from rosparam */
      nhp_.param("openning_pos_x", pose.position.x, 0.0);
      nhp_.param("openning_pos_y", pose.position.y, 0.0);
      nhp_.param("openning_pos_z", pose.position.z, 0.0);
      nhp_.param("openning_roll", r, 0.0);
      nhp_.param("openning_pitch", p, 0.0);
      nhp_.param("openning_yaw", y, 0.0);

      tf::pointMsgToTF(pose.position, openning_center_frame_.getOrigin());
      openning_center_frame_.setRotation(tf::createQuaternionFromRPY(r, p, y));

      env_collision_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/env_collision", 1);
    };


    const tf::Transform& getOpenningCenterFrame() const { return openning_center_frame_;}
    void setOpenningCenterFrame(const tf::Transform& openning_center_frame)
    {
      ROS_ERROR("set openning center");
      openning_center_frame_ = openning_center_frame;
    }

    void setCollisionEnv()
    {
      /* hard-coding to set env */
      double wall_thickness;
      nhp_.param("wall_thickness", wall_thickness, 0.05);
      if(planner_core_ptr_->getMultilinkType() == motion_type::SE2)
        {
          /* setup env */
          double openning_width, env_width, env_length;
          nhp_.param("openning_width", openning_width, 0.8);
          nhp_.param("env_width", env_width, 4.0);
          nhp_.param("env_length", env_length, 6.0);
          /* openning side wall(s) */
          visualization_msgs::Marker wall;
          wall.type = visualization_msgs::Marker::CUBE;
          wall.action = visualization_msgs::Marker::ADD;
          wall.header.frame_id = "/world";
          wall.color.g = 1;
          wall.color.a = 1;

          wall.scale.z = 2;

          wall.id = 1;
          tf::poseTFToMsg(tf::Transform(openning_center_frame_.getRotation(),
                                         openning_center_frame_ * tf::Vector3(0, -env_width / 2, 0)),
                           wall.pose);
          wall.scale.x = env_length;
          wall.scale.y = wall_thickness;
          env_collision_.markers.push_back(wall);

          wall.id = 2;
          tf::poseTFToMsg(tf::Transform(openning_center_frame_.getRotation(),
                                         openning_center_frame_ * tf::Vector3(0, env_width / 2, 0)),
                           wall.pose);
          env_collision_.markers.push_back(wall);

          wall.id = 3;
          tf::poseTFToMsg(tf::Transform(openning_center_frame_.getRotation(),
                                         openning_center_frame_ * tf::Vector3(0, env_width/4 + openning_width/4, 0)),
                           wall.pose);
          wall.scale.x = wall_thickness;
          wall.scale.y = env_width / 2 - openning_width / 2;
          env_collision_.markers.push_back(wall);

          wall.id = 4;
          tf::poseTFToMsg(tf::Transform(openning_center_frame_.getRotation(),
                                         openning_center_frame_ * tf::Vector3(0, -env_width/4 - openning_width/4, 0)),
                           wall.pose);
          env_collision_.markers.push_back(wall);

          double r,p,y;
          openning_center_frame_.getBasis().getRPY(r,p,y);
          if(fabs(p) != M_PI / 2)
            {
              ROS_ERROR("SE2 collision env: the orientation of openinig center frame is invliad: [%f, %f, %f]", r,p,y);
              env_collision_.markers.clear();
            }
        }
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
          wall.header.frame_id = "/world";
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
      target_frame.setOrigin(target_frame.getOrigin() + openning_center_frame_.getBasis() * tf::Vector3(0, 0, delta_pinch_length_));
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

      /* init the pinch point, shouch be the end point of end link */
      updatePinchPoint();

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

          return true;
        }

      /* cannot solve */
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


    tf::Transform openning_center_frame_;

    double delta_pinch_length_; //to propagate the pinch action

    bool debug_;
    Phase phase_;
    double reference_point_ratio_;

    KDL::Chain squeeze_chain_;

    boost::shared_ptr<cost::CartersianConstraint> cartersian_constraint_;

    visualization_msgs::MarkerArray env_collision_;

    bool updatePinchPoint()
    {
      if(debug_) ROS_INFO("update the pinch point ");
      /* case 1: total not pass: use the head */
      /* case 2: half pass: calcualte the pass point */
      /* case 3: final phase: the root link is close to the openning_center gap => finish */

      /* case3 */
      if((openning_center_frame_.inverse() * planner_core_ptr_->getTargetRootPose<tf::Transform>()).getOrigin().z() > -0.01)
        {
          if(phase_ < CASE2_2)
            {
              ROS_ERROR("CurrentPhase: %d, wrong phase:%d, ", phase_, CASE1);
              return false;
            }

          if(debug_) ROS_INFO("case 3");
          /* convergence */
          ROS_WARN("complete passing regarding to the cartersian constraint");
          cartersian_constraint_->setReferenceFrame(std::string("link1"), KDL::Frame::Identity());
          return true;
        }

      /* update robot model (maybe duplicated, but necessary) */
      robot_model_ptr_->setCogDesireOrientation(planner_core_ptr_->getTargetRootPose<KDL::Frame>().M);
      robot_model_ptr_->updateRobotModel(planner_core_ptr_->getTargetJointVector<KDL::JntArray>());

      /* fullFK */
      auto full_fk_result = planner_core_ptr_->getRobotModelPtr()->fullForwardKinematics(planner_core_ptr_->getTargetJointVector<KDL::JntArray>());

      /* check the cross situation: case2-2 */
      tf::Transform prev_seg_tf;
      double prev_seg_z;
      std::string prev_seg_name = squeeze_chain_.segments.front().getName();

      for(const auto& segment: squeeze_chain_.segments)
        {
          tf::Transform curr_seg_tf;
          tf::transformKDLToTF(full_fk_result.at(segment.getName()), curr_seg_tf);
          double curr_seg_z = (openning_center_frame_.inverse() * planner_core_ptr_->getTargetRootPose<tf::Transform>() * curr_seg_tf).getOrigin().z();
          if(curr_seg_z > 0)
            {
              /* case 2.2 */
              if(phase_ > CASE2_2 || phase_ == CASE1)
                {
                  ROS_ERROR("CurrentPhase: %d, wrong phase:%d, ", phase_, CASE1);
                  return false;
                }

              if(phase_ == CASE2_1) reference_point_ratio_ = 1;
              phase_ = CASE2_2;

              double ref_point_ratio = fabs(prev_seg_z) / (fabs(prev_seg_z) + curr_seg_z);
              if(ref_point_ratio < reference_point_ratio_) reference_point_ratio_ = ref_point_ratio;
              double seg_length = (curr_seg_tf.getOrigin() - prev_seg_tf.getOrigin()).length();
              if(debug_) ROS_INFO("case 2.2, upper seg: %s, cross seg: %s, upper point z: %f, lower point_z: %f, ref_point_ratio: %f, seg_length: %f, reference_point_ratio_: %f", segment.getName().c_str(), prev_seg_name.c_str(), curr_seg_z, prev_seg_z, ref_point_ratio, seg_length, reference_point_ratio_);

              assert(segment.getName() !=  prev_seg_name);

              cartersian_constraint_->setReferenceFrame(prev_seg_name, KDL::Frame(KDL::Vector(seg_length * ref_point_ratio , 0, 0)));

              return true;
            }

          prev_seg_tf = curr_seg_tf;
          prev_seg_z = curr_seg_z;
          prev_seg_name = segment.getName();
        }

      /* head of the robot */
      tf::Transform tail_seg_tf;
      tf::transformKDLToTF(full_fk_result.at(squeeze_chain_.segments.back().getName()),tail_seg_tf);
      tf::Vector3 tail_pos_in_root_link; tail_pos_in_root_link = tail_seg_tf * tf::Vector3(robot_model_ptr_->getLinkLength(), 0, 0);
      double tail_z = (openning_center_frame_.inverse() * planner_core_ptr_->getTargetRootPose<tf::Transform>() *  tail_pos_in_root_link).z();

      if (tail_z < 0)  /* case 1 */
        {
          if(phase_ > CASE1)
            {
              ROS_ERROR("CurrentPhase: %d, wrong phase:%d", phase_, CASE1);
              return false;
            }
          if(debug_) ROS_INFO("case 1, the tail does not pass");

          cartersian_constraint_->setReferenceFrame(squeeze_chain_.segments.back().getName(), KDL::Frame(KDL::Vector(robot_model_ptr_->getLinkLength(), 0, 0)));

        }
      else /* case 2.1 */
        {
          if(phase_ > CASE2_1)
            {
              ROS_ERROR("CurrentPhase: %d, wrong phase:%d", phase_, CASE1);
              return false;
            }
          phase_ = CASE2_1;

          double ref_point_ratio = fabs(prev_seg_z) / (fabs(prev_seg_z) + tail_z);
          if(debug_) ROS_INFO("case 2.1, tail_z: %f, previous_z: %f, new_ratio: %f, reference_point_ratio_: %f", tail_z, prev_seg_z, ref_point_ratio, reference_point_ratio_);
          if(ref_point_ratio < reference_point_ratio_) reference_point_ratio_ = ref_point_ratio;

          cartersian_constraint_->setReferenceFrame(squeeze_chain_.segments.back().getName(), KDL::Frame(KDL::Vector(robot_model_ptr_->getLinkLength() * reference_point_ratio_, 0, 0)));
        }

      return true;
    }

    void setInitState(const MultilinkState& state) { start_state_ = state; }

  };
};

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(squeeze_motion_planner::DifferentialKinematics, squeeze_motion_planner::Base);
