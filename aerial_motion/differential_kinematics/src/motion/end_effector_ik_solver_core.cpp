// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, JSK Lab
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

#include <differential_kinematics/motion/end_effector_ik_solver_core.h>
/* special cost plugin for cartesian constraint */
#include <differential_kinematics/cost/cartesian_constraint.h>
/* special constraint plugin for collision avoidance */
#include <differential_kinematics/constraint/collision_avoidance.h>


using namespace differential_kinematics;

using CostContainer = std::vector<boost::shared_ptr<cost::Base> >;
using ConstraintContainer = std::vector<boost::shared_ptr<constraint::Base> >;


EndEffectorIKSolverCore::EndEffectorIKSolverCore(ros::NodeHandle nh, ros::NodeHandle nhp, boost::shared_ptr<aerial_robot_model::transformable::RobotModel> robot_model_ptr, bool simulation): nh_(nh), nhp_(nhp), robot_model_ptr_(robot_model_ptr)
  {
    planner_core_ptr_ = boost::shared_ptr<Planner> (new Planner(nh, nhp, robot_model_ptr));

    baselink_name_ = robot_model_ptr_->getBaselinkName();
    nhp_.param("tf_prefix", tf_prefix_, std::string(""));
    nhp_.param("root_link", root_link_, std::string("link1"));

    /* simulation: check the validation of end-effector ik sovler without dynamics */
    if(simulation)
      {
        planner_core_ptr_->registerMotionFunc(std::bind(&EndEffectorIKSolverCore::motionFunc, this));

        XmlRpc::XmlRpcValue init_joint_angle_params;
        nhp_.getParam("zeros", init_joint_angle_params);
        for(auto init_angle: init_joint_angle_params)
          {
            double angle;
            if(init_angle.second.getType() == XmlRpc::XmlRpcValue::TypeInt)
              angle = int(init_angle.second); // workaround for the int type value (e.g. 0)
            else
              angle = double(init_angle.second);

            ROS_INFO_STREAM("find init angle for joint " << init_angle.first << ", " << angle);
            init_joint_vector_.name.push_back(init_angle.first);
            init_joint_vector_.position.push_back(angle);
          }

        end_effector_ik_service_ = nh_.advertiseService("end_effector_ik", &EndEffectorIKSolverCore::endEffectorIkCallback, this);
        env_collision_sub_ = nh_.subscribe("/env_collision", 1, &EndEffectorIKSolverCore::envCollision, this);
      }

    /* continous path generator */
    continuous_path_generator_ = boost::make_shared<ContinuousPathGenerator>(nh_, nhp_, robot_model_ptr_);
  }

bool EndEffectorIKSolverCore::endEffectorIkCallback(differential_kinematics::TargetPose::Request  &req,
                                                    differential_kinematics::TargetPose::Response &res)
{
  collision_avoidance_ = req.collision_avoidance;
  /* start IK */
  tf::Quaternion q; q.setRPY(req.target_rot.x, req.target_rot.y, req.target_rot.z);
  tf::Transform target_ee_pose(q, tf::Vector3(req.target_pos.x, req.target_pos.y, req.target_pos.z));

  tf::Transform init_root_pose;
  init_root_pose.setIdentity();

  /* exmaple of the end effector: a point from the last link origin with distance of getLinkLength() */
  setEndEffectorPose(std::string("link") + std::to_string(planner_core_ptr_->getRobotModelPtr()->getRotorNum()),
                     tf::Transform(tf::createIdentityQuaternion(),
                                   tf::Vector3(planner_core_ptr_->getRobotModelPtr()->getLinkLength(), 0, 0)));

  if(!inverseKinematics(target_ee_pose, init_joint_vector_, init_root_pose,
                        req.orientation, req.full_body, req.tran_free_axis, req.rot_free_axis,
                        req.collision_avoidance, req.debug))
    return false;

  return true;
}

void EndEffectorIKSolverCore::setEndEffectorPose(std::string parent_seg, tf::Transform pose)
{
  parent_seg_ = parent_seg;
  end_effector_relative_pose_ = pose;
}

void EndEffectorIKSolverCore::envCollision(const visualization_msgs::MarkerArrayConstPtr& env_msg)
{
  setCollision(*env_msg);
}

bool EndEffectorIKSolverCore::inverseKinematics(const tf::Transform& target_ee_pose, const sensor_msgs::JointState& init_joint_vector, const tf::Transform& init_root_pose, bool orientation, bool full_body, std::string tran_free_axis, std::string rot_free_axis, bool collision_avoidance, bool debug)
{
  /* reset path */
  discrete_path_.resize(0);

  /* important: link1(root) should be base link */
  planner_core_ptr_->getRobotModelPtr()->setBaselinkName(root_link_);

  /* declare the differential kinemtiacs const */
  pluginlib::ClassLoader<cost::Base>  cost_plugin_loader("differential_kinematics", "differential_kinematics::cost::Base");
  CostContainer cost_container;

  XmlRpc::XmlRpcValue costs;
  nhp_.getParam("differential_kinematics_cost", costs);

  for(auto cost: costs)
    {
      ROS_INFO_STREAM("inverse kinematics, add cost: " << cost.first);

      /* common case: state_vel + end-effector cartesian error constraint (cost) */
      cost_container.push_back(cost_plugin_loader.createInstance("differential_kinematics_cost/" + cost.first));
      cost_container.back()->initialize(nh_, nhp_, planner_core_ptr_, "differential_kinematics_cost/" + cost.first, orientation, full_body);

      /* special process: definition of end coords */
      if(cost.first == std::string("cartesian_constraint"))
        {
          boost::reinterpret_pointer_cast<cost::CartersianConstraint>(cost_container.back())->setReferenceFrame(parent_seg_, end_effector_relative_pose_);
          target_ee_pose_ = target_ee_pose;
          boost::reinterpret_pointer_cast<cost::CartersianConstraint>(cost_container.back())->setTargetFrame(target_ee_pose);

          /* set free axis */
          std::vector<int> free_axis_list(0);
          if(tran_free_axis.find("x") != std::string::npos) free_axis_list.push_back(MaskAxis::TRAN_X);
          if(tran_free_axis.find("y") != std::string::npos) free_axis_list.push_back(MaskAxis::TRAN_Y);
          if(tran_free_axis.find("z") != std::string::npos) free_axis_list.push_back(MaskAxis::TRAN_Z);
          if(rot_free_axis == std::string("x")) free_axis_list.push_back(MaskAxis::ROT_X);
          if(rot_free_axis == std::string("y")) free_axis_list.push_back(MaskAxis::ROT_Y);
          if(rot_free_axis == std::string("z")) free_axis_list.push_back(MaskAxis::ROT_Z);

          if(free_axis_list.size() > 0)
            boost::reinterpret_pointer_cast<cost::CartersianConstraint>(cost_container.back())->setFreeAxis(free_axis_list);
        }
    }

  /* declare the differential kinemtiacs constraint */
  pluginlib::ClassLoader<constraint::Base>  constraint_plugin_loader("differential_kinematics", "differential_kinematics::constraint::Base");
  ConstraintContainer constraint_container;

  XmlRpc::XmlRpcValue constraints;
  nhp_.getParam("differential_kinematics_constraint", constraints);

  for(auto constraint: constraints)
    {
      /* speical case for collision avoidance */
      if(constraint.first ==  std::string("collision_avoidance") && !collision_avoidance)
        continue;

      ROS_INFO_STREAM("inverse kinematics, add constraint: " << constraint.first);
      constraint_container.push_back(constraint_plugin_loader.createInstance("differential_kinematics_constraint/" +  constraint.first));
      constraint_container.back()->initialize(nh_, nhp_, planner_core_ptr_, "differential_kinematics_constraint/" + constraint.first, orientation, full_body);

      /* speical case for collision avoidance */
      if(constraint.first ==  std::string("collision_avoidance"))
        boost::reinterpret_pointer_cast<constraint::CollisionAvoidance>(constraint_container.back())->setEnv(env_collision_);
    }

  /* reset the init joint(joint) state the init root pose for planner */
  planner_core_ptr_->setTargetRootPose(init_root_pose);
  planner_core_ptr_->setTargetJointVector(init_joint_vector);

  /* start the planning */
  if(planner_core_ptr_->solver(cost_container, constraint_container, debug))
    {
      /* revert to the correct base link ( which is not root_link = link1), to be suitable for the control system */
      robot_model_ptr_->setBaselinkName(baselink_name_);

      for(int index = 0; index < planner_core_ptr_->getRootPoseSequence().size(); index++)
        {
          /* insert the result discrete path */
          geometry_msgs::Pose root_pose;
          tf::poseKDLToMsg(planner_core_ptr_->getRootPoseSequence().at(index), root_pose);
          discrete_path_.push_back(MultilinkState(robot_model_ptr_, root_pose, planner_core_ptr_->getJointStateSequence().at(index)));
        }

      if (planner_core_ptr_->getRootPoseSequence().size() == 2)
        {
          discrete_path_.clear();
          ROS_INFO("linear interpolate a short segment only have two points");
          int seg = 5; // TODO: rosparam

          tf::Transform init_root_pose, end_root_pose;
          tf::poseKDLToTF(planner_core_ptr_->getRootPoseSequence().at(0), init_root_pose);
          tf::poseKDLToTF(planner_core_ptr_->getRootPoseSequence().at(1), end_root_pose);
          KDL::JntArray init_joint_vector, end_joint_vector;
          init_joint_vector = planner_core_ptr_->getJointStateSequence().at(0);
          end_joint_vector = planner_core_ptr_->getJointStateSequence().at(1);

          for(int i = 0; i <= seg; i++)
            {
              // interpolation
              double interpolate_rate = (double)i / seg;

              /* joint */
              KDL::JntArray joint_vector = init_joint_vector;
              for(auto index: robot_model_ptr_->getLinkJointIndices())
                joint_vector(index) = init_joint_vector(index) * (1 - interpolate_rate) + interpolate_rate * end_joint_vector(index);
              /* root pose */
              geometry_msgs::Pose root_pose;
              tf::pointTFToMsg(init_root_pose.getOrigin() * (1 - interpolate_rate) + end_root_pose.getOrigin() * interpolate_rate, root_pose.position); // position
              tf::quaternionTFToMsg(init_root_pose.getRotation().slerp(end_root_pose.getRotation(), interpolate_rate), root_pose.orientation); // orientation
              discrete_path_.push_back(MultilinkState(robot_model_ptr_, root_pose, joint_vector));
            }
        }

      return true;
    }

  /* cannot solver */
  return false;
}

void EndEffectorIKSolverCore::calcContinuousPath(double duration)
{
  //TODO: consier the refine of discrete path

  if(discrete_path_.size() == 0)
    {
      ROS_ERROR("[IK solver] no valid discrete path generated");
      return;
    }

  continuous_path_generator_->calcContinuousPath(discrete_path_, duration);
}

void EndEffectorIKSolverCore::motionFunc()
{
  ros::Time now_time = ros::Time::now();
  br_.sendTransform(tf::StampedTransform(target_ee_pose_, now_time, "world", tf::resolve(tf_prefix_, "target_ee")));

  tf::Transform end_link_ee_tf;
  end_link_ee_tf.setIdentity();
  end_link_ee_tf.setOrigin(tf::Vector3(planner_core_ptr_->getRobotModelPtr()->getLinkLength(), 0, 0));

  int rotor_num = planner_core_ptr_->getRobotModelPtr()->getRotorNum();
  br_.sendTransform(tf::StampedTransform(end_link_ee_tf, now_time, tf::resolve(tf_prefix_, std::string("link") + std::to_string(rotor_num)), tf::resolve(tf_prefix_, "ee")));
}
