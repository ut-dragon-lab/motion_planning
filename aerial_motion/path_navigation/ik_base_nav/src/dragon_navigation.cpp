// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2025, DRAGON Lab
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

#include <ik_base_nav/dragon_navigation.h>

using namespace Dragon;

IKBaseNavigator::IKBaseNavigator(ros::NodeHandle nh, ros::NodeHandle nhp):
  nh_(nh), nhp_(nhp),
  as_(nh_, "target_end_effector_pose", boost::bind(&IKBaseNavigator::executeCallback, this, _1), false),
  robot_connect_(false)
{
  /* rosparam */
  nhp_.param("control_frequency", controller_freq_, 40.0);

  /* publisher */
  joints_ctrl_pub_ = nh_.advertise<sensor_msgs::JointState>("joints_ctrl", 1);
  flight_nav_pub_ = nh_.advertise<aerial_robot_msgs::FlightNav>("uav/nav", 1);
  rot_nav_pub_ = nh_.advertise<nav_msgs::Odometry>("target_rotation_motion", 1);

  /* subscriber */
  baselink_odom_sub_ = nh_.subscribe("uav/baselink/odom", 1, &IKBaseNavigator::odomCallback, this);
  joint_states_sub_ = nh_.subscribe("joint_states", 1, &IKBaseNavigator::jointStatesCallback, this);

  /* robot model */
  robot_model_ptr_ = boost::shared_ptr<HydrusRobotModel>(new Dragon::HydrusLikeRobotModel(true));
  /* set the joint angle limit */
  for(auto itr : robot_model_ptr_->getLinkJointNames())
    {
      auto joint_ptr = robot_model_ptr_->getUrdfModel().getJoint(itr);
      assert(joint_ptr != nullptr);

      angle_min_vec_.push_back(joint_ptr->limits->lower);
      angle_max_vec_.push_back(joint_ptr->limits->upper);
    }

  /* start action server */
  as_.start();
}

void IKBaseNavigator::executeCallback(const ik_base_nav::TargetPoseGoalConstPtr &goal)
{
  ros::Rate r(controller_freq_);

  tf::Quaternion q;
  q.setRPY(goal->target_ee_pose.target_rot.x, goal->target_ee_pose.target_rot.y, goal->target_ee_pose.target_rot.z);
  tf::Transform target_ee_pose(q,
                               tf::Vector3(goal->target_ee_pose.target_pos.x,
                                           goal->target_ee_pose.target_pos.y,
                                           goal->target_ee_pose.target_pos.z));

  bool ret = generateTraj(target_ee_pose,
                          goal->target_ee_pose.orientation,
                          goal->target_ee_pose.tran_free_axis,
                          goal->target_ee_pose.rot_free_axis);

  if (!ret)
    {
      ROS_ERROR("cannot solve the trajectory for the goal");
      as_.setPreempted();
      return;
    }

  while(nh_.ok())
    {
      if(as_.isPreemptRequested()){
        if(as_.isNewGoalAvailable()){
          //if we're active and a new goal is available, we'll accept it, but we won't shut anything down
          ik_base_nav::TargetPoseGoal new_goal = *(as_.acceptNewGoal());
          ROS_INFO("got new goal");

          tf::Quaternion q;
          q.setRPY(new_goal.target_ee_pose.target_rot.x,
                   new_goal.target_ee_pose.target_rot.y,
                   new_goal.target_ee_pose.target_rot.z);
          tf::Transform target_ee_pose(q,
                                       tf::Vector3(new_goal.target_ee_pose.target_pos.x,
                                                   new_goal.target_ee_pose.target_pos.y,
                                                   new_goal.target_ee_pose.target_pos.z));

          ret = generateTraj(target_ee_pose, new_goal.target_ee_pose.orientation,
                             new_goal.target_ee_pose.tran_free_axis,
                             new_goal.target_ee_pose.rot_free_axis);

          if (!ret)
            {
              ROS_WARN("cannot solve the trajectory for the goal");
              as_.setPreempted();
              return;
            }

        }
        else {
          //if we've been preempted explicitly we need to shut things down
          ROS_INFO("stop navigate");
          stopNavigate();

          as_.setPreempted();

          //we'll actually return from execute after preempting
          return;
        }
      }

      double cur_time = ros::Time::now().toSec() - nav_start_time_;
      des_pos_ = traj_ptr_->getPositionVector(cur_time + 1.0 / controller_freq_);
      des_vel_ = traj_ptr_->getVelocityVector(cur_time);

      /* send navigation command */
      sendCommand(des_pos_, des_vel_);

      /* check the end of navigation */
      if(cur_time > traj_ptr_->getPathDuration() + 1.0) // margin: 1.0 [sec]
        {
          ROS_INFO("[IKBaseNavigator] Finish Navigation");

          ik_base_nav::TargetPoseResult result;
          result.done = true;
          as_.setSucceeded(result);
          return;
        }


      double comp_rate = cur_time / traj_ptr_->getPathDuration();
      ROS_INFO_THROTTLE(1.0, "complete rate: %f", comp_rate);
      ik_base_nav::TargetPoseFeedback feedback;
      feedback.rate = comp_rate;
      as_.publishFeedback(feedback);

      r.sleep();
    }
}

bool IKBaseNavigator::generateTraj(const tf::Transform target_ee_pose, const bool orientation_flag, const std::string tran_free_axis, const std::string rot_free_axis)
{
  if (!robot_connect_)
    {
      ROS_ERROR("[DRAGON IK Nav] robot is not connected");
      return false;
    }

  /* end effector ik solver */
  EndEffectorIKSolverCore end_effector_ik_solver(nh_, ros::NodeHandle(nhp_, "end_effector"), robot_model_ptr_, false);

  /* set the end of the end link as the end effector */
  std::string end_link_name = std::string("link") + std::to_string(robot_model_ptr_->getRotorNum());
  double link_length = robot_model_ptr_->getLinkLength();
  tf::Transform end_effector_pose_offset(tf::createIdentityQuaternion(), tf::Vector3(link_length, 0, 0));
  end_effector_ik_solver.setEndEffectorPose(end_link_name, end_effector_pose_offset);

  tf::Transform root_pose;
  MultilinkState::convertBaselinkPose2RootPose(robot_model_ptr_, baselink_pose_, joint_state_, root_pose);

  /* get the discrete path from IK */
  if(!end_effector_ik_solver.inverseKinematics(target_ee_pose, robot_model_ptr_->kdlJointToMsg(joint_state_), root_pose, orientation_flag, true, tran_free_axis, rot_free_axis, false, false))
    {
      ROS_ERROR("[DRAGON IK Nav] cannot solve IK ");
      return false;
    }

  /* get continous path */
  end_effector_ik_solver.calcContinuousPath(10.0); // reconfiguration duration
  traj_ptr_ = end_effector_ik_solver.getContinuousPath();

  /* start navigation imediately */
  nav_start_time_ = ros::Time::now().toSec();

  ROS_INFO("[DRAGON IK Nav] start navigation");

  return true;
}

void IKBaseNavigator::stopNavigate()
{
  std::vector<double> stop_pos = des_pos_;
  std::vector<double> stop_vel(des_vel_.size(), 0);

  sendCommand(stop_pos, stop_vel);
}

void IKBaseNavigator::sendCommand(const std::vector<double>& des_pos, const std::vector<double>& des_vel)
{
  /* send general flight navigation command */
  aerial_robot_msgs::FlightNav nav_msg;
  nav_msg.header.frame_id = std::string("world");
  nav_msg.header.stamp = ros::Time::now();
  /* x & y */
  nav_msg.control_frame = nav_msg.WORLD_FRAME;
  nav_msg.target = nav_msg.COG;
  nav_msg.pos_xy_nav_mode = nav_msg.POS_VEL_MODE;
  nav_msg.target_pos_x = des_pos[0];
  nav_msg.target_pos_y = des_pos[1];
  nav_msg.target_vel_x = des_vel[0];
  nav_msg.target_vel_y = des_vel[1];
  /* z axis */
  nav_msg.pos_z_nav_mode = nav_msg.POS_VEL_MODE;
  nav_msg.target_pos_z = des_pos[2];
  nav_msg.target_vel_z = des_vel[2];
  flight_nav_pub_.publish(nav_msg);

  /* se3: rotation motion */
  nav_msgs::Odometry rot_msg;
  rot_msg.header.stamp = ros::Time::now();
  rot_msg.header.frame_id = std::string("baselink");
  rot_msg.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(des_pos[3], des_pos[4], des_pos[5]);
  // omega_body =  [[ 1 0 -sin(pitch)] [0 cos(roll) cos(pitch)sin(roll)] [0 -sin(roll) cos(pitch)cos(roll)]]  * d_rpy
  tf::Vector3 d_rpy(des_vel[3], des_vel[4], des_vel[5]);
  double sin_roll = sin(des_pos[3]);
  double cos_roll = cos(des_pos[3]);
  double sin_pitch = sin(des_pos[4]);
  double cos_pitch = cos(des_pos[4]);
  rot_msg.twist.twist.angular.x = d_rpy.x() - sin_pitch * d_rpy.z();
  rot_msg.twist.twist.angular.y = cos_roll * d_rpy.y() + sin_roll * cos_pitch * d_rpy.z();
  rot_msg.twist.twist.angular.z = -sin_roll * d_rpy.y() + cos_roll * cos_pitch * d_rpy.z();
  rot_nav_pub_.publish(rot_msg);

  /* joint states */
  int joint_num = robot_model_ptr_->getLinkJointIndices().size();
  sensor_msgs::JointState joints_msg;
  joints_msg.header = nav_msg.header;
  for (int i = 0; i < joint_num; ++i)
    {
      joints_msg.name.push_back(robot_model_ptr_->getLinkJointNames().at(i));
      joints_msg.position.push_back(boost::algorithm::clamp(des_pos[6 + i], angle_min_vec_.at(i), angle_max_vec_.at(i)));
    }

  joints_ctrl_pub_.publish(joints_msg);
}

void IKBaseNavigator::odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
  robot_connect_ = true;
  tf::poseMsgToTF(msg->pose.pose, baselink_pose_);
}


void IKBaseNavigator::jointStatesCallback(const sensor_msgs::JointStateConstPtr& joints_msg)
{
  robot_connect_ = true;
  joint_state_ = robot_model_ptr_->jointMsgToKdl(*joints_msg);
}

