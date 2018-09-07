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

#ifndef MULTILINK_STATE_H
#define MULTILINK_STATE_H

/* ros */
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_datatypes.h>
#include <moveit_msgs/RobotState.h>

/* robot model */
#include <hydrus/transform_control.h>
#include <dragon/transform_control.h>


/* utils */
#include <algorithm>

namespace motion_type
{
  enum {SE2 = 0, SE3 = 1,};
};

class MultilinkState
{
public:
  MultilinkState():
    cog_update_(false), root_update_(false), actuator_update_(false),
    cog_pose_(), root_pose_(), cog_twist_(), root_twist_(), actuator_state_() {}
  ~MultilinkState(){}

  void cogPose2RootPose(boost::shared_ptr<TransformController> robot_model_ptr);
  void baselinkPose2RootPose(geometry_msgs::Pose baselink_pose, boost::shared_ptr<TransformController> robot_model_ptr);
  void targetRootPose2TargetBaselinkPose(boost::shared_ptr<TransformController> robot_model_ptr);

  geometry_msgs::Pose& getCogPoseNonConst()
  {
    assert(cog_update_);
    return cog_pose_;
  }

  const geometry_msgs::Pose& getCogPoseConst() const
  {
    assert(cog_update_);
    return cog_pose_;
  }

  void  setCogPose(geometry_msgs::Pose cog_pose)
  {
    cog_pose_ = cog_pose;
    cog_update_ = true;
  }

  geometry_msgs::Pose& getRootPoseNonConst()
  {
    assert(root_update_);
    return root_pose_;
  }

  const geometry_msgs::Pose& getRootPoseConst() const
  {
    assert(root_update_);
    return root_pose_;
  }

  void setRootPose(geometry_msgs::Pose root_pose)
  {
    root_pose_ = root_pose;
    root_update_ = true;
  }

  void setRootPosition(geometry_msgs::Point root_position)
  {
    root_pose_.position = root_position;
  }

  void  setRootOrientation(geometry_msgs::Quaternion root_orientation)
  {
    root_pose_.orientation = root_orientation;
  }


  const geometry_msgs::Twist& getCogTwistConst() const { return cog_twist_; }
  void  setCogTwist(geometry_msgs::Twist cog_twist)  { cog_twist_ = cog_twist; }

  const geometry_msgs::Twist& getRootTwistConst() const { return root_twist_; }
  void  setRootTwist(geometry_msgs::Twist root_twist)  { root_twist_ = root_twist; }

  const sensor_msgs::JointState& getActuatorStateConst() const
  {
    assert(actuator_update_);
    return actuator_state_;
  }

  sensor_msgs::JointState& getActuatorStateNonConst()
  {
    assert(actuator_update_);
    return actuator_state_;
  }

  void setActuatorState(sensor_msgs::JointState actuator_state)
  {
    actuator_state_ = actuator_state;
    actuator_update_ = true;
  }

  void setActuatorState(int index, double state)
  {
    actuator_state_.position.at(index) = state;
  }

  const tf::Quaternion& getBaselinkDesiredAttConst() const
  {
    assert(baselink_desired_att_update_);
    return baselink_desired_att_;
  }

  void setBaselinkDesiredAtt(tf::Quaternion baselink_desired_att)
  {
    baselink_desired_att_ = baselink_desired_att;
    baselink_desired_att_update_ = true;
  }

  const std::vector<double> getRootActuatorStateConst() const;
  void setRootActuatorState(const std::vector<double>& states);

  const moveit_msgs::RobotState getVisualizeRobotStateConst() const;

private:

  bool cog_update_, root_update_, actuator_update_, baselink_desired_att_update_;
  geometry_msgs::Pose cog_pose_, root_pose_;
  geometry_msgs::Twist cog_twist_, root_twist_;
  sensor_msgs::JointState actuator_state_;
  tf::Quaternion baselink_desired_att_;

};

#endif
