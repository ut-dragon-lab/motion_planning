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
#include <moveit_msgs/RobotState.h>
#include <tf_conversions/tf_kdl.h>

/* robot model */
#include <aerial_robot_model/model/transformable_aerial_robot_model.h>

/* utils */
#include <algorithm>

namespace motion_type
{
  enum {SE2 = 0, SE3 = 1,};
};

// utils
double generateContinousEulerAngle(double ang, double prev_ang);

class MultilinkState
{
public:
  MultilinkState() {}

  MultilinkState(boost::shared_ptr<aerial_robot_model::RobotModel> robot_model_ptr):
    cog_pose_(), root_pose_(), cog_twist_(), gimbal_module_flag_(false)
  {
    joint_index_map_ = robot_model_ptr->getJointIndexMap();
    joint_state_.resize(joint_index_map_.size());

    /* TODO: hard-coding */
    for(auto tree_itr : robot_model_ptr->getTree().getSegments())
      {
        std::string joint_name = tree_itr.second.segment.getJoint().getName();
        if(joint_name.find("gimbal") == 0 &&
           (joint_name.find("roll") != std::string::npos ||
            joint_name.find("pitch") != std::string::npos) &&
           tree_itr.second.segment.getJoint().getType() != KDL::Joint::JointType::None)
          {
            gimbal_module_flag_ = true;
            break;
          }
      }

  }

  MultilinkState(boost::shared_ptr<aerial_robot_model::RobotModel> robot_model_ptr,
                        const tf::Quaternion& baselink_desired_att,
                        const geometry_msgs::Pose& cog_pose,
                        const KDL::JntArray& joint_state): MultilinkState(robot_model_ptr)
  {
    setStatesFromCog(robot_model_ptr, baselink_desired_att, cog_pose, joint_state);
  }

  MultilinkState(boost::shared_ptr<aerial_robot_model::RobotModel> robot_model_ptr,
                 const geometry_msgs::Pose& root_pose,
                 const KDL::JntArray& joint_state): MultilinkState(robot_model_ptr)
  {
    setStatesFromRoot(robot_model_ptr, root_pose, joint_state);
  }

  ~MultilinkState(){}

  void setStatesFromCog(boost::shared_ptr<aerial_robot_model::RobotModel> robot_model_ptr,
                        const tf::Quaternion& baselink_desired_att,
                        const geometry_msgs::Pose& cog_pose,
                        const KDL::JntArray& joint_state);

  void setStatesFromRoot(boost::shared_ptr<aerial_robot_model::RobotModel> robot_model_ptr,
                        const geometry_msgs::Pose& root_pose,
                        const KDL::JntArray& joint_state);


  static void convertCogPose2RootPose(boost::shared_ptr<aerial_robot_model::RobotModel> robot_model_ptr,
                               const tf::Quaternion& baselink_desired_att,
                               const geometry_msgs::Pose& cog_pose,
                               const KDL::JntArray& joint_state,
                               geometry_msgs::Pose& root_pose);


  static void convertRootPose2CogPose(boost::shared_ptr<aerial_robot_model::RobotModel> robot_model_ptr,
                               const geometry_msgs::Pose& root_pose,
                               const KDL::JntArray& joint_state,
                               tf::Quaternion& baselink_desired_att,
                               geometry_msgs::Pose& cog_pose);



  static void convertBaselinkPose2RootPose(boost::shared_ptr<aerial_robot_model::RobotModel> robot_model_ptr,
                                    const geometry_msgs::Pose& baselink_pose,
                                    const KDL::JntArray& joint_state,
                                    geometry_msgs::Pose& root_pose);


  inline geometry_msgs::Pose& getCogPoseNonConst() { return cog_pose_; }
  const geometry_msgs::Pose& getCogPoseConst() const { return cog_pose_; }
  void setCogPose(const geometry_msgs::Pose& cog_pose) { cog_pose_ = cog_pose; }

  geometry_msgs::Pose& getRootPoseNonConst() { return root_pose_; }
  const geometry_msgs::Pose& getRootPoseConst() const { return root_pose_; }
  void setRootPose(const geometry_msgs::Pose& root_pose) { root_pose_ = root_pose; }
  void setRootPosition(const geometry_msgs::Point& root_position) { root_pose_.position = root_position; }
  void  setRootOrientation(const geometry_msgs::Quaternion& root_orientation) { root_pose_.orientation = root_orientation; }

  const geometry_msgs::Twist& getCogTwistConst() const { return cog_twist_; }
  void setCogTwist(const geometry_msgs::Twist& cog_twist)  { cog_twist_ = cog_twist; }

  const KDL::JntArray& getJointStateConst() const { return joint_state_; }
  void setJointState(const KDL::JntArray& joint_state)
  {
    assert(joint_state.rows() == joint_state_.rows());
    joint_state_ = joint_state;
  }

  const tf::Quaternion& getBaselinkDesiredAttConst() const { return baselink_desired_att_; }
  void setBaselinkDesiredAtt(const tf::Quaternion& baselink_desired_att) { baselink_desired_att_ = baselink_desired_att; }

  [[deprecated("please use setStatesFromRoot() function")]]
  void setRootJointState(std::vector<double> states)
  {
    assert(states.size() == 7 + joint_index_map_.size());

    root_pose_.position.x = states.at(0);
    root_pose_.position.y = states.at(1);
    root_pose_.position.z = states.at(2);

    root_pose_.orientation.x = states.at(3);
    root_pose_.orientation.y = states.at(4);
    root_pose_.orientation.z = states.at(5);
    root_pose_.orientation.w = states.at(6);

    joint_state_.data = Eigen::Map<Eigen::VectorXd>(states.data() + 7, joint_state_.rows());
  }

  template<class T> const T getRootJointStateConst() const;

private:
  geometry_msgs::Pose cog_pose_, root_pose_;
  geometry_msgs::Twist cog_twist_;
  KDL::JntArray joint_state_;
  tf::Quaternion baselink_desired_att_;
  std::map<std::string, uint32_t> joint_index_map_;

  bool gimbal_module_flag_;
};

#endif
