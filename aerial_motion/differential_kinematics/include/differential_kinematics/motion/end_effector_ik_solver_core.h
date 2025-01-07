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

#ifndef END_EFFECTOR_IK_SOLVER_CORE_H
#define END_EFFECTOR_IK_SOLVER_CORE_H

#include <differential_kinematics/planner_core.h>

#include <pluginlib/class_loader.h>
/* rosservice for target end-effector pose */
#include <visualization_msgs/MarkerArray.h>
#include <differential_kinematics/TargetPose.h>
#include <aerial_motion_planning_msgs/multilink_state.h>
#include <aerial_motion_planning_msgs/continuous_path_generator.h>
#include <kdl_conversions/kdl_msg.h>

using namespace differential_kinematics;

class EndEffectorIKSolverCore
{
public:
  EndEffectorIKSolverCore(ros::NodeHandle nh, ros::NodeHandle nhp, boost::shared_ptr<aerial_robot_model::transformable::RobotModel> robot_model_ptr, bool simulation);
  ~EndEffectorIKSolverCore(){}

  const std::string getParentSegName() const {return parent_seg_;}
  const tf::Transform getEndEffectorRelativePose() const {return end_effector_relative_pose_;}
  const std::vector<MultilinkState>& getDiscretePath() const { return discrete_path_;}
  virtual const MultilinkState& getDiscreteState(int index) const { return discrete_path_.at(index); }

  /* continuous path */
  const boost::shared_ptr<ContinuousPathGenerator> getContinuousPath() const { return continuous_path_generator_;}
  const double getPathDuration() const { return continuous_path_generator_->getPathDuration();}
  const std::vector<double> getPositionVector(double t) { return continuous_path_generator_->getPositionVector(t); }
  const std::vector<double> getVelocityVector(double t) { return continuous_path_generator_->getVelocityVector(t); }


  void setEndEffectorPose(std::string parent_seg, tf::Transform pose);
  void setCollision(const visualization_msgs::MarkerArray& env_collision)
  {
    env_collision_ = env_collision;
  }

  bool inverseKinematics(const tf::Transform& target_ee_pose, const sensor_msgs::JointState& init_joint_vector, const tf::Transform& init_root_pose, bool orientation, bool full_body, std::string tran_free_axis, std::string rot_free_axis, bool collision_avoidance, bool debug);

  void calcContinuousPath(double duration);

private:

  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  ros::ServiceServer end_effector_ik_service_;
  ros::Subscriber env_collision_sub_;
  tf::TransformBroadcaster br_;

  boost::shared_ptr<aerial_robot_model::transformable::RobotModel> robot_model_ptr_;
  std::string baselink_name_;
  std::string root_link_;
  std::string tf_prefix_;
  std::string parent_seg_;
  tf::Transform end_effector_relative_pose_;

  boost::shared_ptr<Planner> planner_core_ptr_;
  std::vector<MultilinkState> discrete_path_;
  boost::shared_ptr<ContinuousPathGenerator> continuous_path_generator_;
  tf::Transform target_ee_pose_;
  sensor_msgs::JointState init_joint_vector_;

  /* collision avoidance */
  bool collision_avoidance_;
  visualization_msgs::MarkerArray env_collision_;

  bool endEffectorIkCallback(differential_kinematics::TargetPose::Request  &req,
                             differential_kinematics::TargetPose::Response &res);

  void envCollision(const visualization_msgs::MarkerArrayConstPtr& env_msg);
  void motionFunc();

};

#endif
