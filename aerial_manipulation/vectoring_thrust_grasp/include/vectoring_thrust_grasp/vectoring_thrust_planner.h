// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, JSK Lab
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

#pragma once

/* ros */
#include <ros/ros.h>
#include <dragon/GraspVectoringForce.h>

/* robot model */
#include <dragon/dragon_robot_model.h>

/* kinematics */
#include <kdl/treejnttojacsolver.hpp>

/* optimization tools */
#include <OsqpEigen/OsqpEigen.h>
#include <nlopt.hpp>

class GraspVectoringThrust
{
public:
  GraspVectoringThrust(ros::NodeHandle nh, ros::NodeHandle nhp, boost::shared_ptr<DragonRobotModel> robot_model_ptr, bool realtime_control);
  ~GraspVectoringThrust(){}

  bool jointAnglesForQuadDragon(sensor_msgs::JointState& joint_angles); // adhoc function for dragon quad type

  void setRealtimeControl(bool flag) {realtime_control_ = flag; }
  const Eigen::VectorXd& getVectoringForce() const {return vectoring_f_vector_;}

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  ros::Subscriber joint_state_sub_;
  ros::Publisher vectoring_force_pub_;
  boost::shared_ptr<DragonRobotModel> robot_model_ptr_;

  Eigen::VectorXd vectoring_f_vector_;

  bool verbose_;
  bool realtime_control_;

  void jointStatesCallback(const sensor_msgs::JointStateConstPtr& state);

  bool calculateVectoringForce(const sensor_msgs::JointState& joint_angles, bool calculate_maximum_force = false);
  const Eigen::MatrixXd getJacobian(std::string root_link, std::string tip_link, const sensor_msgs::JointState& joint_angles, bool full_body, KDL::Segment additional_frame = KDL::Segment());
};
