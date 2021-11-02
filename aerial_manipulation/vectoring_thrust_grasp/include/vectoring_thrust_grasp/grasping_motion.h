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
#include <dragon/dragon_navigation.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Inertia.h>
#include <geometry_msgs/WrenchStamped.h>
#include <aerial_robot_msgs/ExtraModule.h>
#include <aerial_robot_msgs/ForceList.h>

/* vectoring force planner */
#include <vectoring_thrust_grasp/vectoring_thrust_planner.h>

enum
  {
    PHASE0, // init phase (hovering)
    PHASE1, // do grasping (change joint angle)
    PHASE2, // hold (give the vectoring force)
    PHASE3, // reserve: realse object
  };

class GraspingMotion
{
public:
  GraspingMotion(ros::NodeHandle nh, ros::NodeHandle nhp);
  ~GraspingMotion(){}

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  ros::Publisher joint_control_pub_;
  ros::Publisher vectoring_force_pub_;
  ros::Publisher extra_module_pub_;
  ros::Subscriber start_sub_;
  ros::Subscriber release_sub_;
  ros::Subscriber external_wrench_sub_;
  ros::Subscriber joy_sub_;
  ros::Subscriber flight_state_sub_;

  int motion_phase_;
  ros::Timer motion_timer_;

  boost::shared_ptr<Dragon::HydrusLikeRobotModel> robot_model_;
  std::unique_ptr<GraspVectoringThrust> planner_;
  geometry_msgs::Inertia object_inertia_;
  geometry_msgs::Vector3 object_offset_;
  geometry_msgs::Wrench estimated_wrench_;
  int flight_state_;

  /* TODO: ad-hoc for quad dragon to grasp object */
  std::vector<int> joints_index_;
  double delta_angle_;
  double contact_torque_thresh_;
  double phase_init_time_;
  double extra_module_comepsention_delay_;
  double estimate_mass_du_;
  bool measure_object_mass_;
  double offset_force_z_;
  bool once_flag_;
  double joint1_init_angle_;
  double joint3_init_angle_;

  void startCallback(const std_msgs::Empty msg);
  void releaseCallback(const std_msgs::Empty msg);
  void estimatedWrenchCallback(const geometry_msgs::WrenchStampedConstPtr msg);
  void joyCallback(const sensor_msgs::Joy::ConstPtr msg);
  void flightStateCallback(const std_msgs::UInt8::ConstPtr msg);
  void stateMachine(const ros::TimerEvent& event);
};
