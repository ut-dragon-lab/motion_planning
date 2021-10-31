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
#include <std_msgs/Empty.h>
#include <geometry_msgs/Inertia.h>
#include <aerial_robot_model/AddExtraModule.h>

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
  ros::Subscriber start_sub_;
  ros::Subscriber release_sub_;

  int motion_phase_;
  ros::Timer motion_timer_;

  boost::shared_ptr<Dragon::HydrusLikeRobotModel> robot_model_;
  std::unique_ptr<GraspVectoringThrust> planner_;
  geometry_msgs::Inertia object_inertia_;

  void startCallback(const std_msgs::Empty msg);
  void releaseCallback(const std_msgs::Empty msg);
  void stateMachine(const ros::TimerEvent& event);
};
