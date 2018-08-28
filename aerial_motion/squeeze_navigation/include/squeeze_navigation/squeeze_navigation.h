// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, JSK Lab
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

#ifndef SQUEEZE_NAVIGATION_H_
#define SQUEEZE_NAVIGATION_H_

/* ros */
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/JointState.h>
#include <aerial_robot_msgs/FlightNav.h>
#include <spinal/DesireCoord.h>
#include <spinal/FlightConfigCmd.h>
#include <moveit_msgs/DisplayRobotState.h>

/* robot model */
#include <dragon/transform_control.h>

/* discrete path search */
#include <sampling_based_method/se3/motion_planning.h>

/* continous path generator */
#include <bspline_generator/tinyspline_interface.h>

/* utils */
#include <tf/LinearMath/Transform.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>
#include <iostream>
#include <vector>
#include <boost/algorithm/clamp.hpp>


namespace motion_type
{
  enum {SE2 = 0, SE3 = 1,};
};


class SqueezeNavigation{
public:
  SqueezeNavigation(ros::NodeHandle nh, ros::NodeHandle nhp);
  ~SqueezeNavigation(){}

  std::vector<double> getKeypose(int id);

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  ros::Subscriber plan_start_flag_sub_;
  ros::Subscriber move_start_flag_sub_;
  ros::Subscriber adjust_initial_state_sub_;
  ros::Subscriber flight_config_sub_;

  ros::Subscriber robot_baselink_odom_sub_;
  ros::Subscriber robot_joint_states_sub_;

  ros::Publisher joints_ctrl_pub_;
  ros::Publisher flight_nav_pub_;
  ros::Publisher se3_roll_pitch_nav_pub_;
  ros::Publisher desired_path_pub_;

  bool debug_verbose_;
  bool headless_;

  /* discrete path */
  int discrete_path_search_method_type_;
  bool discrete_path_debug_flag_;
  bool load_path_flag_;
  int motion_type_;

  /* continuous path */
  int bspline_degree_;
  double trajectory_period_;

  /* navigation */
  ros::Timer navigate_timer_;
  bool move_start_flag_;
  double move_start_time_;
  double controller_freq_;

  /* robot model */
  boost::shared_ptr<TransformController> robot_model_ptr_;
  int joint_num_;

  /* discrete path search */
  // 1. sampling based method
  boost::shared_ptr<sampling_base::se2::MotionPlanning> sampling_base_planner_;


  /* continuous path generator */
  boost::shared_ptr<TinysplineInterface> bspline_ptr_;
  boost::shared_ptr<bspline_generator::ControlPoints> control_pts_ptr_;

  void rosParamInit();
  void navigate(const ros::TimerEvent& event);
  void continuousPath(const std::vector<MultilinkState>& discrete_path);

  void planStartCallback(const std_msgs::Empty msg);
  void moveStartCallback(const std_msgs::Empty msg);
  void adjustInitalStateCallback(const std_msgs::Empty msg);
  void flightConfigCallback(const spinal::FlightConfigCmdConstPtr msg);
  void robotOdomCallback(const nav_msgs::OdometryConstPtr& msg);
  void robotJointStatesCallback(const sensor_msgs::JointStateConstPtr& joint_msg);

  double generateContinousEulerAngle(double ang, int id)
  {
    static double prev_ang = 0.0;
    double new_ang = ang;
    if (id == 0){
      prev_ang = new_ang;
    }
    else{
      if (fabs(new_ang - prev_ang) > M_PI){ // jumping gap in ang angle
        ROS_WARN("[bspline debug] find the angle gap in index %d, prev: %f, current: %f", id, prev_ang, new_ang);
        if (new_ang > prev_ang){
          while (fabs(new_ang - prev_ang) > M_PI){ // Adjust ang
            new_ang -= 2 * M_PI;
            if (new_ang < prev_ang - 2 * M_PI){ // adjust overhead
              ROS_ERROR("Could not find suitable ang. previous ang: %f, current ang: %f", prev_ang, ang);
              new_ang += 2 * M_PI;
              break;
            }
          }
        }
        else{
          while (fabs(new_ang - prev_ang) > M_PI){
            new_ang += 2 * M_PI;
            if (new_ang > prev_ang + 2 * M_PI){
              ROS_ERROR("Could not find suitable ang. previous ang: %f, current ang: %f", prev_ang, ang);
              new_ang -= 2 * M_PI;
              break;
            }
          }
        }
      }
      prev_ang = new_ang;
    }
    return new_ang;
  }
};

#endif
