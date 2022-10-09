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
#include <geometry_msgs/QuaternionStamped.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <aerial_robot_msgs/FlightNav.h>
#include <aerial_robot_msgs/PoseControlPid.h>
#include <spinal/DesireCoord.h>
#include <spinal/FlightConfigCmd.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <gazebo_msgs/ApplyBodyWrench.h>
#include <gazebo_msgs/BodyRequest.h>

/* robot model */
#include <dragon/model/hydrus_like_robot_model.h> // TODO: change to full vectoring model
#include <dragon/dragon_navigation.h>

/* discrete path search */
#include <pluginlib/class_loader.h>
#include <squeeze_navigation/planner/base_plugin.h>

/* continous path generator */
#include <kalman_filter/lpf_filter.h>
#include <bspline_ros/bspline_ros.h>

/* utils */
#include <tf/LinearMath/Transform.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>
#include <iostream>
#include <vector>
#include <boost/algorithm/clamp.hpp>

enum
  {
    PHASE0, // init phase (hovering)
    PHASE1, // approach under the contact point on the cover (need planning based on differential kinematics)
    PHASE2, // contact to the cover with external wrench (consider the weight of the cover)
    PHASE3, // move the cover (need planning based on differential kinematics)
    PHASE4, // move away from the cover
    PHASE5, // squeeze (need planning based on differential kinematics)
  };


class SqueezeNavigation{
public:
  SqueezeNavigation(ros::NodeHandle nh, ros::NodeHandle nhp);
  ~SqueezeNavigation(){}

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  ros::Subscriber plan_start_flag_sub_;
  ros::Subscriber move_start_flag_sub_;
  ros::Subscriber return_flag_sub_;
  ros::Subscriber adjust_initial_state_sub_;
  ros::Subscriber flight_config_sub_;
  ros::Subscriber phase_up_sub_;
  ros::Subscriber robot_baselink_odom_sub_;
  ros::Subscriber robot_joint_states_sub_;
  ros::Subscriber controller_debug_sub_;
  ros::Subscriber joy_stick_sub_;

  ros::Publisher joints_ctrl_pub_;
  ros::Publisher flight_nav_pub_;
  ros::Publisher rot_nav_pub_;
  ros::Publisher desired_path_pub_;
  ros::Publisher end_effector_pos_pub_;

  bool debug_verbose_;
  bool headless_;

  int motion_phase_;
  bool first_time_in_new_phase_;
  bool teleop_flag_;

  /* discrete path */
  int discrete_path_search_method_type_;
  bool discrete_path_debug_flag_;
  int motion_type_;

  /* continuous path */
  int bspline_degree_;
  std::vector<double> angle_min_vec_;
  std::vector<double> angle_max_vec_;

  /* navigation */
  ros::Timer navigate_timer_;
  double move_start_time_;
  bool move_start_flag_;
  bool return_flag_;
  bool replay_;
  double controller_freq_;
  double return_delay_;

  /* robot model */
  boost::shared_ptr<HydrusRobotModel> robot_model_ptr_;

  /* discrete path search */
  boost::shared_ptr<squeeze_motion_planner::Base> discrete_path_planner_;
  std::vector<MultilinkState> discrete_path_;

  /* continuous path generator */
  boost::shared_ptr<BsplineRos> bspline_ptr_;

  void rosParamInit();
  void stateMachine(const ros::TimerEvent& event);
  void pathNavigate();

  const std::vector<MultilinkState> discretePathSmoothing(const std::vector<MultilinkState>& raw_path) const ;
  const std::vector<MultilinkState> discretePathResampling(const std::vector<MultilinkState>& raw_path) const;
  void continuousPath(const std::vector<MultilinkState>& discrete_path, double trajectory_period);

  /* robot real state */
  void robotOdomCallback(const nav_msgs::OdometryConstPtr& msg);
  void robotJointStatesCallback(const sensor_msgs::JointStateConstPtr& joint_msg);
  void controlDebugCallback(const aerial_robot_msgs::PoseControlPidConstPtr& control_msg);

  /* teleop & debug */
  void moveStartCallback(const std_msgs::Empty msg);
  void returnCallback(const std_msgs::Empty msg);
  void phaseUpCallback(const std_msgs::Empty msg);
  void adjustInitalStateCallback(const std_msgs::Empty msg);
  void planSqueezeMotionCallback(const std_msgs::Empty msg);
  void joyStickControl(const sensor_msgs::JoyConstPtr & joy_msg);

  void reset()
  {
    move_start_flag_ = false;
    return_flag_ = false;
    motion_phase_ = PHASE0;
    first_time_in_new_phase_ = true;
    discrete_path_.resize(0);
  }

  void startNavigate()
  {
    if(replay_) return;

    move_start_flag_ = true;
    move_start_time_ = ros::Time::now().toSec();
  }

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
