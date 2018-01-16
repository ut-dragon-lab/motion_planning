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

#ifndef AERIAL_PLANNAR_H_
#define AERIAL_PLANNAR_H_

/* ros */
#include <ros/ros.h>
#include <gap_passing/Endposes.h>
#include <gap_passing/Keyposes.h>
#include <gap_passing/PlanningMode.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/JointState.h>
#include <aerial_robot_base/FlightNav.h>
#include <aerial_robot_base/DesireCoord.h>

/* continous path generator */
#include <bspline_generator/tinyspline_interface.h>

/* utils */
#include <tf/LinearMath/Transform.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>
#include <iostream>
#include <vector>
#include <boost/algorithm/clamp.hpp>

class AerialPlannar{
public:
  AerialPlannar(ros::NodeHandle nh, ros::NodeHandle nhp);
  ~AerialPlannar();

  std::vector<double> getKeypose(int id);

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  ros::Subscriber inquiry_robot_state_sub_;
  ros::Subscriber move_start_flag_sub_;
  ros::Subscriber adjust_initial_state_sub_;
  ros::Publisher desired_state_pub_;
  ros::Publisher joints_ctrl_pub_;
  ros::Publisher flight_nav_pub_;
  ros::Publisher se3_roll_pitch_nav_pub_;

  int joint_num_;
  int motion_type_;
  bool move_start_flag_;

  bool debug_verbose_;
  double controller_freq_;
  double trajectory_period_;
  int bspline_degree_;
  ros::Timer navigate_timer_;
  double move_start_time_;
  double joint_upperbound_;
  double joint_lowerbound_;

  boost::shared_ptr<TinysplineInterface> bspline_ptr_;
  bspline_generator::ControlPoints* control_pts_ptr_;

  boost::thread spline_init_thread_;
  void splineInitThread();

  void navigate(const ros::TimerEvent& event);
  void waitForKeyposes();

  void moveStartCallback(const std_msgs::Empty msg);
  void adjustInitalStateCallback(const std_msgs::Empty msg);

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
