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

#ifndef AERIAL_CONTROLLER_INTERFACE_H_
#define AERIAL_CONTROLLER_INTERFACE_H_

/* ros */
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <aerial_robot_base/FlightNav.h>

/* utils */
#include <iostream>
#include <vector>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

namespace aerial_controller_interface{
  #define PI 3.1415926
  class AerialControllerInterface{
  public:
    AerialControllerInterface(ros::NodeHandle nh, ros::NodeHandle nhp, int joint_num, double controller_freq);
    ~AerialControllerInterface();
    void robot_start();
    void takeoff();
    void ff_controller(std::vector<double> &ff_term, std::vector<double> &target);
    void moveToInitialState(std::vector<double> initial_state);

    tf::Vector3 baselink_pos_;
    tf::Vector3 baselink_vel_;
    tf::Vector3 baselink_ang_;
    tf::Vector3 baselink_w_;
    tf::Vector3 cog_pos_;
    tf::Vector3 cog_vel_;
    tf::Vector3 cog_ang_;
    tf::Vector3 cog_w_;
    std::vector<double> joints_ang_vec_;
    std::vector<double> joints_vel_vec_;

  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;
    double controller_freq_;

    int state_dim_;
    int joint_num_;
    nav_msgs::Odometry baselink_odom_;
    nav_msgs::Odometry cog_odom_;

    bool debug_;
    bool move_start_flag_;

    ros::Publisher robot_start_pub_;
    ros::Publisher takeoff_pub_;
    ros::Publisher joints_ctrl_pub_;
    ros::Publisher flight_nav_pub_;

    ros::Subscriber joints_state_sub_;
    ros::Subscriber baselink_odom_sub_;
    ros::Subscriber cog_odom_sub_;
    ros::Subscriber move_start_flag_sub_;

    void init_param();
    void jointStatesCallback(const sensor_msgs::JointStateConstPtr& joints_msg);
    void baselinkOdomCallback(const nav_msgs::OdometryConstPtr& odom_msg);
    void cogOdomCallback(const nav_msgs::OdometryConstPtr& odom_msg);
    void moveStartCallback(const std_msgs::Empty msg);
    double yawDistance(double target_yaw, double cur_yaw);
  };
}
#endif
