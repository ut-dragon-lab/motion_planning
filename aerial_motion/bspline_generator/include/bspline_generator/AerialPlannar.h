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
#include <std_msgs/Float64MultiArray.h>

/* utils */
#include <iostream>
#include <vector>

/* local */
#include <bspline_generator/AerialControllerInterface.h>
#include <bspline_generator/BsplineGenerator.h>

/* thread */
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>

using namespace aerial_controller_interface;
using namespace bspline_generator;

namespace aerial_plannar{
  class AerialPlannar{
  public:
    AerialPlannar(ros::NodeHandle nh, ros::NodeHandle nhp);
    ~AerialPlannar();

  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;
    ros::ServiceServer endposes_server_;
    std::vector<double> start_pose_;
    std::vector<double> end_pose_;
    ros::Publisher desired_state_pub_;

    bool auto_takeoff_flag_;
    bool manual_start_state_flag_;

    int joint_num_;
    bool uav_takeoff_flag_;
    bool spline_generated_flag_;
    bool move_start_flag_;
    bool move_topic_recv_flag_;
    boost::thread spline_init_thread_;
    double controller_freq_;
    double trajectory_period_;
    int bspline_degree_;
    ros::Timer plannar_timer_;
    double move_start_time_;
    tf::Vector3 target_offset_;

    boost::shared_ptr<AerialControllerInterface> aerial_controller_;
    boost::shared_ptr<BsplineGenerator> spline_;

    ros::Subscriber inquiry_robot_state_sub_;
    ros::Subscriber move_start_flag_sub_;
    ros::Subscriber adjust_initial_state_sub_;

    bool getEndposes(gap_passing::Endposes::Request &req, gap_passing::Endposes::Response &res);
    void splineInitThread();
    void inquiryRobotStateCallback(const std_msgs::Empty msg);
    void adjustInitalStateCallback(const std_msgs::Empty msg);
    void moveStartCallback(const std_msgs::Empty msg);
    void plannarCallback(const ros::TimerEvent& event);
    std::vector<double> getDesiredPosition(double time);
    std::vector<double> getDesiredVelocity(double time);
  };
}
#endif
