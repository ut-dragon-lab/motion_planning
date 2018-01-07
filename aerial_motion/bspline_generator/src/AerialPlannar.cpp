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

#include <bspline_generator/AerialPlannar.h>

namespace aerial_plannar{
  AerialPlannar::AerialPlannar(ros::NodeHandle nh, ros::NodeHandle nhp){
    nh_ = nh;
    nhp_ = nhp;
    joint_num_ = 3;
    double target_offset_x, target_offset_y;
    start_pose_.resize(3); end_pose_.resize(3);
    nhp_.param("control_frequency", controller_freq_, 100.0);
    nhp_.param("trajectory_period", trajectory_period_, 50.0);
    nhp_.param("bspline_degree", bspline_degree_, 5);
    nhp_.param("auto_takeoff_machine", auto_takeoff_flag_, false);
    nhp_.param("manual_start_state", manual_start_state_flag_, true);
    nhp_.param("target_offset_x", target_offset_x, 4.0);
    nhp_.param("target_offset_y", target_offset_y, 0.0);
    nhp_.param("start_x", start_pose_[0], 0.0);
    nhp_.param("start_y", start_pose_[1], 0.0);
    nhp_.param("start_yaw", start_pose_[2], 0.0);
    target_offset_.setValue(target_offset_x, target_offset_y, 0.0);

    desired_state_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/desired_state", 1);

    aerial_controller_ = boost::shared_ptr<AerialControllerInterface>(new AerialControllerInterface(nh_, nhp_, joint_num_, controller_freq_));

    uav_takeoff_flag_ = false;

    sleep(1.0);

    if (auto_takeoff_flag_){
      aerial_controller_->robot_start();
      ROS_INFO("[AerialPlannar] Published robot start topic.");
      sleep(2.5);
      aerial_controller_->takeoff();
      sleep(18.0); // waiting for finishing taking off
      ROS_INFO("[AerialPlannar] Published takeoff topic.");
    }

    uav_takeoff_flag_ = true;
    spline_generated_flag_ = false;
    move_start_flag_ = false;
    move_topic_recv_flag_ = false;

    endposes_server_ = nh_.advertiseService("endposes_server", &AerialPlannar::getEndposes, this);
    inquiry_robot_state_sub_ = nh_.subscribe<std_msgs::Empty>("/robot_state_inquiry", 1, &AerialPlannar::inquiryRobotStateCallback, this);
    move_start_flag_sub_ = nh_.subscribe<std_msgs::Empty>("/move_start", 1, &AerialPlannar::moveStartCallback, this);
    adjust_initial_state_sub_ = nh_.subscribe<std_msgs::Empty>("/adjust_robot_initial_state", 1, &AerialPlannar::adjustInitalStateCallback, this);

    spline_init_thread_ = boost::thread(boost::bind(&AerialPlannar::splineInitThread, this));
    plannar_timer_ = nh_.createTimer(ros::Duration(1.0 / controller_freq_), &AerialPlannar::plannarCallback, this);
  }

  AerialPlannar::~AerialPlannar(){
    spline_init_thread_.interrupt();
    spline_init_thread_.join();
  }

  void AerialPlannar::splineInitThread(){
    spline_ = boost::shared_ptr<BsplineGenerator>(new BsplineGenerator(nh_, nhp_, trajectory_period_, bspline_degree_));
    spline_generated_flag_ = true;
    ROS_INFO("bspline initalized.");
  }

  bool AerialPlannar::getEndposes(gap_passing::Endposes::Request &req, gap_passing::Endposes::Response &res){
    if (ros::ok && uav_takeoff_flag_){
      res.dim = 6;
      if (!manual_start_state_flag_){
        start_pose_[0] = aerial_controller_->cog_pos_.getX();
        start_pose_[1] = aerial_controller_->cog_pos_.getY();
        start_pose_[2] = aerial_controller_->cog_ang_.getZ();
      }
      for (int i = 0; i < 3; ++i)
        res.start_pose.data.push_back(start_pose_[i]);
      for (int i = 0; i < joint_num_; ++i)
        res.start_pose.data.push_back(aerial_controller_->joints_ang_vec_[i]);

      end_pose_[0] = start_pose_[0] + target_offset_.getX();
      end_pose_[1] = start_pose_[1] + target_offset_.getY();
      end_pose_[2] = start_pose_[2];
      for (int i = 0; i < 3; ++i)
        res.end_pose.data.push_back(end_pose_[i]);
      for (int i = 0; i < joint_num_; ++i)
        res.end_pose.data.push_back(aerial_controller_->joints_ang_vec_[i]);
    }
  }

  void AerialPlannar::inquiryRobotStateCallback(const std_msgs::Empty msg){
    std::cout << "[AerialPlannar] Robot current state: ";
    std::cout << aerial_controller_->cog_pos_.getX() << ", "
              << aerial_controller_->cog_pos_.getY() << ", "
              << aerial_controller_->cog_pos_.getZ() << ", "
              << aerial_controller_->cog_ang_.getZ() << ", ";
    std::cout << "\n[AerialPlannar] Robot current joints: ";
    for (int i = 0; i < joint_num_; ++i)
      std::cout << aerial_controller_->joints_ang_vec_[i] << ", ";
    std::cout << "\n\n";
  }

  void AerialPlannar::adjustInitalStateCallback(const std_msgs::Empty msg){
    std::vector<double> inital_state = spline_->getKeypose(0);
    aerial_controller_->moveToInitialState(inital_state);
  }

  void AerialPlannar::moveStartCallback(const std_msgs::Empty msg){
    move_topic_recv_flag_ = true;
    ROS_INFO("[AerialPlannar] Receive move start topic.");
  }

  void AerialPlannar::plannarCallback(const ros::TimerEvent& event){
    if (move_topic_recv_flag_){
      move_topic_recv_flag_ = false;
      move_start_flag_ = true;
      move_start_time_ = ros::Time::now().toSec();
      return;
    }
    if (move_start_flag_){
      double cur_time = ros::Time::now().toSec() - move_start_time_;
      std::vector<double> des_pos = getDesiredPosition(cur_time + 1.0 / controller_freq_);
      std::vector<double> des_vel = getDesiredVelocity(cur_time);
      std_msgs::Float64MultiArray desired_state;
      for (int i = 0; i < 2; ++i)
        desired_state.data.push_back(des_pos[i]);
      // not publish z axis state
      for (int i = 3; i < des_pos.size(); ++i)
        desired_state.data.push_back(des_pos[i]);
      desired_state_pub_.publish(desired_state);

      aerial_controller_->ff_controller(des_vel, des_pos);
    }
  }

  std::vector<double> AerialPlannar::getDesiredPosition(double time){
    std::vector<double> des_pos;
    des_pos = spline_->getPosition(time);
    return des_pos;
  }

  std::vector<double> AerialPlannar::getDesiredVelocity(double time){
    std::vector<double> des_vel;
    des_vel = spline_->getVelocity(time);
    return des_vel;
  }
}
