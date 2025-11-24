// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2025, DRAGON Lab
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

#ifndef DRAGON_IK_BASE_NAV_H_
#define DRAGON_IK_BASE_NAV_H_

/* ros */
#include <actionlib/server/simple_action_server.h>
#include <ik_base_nav/TargetPoseAction.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <aerial_robot_msgs/FlightNav.h>
#include <moveit_msgs/DisplayRobotState.h>

/* robot model */
#include <dragon/model/hydrus_like_robot_model.h> // TODO: change to full vectoring model
#include <dragon/dragon_navigation.h>
#include <differential_kinematics/motion/end_effector_ik_solver_core.h>

/* utils */
#include <tf/LinearMath/Transform.h>
#include <boost/algorithm/clamp.hpp>


namespace Dragon
{
  class IKBaseNavigator{

  public:
    IKBaseNavigator(ros::NodeHandle nh, ros::NodeHandle nhp);
    ~IKBaseNavigator(){}

  protected:
    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;

    ros::Subscriber baselink_odom_sub_;
    ros::Subscriber joint_states_sub_;

    ros::Publisher joints_ctrl_pub_;
    ros::Publisher flight_nav_pub_;
    ros::Publisher rot_nav_pub_;

    actionlib::SimpleActionServer<ik_base_nav::TargetPoseAction> as_;
    ik_base_nav::TargetPoseFeedback feedback_;
    ik_base_nav::TargetPoseResult result_;

    /* navigation */
    ros::Timer navigate_timer_;
    double controller_freq_;
    bool robot_connect_;
    double nav_start_time_;

    std::vector<double> des_pos_;
    std::vector<double> des_vel_;

    tf::Transform baselink_pose_;
    KDL::JntArray joint_state_;

    std_msgs::ColorRGBA desired_state_color_;

    /* robot model */
    boost::shared_ptr<HydrusRobotModel> robot_model_ptr_;
    /* robot joint limitation */
    std::vector<double> angle_min_vec_;
    std::vector<double> angle_max_vec_;

    /* trajectory */
    boost::shared_ptr<ContinuousPathGenerator> traj_ptr_;

    bool generateTraj(const tf::Transform target_ee_pose, const bool orientation_flag, const std::string tran_free_axis, const std::string rot_free_axis);
    void process(const ros::TimerEvent& event);
    void stopNavigate();
    void sendCommand(const std::vector<double>& des_pos, const std::vector<double>& des_vel);

    void odomCallback(const nav_msgs::OdometryConstPtr& msg);
    void jointStatesCallback(const sensor_msgs::JointStateConstPtr& joint_msg);
    void executeCallback(const ik_base_nav::TargetPoseGoalConstPtr &goal);

  };

};
#endif
