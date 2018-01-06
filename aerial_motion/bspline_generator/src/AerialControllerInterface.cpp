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

#include <bspline_generator/AerialControllerInterface.h>

namespace aerial_controller_interface{
  AerialControllerInterface::AerialControllerInterface(ros::NodeHandle nh, ros::NodeHandle nhp, int joint_num, double controller_freq){
    nh_ = nh;
    nhp_ = nhp;
    joint_num_ = joint_num;
    controller_freq_ = controller_freq;

    state_dim_ = 4 + joint_num_; // x, y, z, yaw, joints

    init_param();

    robot_start_pub_ = nh_.advertise<std_msgs::Empty>("/teleop_command/start", 1);
    takeoff_pub_ = nh_.advertise<std_msgs::Empty>("/teleop_command/takeoff", 1);
    joints_ctrl_pub_ = nh_.advertise<sensor_msgs::JointState>("/hydrusx/joints_ctrl", 1);
    flight_nav_pub_ = nh_.advertise<aerial_robot_base::FlightNav>("/uav/nav", 1);;

    joints_state_sub_ = nh_.subscribe<sensor_msgs::JointState>("/hydrusx/joint_states", 1, &AerialControllerInterface::jointStatesCallback, this);
    baselink_odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/uav/baselink/odom", 1, &AerialControllerInterface::baselinkOdomCallback, this);
    cog_odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/uav/cog/odom", 1, &AerialControllerInterface::cogOdomCallback, this);
    move_start_flag_sub_ = nh_.subscribe<std_msgs::Empty>("/move_start", 1, &AerialControllerInterface::moveStartCallback, this);

    sleep(1.0);
  }

  AerialControllerInterface::~AerialControllerInterface(){
  }

  void AerialControllerInterface::init_param(){
    move_start_flag_ = false;
    nhp_.param("debug_output", debug_, false);

    for (int i = 0; i < joint_num_; ++i){
      joints_ang_vec_.push_back(0.0);
      joints_vel_vec_.push_back(0.0);
    }
  }

  void AerialControllerInterface::robot_start(){
    std_msgs::Empty msg; robot_start_pub_.publish(msg);
  }

  void AerialControllerInterface::takeoff(){
    std_msgs::Empty msg; takeoff_pub_.publish(msg);
  }

  void AerialControllerInterface::ff_controller(std::vector<double> &ff_term, std::vector<double> &target){
    /* pid controller */
    double p_gain = 1.0;
    tf::Vector3 ff_vel(ff_term[0], ff_term[1], ff_term[2]);
    tf::Vector3 target_pos(target[0], target[1], target[2]);
    tf::Vector3 vel;
    vel = ff_vel + p_gain * (target_pos - cog_pos_);

    double target_yaw = target[3] + p_gain * yawDistance(target[3], cog_ang_[2]);
    std::vector<double> target_joints;
    for (int i = 0; i < joint_num_; ++i){
      target_joints.push_back(target[4 + i]
                              + p_gain * (target[4 + i] - joints_ang_vec_[i]));
    }

    if (debug_){
      std::cout << "target yaw: " << target[3] << ", cur yaw: " << cog_ang_[2] << "\n";
      std::cout << "ff_vel: " << ff_vel.getX() << ", "
                << ff_vel.getY() << ", "
                << ff_vel.getZ() << "\n";
      std::cout << "target_pos: " << target_pos.getX() << ", "
                << target_pos.getY() << ", "
                << target_pos.getZ() << "\n";
      std::cout << "cog_pos_: " << cog_pos_.getX() << ", "
                << cog_pos_.getY() << ", "
                << cog_pos_.getZ() << "\n";
      std::cout << "vel: " << vel.getX() << ", "
                << vel.getY() << ", "
                << vel.getZ() << "\n";
    }

    /* publish uav nav to control */
    aerial_robot_base::FlightNav nav_msg;
    nav_msg.header.frame_id = std::string("/world");
    nav_msg.header.stamp = ros::Time::now();
    nav_msg.header.seq = 1;
    nav_msg.control_frame = nav_msg.WORLD_FRAME;
    nav_msg.target = nav_msg.COG;
    nav_msg.pos_xy_nav_mode = nav_msg.VEL_MODE;
    nav_msg.target_vel_x = vel.getX();
    nav_msg.target_vel_y = vel.getY();
    // todo: add control in z axis
    nav_msg.psi_nav_mode = nav_msg.POS_MODE;
    nav_msg.target_psi = target_yaw;
    flight_nav_pub_.publish(nav_msg);

    /* publish joint states */
    sensor_msgs::JointState joints_msg;
    joints_msg.header = nav_msg.header;
    for (int i = 0; i < joint_num_; ++i){
      if (target_joints[i] > PI / 2.0){ // todo: inner collision avoidance
        ROS_ERROR("joint %d angle: %f is larger than PI/2.", i, target_joints[i]);
        target_joints[i] = PI / 2.0;
      }
      joints_msg.position.push_back(target_joints[i]);
    }
    joints_ctrl_pub_.publish(joints_msg);

    if (debug_){
      std::cout << "joints: ";
      for (int i = 0; i < joint_num_; ++i)
        std::cout << joints_msg.position[i] << ", ";
      std::cout << "\n\n";
    }
  }

  void AerialControllerInterface::jointStatesCallback(const sensor_msgs::JointStateConstPtr& joints_msg){
    for (int i = 0; i < joints_msg->position.size(); ++i)
      joints_ang_vec_[i] = joints_msg->position[i];
    for (int i = 0; i < joints_msg->velocity.size(); ++i)
      joints_vel_vec_[i] = joints_msg->velocity[i];
  }

  void AerialControllerInterface::baselinkOdomCallback(const nav_msgs::OdometryConstPtr& odom_msg){
    baselink_odom_ = *odom_msg;
    tf::Quaternion q(odom_msg->pose.pose.orientation.x,
                     odom_msg->pose.pose.orientation.y,
                     odom_msg->pose.pose.orientation.z,
                     odom_msg->pose.pose.orientation.w);
    tf::Matrix3x3 rot_mat;
    rot_mat.setRotation(q);
    tfScalar r,p,y;
    rot_mat.getRPY(r, p, y);
    baselink_ang_.setValue(r, p, y);
    baselink_pos_.setValue(odom_msg->pose.pose.position.x,
                           odom_msg->pose.pose.position.y,
                           odom_msg->pose.pose.position.z);
    baselink_vel_.setValue(odom_msg->twist.twist.linear.x,
                           odom_msg->twist.twist.linear.y,
                           odom_msg->twist.twist.linear.z);
    baselink_w_.setValue(odom_msg->twist.twist.angular.x,
                         odom_msg->twist.twist.angular.y,
                         odom_msg->twist.twist.angular.z);
  }

  void AerialControllerInterface::cogOdomCallback(const nav_msgs::OdometryConstPtr& odom_msg){
    cog_odom_ = *odom_msg;
    tf::Quaternion q(odom_msg->pose.pose.orientation.x,
                     odom_msg->pose.pose.orientation.y,
                     odom_msg->pose.pose.orientation.z,
                     odom_msg->pose.pose.orientation.w);
    tf::Matrix3x3 rot_mat;
    rot_mat.setRotation(q);
    tfScalar r,p,y;
    rot_mat.getRPY(r, p, y);
    cog_ang_.setValue(r, p, y);
    cog_pos_.setValue(odom_msg->pose.pose.position.x,
                      odom_msg->pose.pose.position.y,
                      odom_msg->pose.pose.position.z);
    cog_vel_.setValue(odom_msg->twist.twist.linear.x,
                      odom_msg->twist.twist.linear.y,
                      odom_msg->twist.twist.linear.z);
    cog_w_.setValue(odom_msg->twist.twist.angular.x,
                    odom_msg->twist.twist.angular.y,
                    odom_msg->twist.twist.angular.z);
  }

  void AerialControllerInterface::moveStartCallback(const std_msgs::Empty msg){
    move_start_flag_ = true;
  }

  double AerialControllerInterface::yawDistance(double target_yaw, double cur_yaw){
    if (fabs(cur_yaw - target_yaw) > 3.0){ // jumping gap in yaw angle
      if (cur_yaw > target_yaw){
        while (fabs(cur_yaw - target_yaw) > 3.0){ // Adjust yaw
          cur_yaw -= 2 * PI;
          if (cur_yaw < target_yaw - 2 * PI){ // adjust overhead
            ROS_ERROR("Could not find yaw distance. target yaw: %f, current yaw: %f", target_yaw, cur_yaw);
            cur_yaw += 2 * PI;
            break;
          }
        }
      }
      else{
        while (fabs(cur_yaw - target_yaw) > 3.0){
          cur_yaw += 2 * PI;
          if (cur_yaw > target_yaw + 2 * PI){
            ROS_ERROR("Could not find yaw distance. previous yaw: %f, current yaw: %f", target_yaw, cur_yaw);
            cur_yaw -= 2 * PI;
            break;
          }
        }
      }
    }
    return target_yaw - cur_yaw;
  }

  void AerialControllerInterface::moveToInitialState(std::vector<double> initial_state){
    // intial state containing z axis
    /* publish uav nav */
    aerial_robot_base::FlightNav nav_msg;
    nav_msg.header.frame_id = std::string("/world");
    nav_msg.header.stamp = ros::Time::now();
    nav_msg.header.seq = 1;
    nav_msg.control_frame = nav_msg.WORLD_FRAME;
    nav_msg.target = nav_msg.COG;
    nav_msg.pos_xy_nav_mode = nav_msg.POS_MODE;
    nav_msg.target_pos_x = initial_state[0];
    nav_msg.target_pos_y = initial_state[1];
    // todo: add control in z axis
    nav_msg.psi_nav_mode = nav_msg.POS_MODE;
    nav_msg.target_psi = initial_state[3];
    flight_nav_pub_.publish(nav_msg);

    /* publish joint states */
    sensor_msgs::JointState joints_msg;
    joints_msg.header = nav_msg.header;
    for (int i = 0; i < joint_num_; ++i){
      joints_msg.position.push_back(initial_state[4 + i]);
    }
    joints_ctrl_pub_.publish(joints_msg);
  }
}
