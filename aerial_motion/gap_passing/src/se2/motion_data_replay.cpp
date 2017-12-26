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

#include <gap_passing/se2/motion_control.h>
#include <gap_passing/se2/motion_data_replay.h>

namespace se2
{
  MotionDataReplay::MotionDataReplay(ros::NodeHandle nh, ros::NodeHandle nhp) :nh_(nh), nhp_(nhp)
  {
    transform_controller_ = boost::shared_ptr<TransformController>(new TransformController(nh_, nhp_, false));
    joint_num_ = transform_controller_->getRotorNum() - 1;
    transform_controller_->modelling();

    move_start_flag_ = false;

    rosParamInit();

    motion_control_ = new MotionControl(nh, nhp, transform_controller_);

    robot_move_start_sub_ = nh_.subscribe<std_msgs::Empty>("/move_start", 1, &MotionDataReplay::moveStartCallback, this);

    /* experiment data replay */
    experiment_scene_diff_pub_ = nh_.advertise<moveit_msgs::PlanningScene>("experiment_scene", 1);
    experiment_robot_cog_odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/uav/cog/odom", 1, &MotionDataReplay::experimentRobotOdomCallback, this);
    experiment_robot_joint_states_sub_ = nh_.subscribe<sensor_msgs::JointState>("/hydrusx/joint_states", 1, &MotionDataReplay::experimentRobotJointStatesCallback, this);

    while(experiment_scene_diff_pub_.getNumSubscribers() < 1)
      {
        ros::WallDuration sleep_t(0.5);
        sleep_t.sleep();
      }
    ros::Duration sleep_time(1.0);
    sleep_time.sleep();

    robot_model_loader_ = new robot_model_loader::RobotModelLoader("robot_description");
    kinematic_model_ = robot_model_loader_->getModel();
    experiment_scene_ = new planning_scene::PlanningScene(kinematic_model_);

    motion_sequence_timer_ = nhp_.createTimer(ros::Duration(1.0 / motion_sequence_rate_), &MotionDataReplay::motionSequenceFunc, this);
  }

  MotionDataReplay::~MotionDataReplay()
  {
    delete robot_model_loader_;
    delete experiment_scene_;
  }

  void MotionDataReplay::motionSequenceFunc(const ros::TimerEvent &e)
  {
    if(ros::ok())
      {
        /* experiment data replay */
        if (experiment_odom_recv_flag_ && experiment_joint_states_recv_flag_){
          robot_state::RobotState& replay_robot_state = experiment_scene_->getCurrentStateNonConst();
          std::vector<double> replay_curr_state(3 + joint_num_ + 7); //se(2) + joint_num + 7
          std::vector<double> replay_cog_state;
          replay_cog_state.push_back(experiment_robot_cog_odom_.pose.pose.position.x);
          replay_cog_state.push_back(experiment_robot_cog_odom_.pose.pose.position.y);
          // yaw
          tf::Quaternion q(experiment_robot_cog_odom_.pose.pose.orientation.x,
                           experiment_robot_cog_odom_.pose.pose.orientation.y,
                           experiment_robot_cog_odom_.pose.pose.orientation.z,
                           experiment_robot_cog_odom_.pose.pose.orientation.w);
          tf::Matrix3x3 rot_mat;
          rot_mat.setRotation(q);
          tfScalar r,p,y;
          rot_mat.getRPY(r, p, y);
          replay_cog_state.push_back(y);
          for (int i = 0; i < joint_num_; ++i) // joints
            replay_cog_state.push_back(experiment_robot_joint_states_.position[i]);

          std::vector<double> replay_root_state = cog2root(replay_cog_state);
          for (int i = 0; i < 3 + joint_num_; ++i)
            replay_curr_state[i] = replay_root_state[i];
          replay_robot_state.setVariablePositions(replay_curr_state);

          experiment_scene_->getPlanningSceneMsg(experiment_scene_msg_);
          experiment_scene_msg_.is_diff = true;
          experiment_scene_diff_pub_.publish(experiment_scene_msg_);
        }

      }
  }

  std::vector<double> MotionDataReplay::cog2root(std::vector<double> &keypose)
  {
    sensor_msgs::JointState joint_state;
    for(int i = 0; i < joint_num_; i++)
      {
        std::stringstream ss;
        ss << i + 1;
        joint_state.name.push_back(std::string("joint") + ss.str());
      }
    joint_state.position.resize(0);
    for (int j = 0; j < joint_num_; ++j)
      joint_state.position.push_back(keypose[3 + j]);
    transform_controller_->kinematics(joint_state);
    transform_controller_->modelling();
    tf::Transform cog_root = transform_controller_->getCog(); // cog in root frame
    // delete
    tf::Quaternion q_cog = cog_root.getRotation();
    tf::Matrix3x3  cog_mat(q_cog);
    tfScalar cog_r, cog_p, cog_y;
    cog_mat.getRPY(cog_r, cog_p, cog_y);
    std::cout << "cog_root: " << cog_r << ", "
              << cog_p << ", "
              << cog_y << "\n";

    tf::Transform root_cog = cog_root.inverse();
    tf::Transform cog_world;
    cog_world.setOrigin(tf::Vector3(keypose[0],
                                    keypose[1],
                                    0.0));
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, keypose[2]);
    cog_world.setRotation(q);
    tf::Transform root_world = cog_world * root_cog;

    std::vector<double> keypose_root;
    keypose_root.push_back(root_world.getOrigin().getX());
    keypose_root.push_back(root_world.getOrigin().getY());
    q = root_world.getRotation();
    tf::Matrix3x3  rot_mat(q);
    tfScalar e_r, e_p, e_y;
    rot_mat.getRPY(e_r, e_p, e_y);
    keypose_root.push_back(e_y);
    for (int i = 0; i < joint_num_; i++)
      keypose_root.push_back(keypose[3 + i]);
    return keypose_root;
  }

  std::vector<double> MotionDataReplay::root2cog(std::vector<double> &keypose)
  {
    sensor_msgs::JointState joint_state;
    for(int i = 0; i < joint_num_; i++)
      {
        std::stringstream ss;
        ss << i + 1;
        joint_state.name.push_back(std::string("joint") + ss.str());
      }
    joint_state.position.resize(0);
    for (int j = 0; j < joint_num_; ++j)
      joint_state.position.push_back(keypose[3 + j]);
    transform_controller_->kinematics(joint_state);
    tf::Transform cog_root = transform_controller_->getCog();
    tf::Transform root_world;
    root_world.setOrigin(tf::Vector3(keypose[0],
                                     keypose[1],
                                     0.0));
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, keypose[2]);
    root_world.setRotation(q);
    tf::Transform cog_world = root_world * cog_root;

    std::vector<double> keypose_root;
    keypose_root.push_back(cog_world.getOrigin().getX());
    keypose_root.push_back(cog_world.getOrigin().getY());
    q = cog_world.getRotation();
    tf::Matrix3x3  rot_mat(q);
    tfScalar e_r, e_p, e_y;
    rot_mat.getRPY(e_r, e_p, e_y);
    keypose_root.push_back(e_y);
    for (int i = 0; i < joint_num_; i++)
      keypose_root.push_back(keypose[3 + i]);
    return keypose_root;
  }

  void MotionDataReplay::rosParamInit()
  {
    nhp_.param("simulator", simulator_, true);
    nhp_.param("replay_experiment_data", replay_experiment_data_flag_, false);

    nhp_.param("gap_left_x", gap_left_x_, 1.0);
    nhp_.param("gap_left_y", gap_left_y_, 0.3);
    nhp_.param("gap_x_offset", gap_x_offset_, 0.6); //minus: overlap
    nhp_.param("gap_y_offset", gap_y_offset_, 0.0); //minus: overlap
    nhp_.param("gap_left_width", gap_left_width_, 0.3); //minus: bandwidth
    nhp_.param("gap_right_width", gap_right_width_, 0.3); //minus: bandwidth

    //nhp_.param("coefficient_rate", coefficient_rate_, 0.035);
    nhp_.param("ompl_mode", ompl_mode_, 0); //RRT_START_MODE
    nhp_.param("planning_mode", planning_mode_, 0); //ONLY_JOINTS_MODE
    nhp_.param("motion_sequence_rate", motion_sequence_rate_, 10.0);

    start_state_.resize(3 + joint_num_);
    nhp_.param("start_state_x", start_state_[0], 0.0);
    nhp_.param("start_state_y", start_state_[1], 0.5);
    nhp_.param("start_state_theta", start_state_[2], 0.785);
    goal_state_.resize(3 + joint_num_);
    nhp_.param("goal_state_x", goal_state_[0], 3.0);
    nhp_.param("goal_state_y", goal_state_[1], 0.5);
    nhp_.param("goal_state_theta", goal_state_[2], 0.785);

    for(int i = 0; i < joint_num_; i++)
      {
        std::stringstream joint_no;
        joint_no << i + 1;

        nhp_.param(std::string("start_state_joint") + joint_no.str(), start_state_[3 + i], 0.0);
        nhp_.param(std::string("goal_state_joint") + joint_no.str(), goal_state_[3 + i], 0.0);
      }

    nhp_.param("real_robot_move_base", real_robot_move_base_, false);

    nhp_.param("state_validity_check_res", state_validity_check_res_, 0.03);
    nhp_.param("valid_segment_count_factor", valid_segment_count_factor_,20);

    nhp_.param("solving_time_limit", solving_time_limit_, 3600.0);

    nhp_.param("length_opt_weight", length_opt_weight_, 1.0);
    nhp_.param("stability_opt_weight", stability_opt_weight_, 0.0);

    nhp_.param("stability_cost_thre", stability_cost_thre_, 100000000.0);
    nhp_.param("length_cost_thre", length_cost_thre_, 0.0);

    nhp_.param("play_log_path", play_log_path_, false);
    if(play_log_path_) solved_ = true;

    experiment_odom_recv_flag_ = false;
    experiment_joint_states_recv_flag_ = false;
  }

  void MotionDataReplay::moveStartCallback(const std_msgs::Empty msg){
    move_start_flag_ = true;
  }

  void MotionDataReplay::experimentRobotOdomCallback(const nav_msgs::OdometryConstPtr& msg){
    experiment_odom_recv_flag_ = true;
    experiment_robot_cog_odom_ = *msg;
  }

  void MotionDataReplay::experimentRobotJointStatesCallback(const sensor_msgs::JointStateConstPtr& joints_msg){
    experiment_joint_states_recv_flag_ = true;
    experiment_robot_joint_states_ = *joints_msg;
  }
}
