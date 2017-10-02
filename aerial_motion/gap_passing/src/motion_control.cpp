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

/*
TODO: the log saving and loading for more than 4 links
 */

#include <gap_passing/motion_control.h>

MotionControl::MotionControl(ros::NodeHandle nh, ros::NodeHandle nhp, boost::shared_ptr<TransformController>  transform_controller): nh_(nh), nhp_(nhp)
{
  nhp_.param("log_flag", log_flag_, false);
  nhp_.param("play_log_path", play_log_path_, false);

  nhp_.param("robot_type", robot_type_, 0); //0: hydrus; 1: dragon

  nhp_.param("joint_cmd_rate", joint_cmd_rate_, 50.0);
  nhp_.param("move_cmd_rate", move_cmd_rate_, 40.0);

  nhp_.param("backward_offset", backward_offset_, 100);
  nhp_.param("forward_offset", forward_offset_, 100);

  if(play_log_path_) log_flag_ = false;
  nhp_.param("file_name", file_name_, std::string("planning_log.txt"));

  //subscriber
  control_flag_sub_ = nh_.subscribe<std_msgs::UInt8>("hydrus/motion_control", 1, &MotionControl::controlFlagCallback, this, ros::TransportHints().tcpNoDelay());

  joint_values_sub_ = nh_.subscribe<sensor_msgs::JointState>("joint_states", 1, &MotionControl::jointStateCallback, this, ros::TransportHints().tcpNoDelay());

  //publisher
  joint_cmd_pub_ = nh_.advertise<sensor_msgs::JointState>("hydrus/joints_ctrl", 1);
  /*TODO*/
  move_cmd_pub_ = nh_.advertise<aerial_robot_base::FlightNav>("hoge", 1);

  //init
  link_num_ = transform_controller->getRotorNum();

  planning_path_.resize(0);
  planning_mode_ = gap_passing::PlanningMode::ONLY_JOINTS_MODE;

  min_var_ = 1e6;
  min_var_state_ = 0;
  semi_stable_states_ = 0;

  control_index_ = 0;

  if(play_log_path_)
    {
      planFromFile();
    }
  else
    {
      transform_controller_ = transform_controller;
    }

  control_flag_ = false;

  //control sub thread
  if(planning_mode_ != gap_passing::PlanningMode::ONLY_BASE_MODE)
    {
      joint_cmd_thread_ = boost::thread(boost::bind(&MotionControl::jointCmd, this));
    }
  if(planning_mode_ != gap_passing::PlanningMode::ONLY_JOINTS_MODE)
    move_cmd_thread_ = boost::thread(boost::bind(&MotionControl::moveCmd, this));
}

MotionControl::~MotionControl()
{
  if(planning_mode_ != gap_passing::PlanningMode::ONLY_BASE_MODE)
    {
      joint_cmd_thread_.interrupt();
      joint_cmd_thread_.join();
    }
  if(planning_mode_ != gap_passing::PlanningMode::ONLY_JOINTS_MODE)
    {
      move_cmd_thread_.interrupt();
      move_cmd_thread_.join();
    }
}


void MotionControl::setJointStates(std::vector<double> joint_states)
{
/*
  boost::lock_guard<boost::mutex> lock(real_state_mutex_);

  if(real_states_.size() == 0)
    real_states_.resize(joint_states.size());
  for(int i = 0; i <  (int)joint_states.size(); i ++)
    real_states_[i + 3] = joint_states[i];
*/
}


void MotionControl::planStoring(const ompl::base::StateStorage* plan_states, int planning_mode, const std::vector<double>& start_state, const std::vector<double>& goal_state,  double best_cost, double calculation_time)
{
  best_cost_ = best_cost;
  calculation_time_ = calculation_time;
  planning_mode_ = planning_mode;

  conf_values state;
  //use start state to initialize the state
  state.state_values.resize(start_state.size());
  state.state_values = start_state;
  state.stable_mode = TransformController::LQI_FOUR_AXIS_MODE;
  state.dist_thre_value = 1;
  state.control_mode = gap_passing::PlanningMode::POSITION_MODE;

  int state_list = (int)plan_states->size();

  for(int i = 0; i < state_list; i++)
    {
      if(planning_mode == gap_passing::PlanningMode::ONLY_JOINTS_MODE || planning_mode == gap_passing::PlanningMode::JOINTS_AND_BASE_MODE)
        {
          if(planning_mode == gap_passing::PlanningMode::ONLY_JOINTS_MODE)
            {
              for(int j = 0; j < link_num_ - 1; j++)
                state.state_values[3 + j] = plan_states->getState(i)->as<ompl::base::RealVectorStateSpace::StateType>()->values[j];
            }
          else if(planning_mode == gap_passing::PlanningMode::JOINTS_AND_BASE_MODE)
            {
              const ompl::base::CompoundState* state_tmp = dynamic_cast<const ompl::base::CompoundState*>(plan_states->getState(i));
              state.state_values[0] = state_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->getX();
              state.state_values[1] = state_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->getY();
              state.state_values[2] = state_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->getYaw();
              for(int j = 0; j < link_num_ - 1; j++)
                state.state_values[3 + j] = state_tmp->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[j];
            }

          // dist thre
          sensor_msgs::JointState joint_state;
          if(robot_type_ == HYDRUS)
            for(int j = 0; j < link_num_ - 1; j++)
              {
                std::stringstream ss;
                ss << j + 1;
                joint_state.name.push_back(std::string("joint") + ss.str());
                joint_state.position.push_back(state.state_values[3 + j]);
              }
          transform_controller_->kinematics(joint_state);
          float min_var = transform_controller_->distThreCheck();
          if(min_var == 0)
            {
              state.dist_thre_value = 0;
              ROS_ERROR("(Invalid pose, the distance is not enough");
            }
          else state.dist_thre_value = 1;

           if(min_var < min_var_)
             {
               min_var_ = min_var;
               min_var_state_ = i;
             }

          if(!transform_controller_->modelling())
            {//semi stable
              semi_stable_states_ ++;
              state.stable_mode = TransformController::LQI_THREE_AXIS_MODE;
            }
          else
            state.stable_mode = TransformController::LQI_FOUR_AXIS_MODE;

          //stability
          /*
          ROS_INFO("validate: [%f, %f, %f, %f, %f], min var: %f",
                   joint_state.position[0],
                   joint_state.position[1],
                   joint_state.position[2],
                   joint_state.position[3],
                   joint_state.position[4],
                   min_var_);
          */
        }
      else if(planning_mode == gap_passing::PlanningMode::ONLY_BASE_MODE)
        {
          state.state_values[0] = plan_states->getState(i)->as<ompl::base::SE2StateSpace::StateType>()->getX();
          state.state_values[1] = plan_states->getState(i)->as<ompl::base::SE2StateSpace::StateType>()->getY();
          state.state_values[2] = plan_states->getState(i)->as<ompl::base::SE2StateSpace::StateType>()->getYaw();
        }
      //interation
      planning_path_.push_back(state);
    }

  //file log
  if(log_flag_)
    {
      std::ofstream ofs;
      ofs.open( "planning_log.txt" );
      ofs << "start_state: " << start_state[0] << " " <<start_state[1] << " " << 
        start_state[2] << " " << start_state[3] <<  " " << start_state[4] <<
        " " << start_state[5] << std::endl;
      ofs << "goal_state: " << goal_state[0] << " " <<goal_state[1] << " " <<
        goal_state[2] << " " << goal_state[3] << " " <<goal_state[4] << " " << 
        goal_state[5] << std::endl;

      ofs << "states: " << state_list  << std::endl;
      ofs << "planning_mode: " << planning_mode << std::endl;
      ofs << "planning_time: " << calculation_time << std::endl;
      ofs << "motion_cost: " << best_cost << std::endl;
      ofs << "minimum_var: " << min_var_ << std::endl;
      ofs << "minimum_var_state_entry: " << min_var_state_  << std::endl;
      ofs << "semi_stable_states: " << semi_stable_states_  << std::endl;

      for(int k = 0; k < state_list;  k++)
        {
          ofs << "state" << k << ": " << planning_path_[k].state_values[0] << " "
              << planning_path_[k].state_values[1] << " " << planning_path_[k].state_values[2]
              << " " <<planning_path_[k].state_values[3] << " " <<planning_path_[k].state_values[4]
              << " " <<planning_path_[k].state_values[5] << " " <<planning_path_[k].stable_mode
              << " " <<planning_path_[k].dist_thre_value << std::endl;
        }
      ofs << "end"  << std::endl;
      ofs.close();
    }
}

void MotionControl::planFromFile()
{
  std::ifstream ifs(file_name_.c_str());

  if(ifs.fail()) 
    {
      ROS_ERROR("File do not exist");
      return;
    }

  //hard code
  std::vector<double> start_state(6,0);
  std::vector<double> goal_state(6,0);
  int state_list;
  std::stringstream ss[11];
  std::string str;
  std::string header;
  //1 start and goal state
  std::getline(ifs, str);
  ss[0].str(str);
  ss[0] >> header >> start_state[0] >> start_state[1] >> start_state[2] 
     >> start_state[3] >> start_state[4] >> start_state[5];
  std::cout << header << std::endl;
  std::getline(ifs, str);
  ss[1].str(str);
  ss[1] >> header >> goal_state[0] >> goal_state[1] >> goal_state[2] 
     >> goal_state[3] >> goal_state[4] >> goal_state[5];
  std::cout << header << std::endl;
  ROS_WARN("from (%f, %f, %f, %f, %f, %f) to (%f, %f, %f, %f, %f, %f)",
           start_state[0], start_state[1], start_state[2],
           start_state[3], start_state[4], start_state[5],
           goal_state[0], goal_state[1], goal_state[2],
           goal_state[3], goal_state[4], goal_state[5]);

  //states size, planning time, motion cost
  std::getline(ifs, str);
  ss[2].str(str);
  ss[2] >> header >> state_list;
  std::cout << header << state_list <<std::endl;
  std::getline(ifs, str);
  ss[3].str(str);
  ss[3] >> header >> planning_mode_;
  std::cout << header << planning_mode_ <<std::endl;
  std::getline(ifs, str);
  ss[4].str(str);
  ss[4] >> header >> calculation_time_;
  std::cout << header << calculation_time_ <<std::endl;
  std::getline(ifs, str);
  ss[5].str(str);
  ss[5] >> header >> best_cost_;
  std::cout << header << best_cost_ <<std::endl;
  std::getline(ifs, str);
  ss[6].str(str);
  ss[6] >> header >> min_var_;
  std::cout << header << min_var_ << std::endl;
  std::getline(ifs, str);
  ss[7].str(str);
  ss[7] >> header >> min_var_state_;
  std::cout << header << min_var_state_  <<std::endl;
  std::getline(ifs, str);
  ss[10].str(str);
  ss[10] >> header >> semi_stable_states_;
  std::cout << header << semi_stable_states_  <<std::endl;

  planning_path_.resize(0);
  for(int k = 0; k < state_list;  k++)
    {
      std::stringstream ss_tmp[3];
      conf_values state;
      //hard code
      state.state_values.resize(6);

      std::getline(ifs, str);
      ss_tmp[0].str(str);

      ss_tmp[0] >> header >> state.state_values[0] >> state.state_values[1] >> state.state_values[2] >> state.state_values[3] >>state.state_values[4] >> state.state_values[5] >> state.stable_mode >> state.dist_thre_value;

      planning_path_.push_back(state);

      //debug
      //ROS_INFO("state%d: joint1: %f",k , planning_path_[k].state_values[3]);
      // std::cout << "state: " << k << std::endl;
      // if(!transform_controller_->distThreCheckFromJointValues(state.state_values, 3))
      //   ROS_ERROR("(singular pose, can not resolve the lqi control problem");

    }
}

void MotionControl::controlFlagCallback(const std_msgs::UInt8ConstPtr& control_msg)
{
  if(control_msg->data == 0) 
    {
      ROS_WARN("stop motion control");
      control_flag_ = false;
      control_index_ = 0;
    }
  if(control_msg->data == 1) 
    {
      ROS_WARN("start motion control");
      control_flag_ = true;
      control_index_ = 0;
    }
}

void MotionControl::jointStateCallback(const sensor_msgs::JointStateConstPtr& joint_state_msg)
{

  std::vector<double> joint_state;
  joint_state.resize(0);
  for(int i = 0; i < (int)joint_state_msg->position.size(); i++)
    joint_state.push_back(joint_state_msg->position[i]);
  setJointStates(joint_state);

}

void MotionControl::moveCmd()
{
  //TODO
}

void MotionControl::jointCmd()
{
  ros::Rate loop_rate(joint_cmd_rate_);
  int joint_index = 0;

  while(ros::ok())
    {
      if(control_flag_) 
        {
          sensor_msgs::JointState ctrl_joint_msg;
          int temp = planning_path_[joint_index].state_values.size();
          ctrl_joint_msg.position.resize(0);
          for(int i = 3; i < temp; i ++)
            ctrl_joint_msg.position.push_back(planning_path_[joint_index].state_values[i]);
          joint_cmd_pub_.publish(ctrl_joint_msg);
          joint_index ++;
          //end, stop
          if(joint_index == planning_path_.size()) break;
        }
      loop_rate.sleep();
    }

}
