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

#include <gap_passing/se3/motion_control.h>

namespace se3
{
  MotionControl::MotionControl(ros::NodeHandle nh, ros::NodeHandle nhp, boost::shared_ptr<DragonTransformController>  transform_controller): nh_(nh), nhp_(nhp)
  {
    nhp_.param("log_flag", log_flag_, false);
    nhp_.param("play_log_path", play_log_path_, false);

    nhp_.param("joint_cmd_rate", joint_cmd_rate_, 50.0);
    nhp_.param("move_cmd_rate", move_cmd_rate_, 40.0);

    nhp_.param("backward_offset", backward_offset_, 100);
    nhp_.param("forward_offset", forward_offset_, 100);

    if(play_log_path_) log_flag_ = false;
    nhp_.param("file_name", file_name_, std::string("planning_log.txt"));

    //publisher
    joint_cmd_pub_ = nh_.advertise<sensor_msgs::JointState>("hydrus/joints_ctrl", 1);

    //init
    /* special for dragon: 2dof joint */
    joint_num_ = (transform_controller->getRotorNum() - 1) * 2;

    planning_path_.resize(0);
    planning_mode_ = 0;

    min_var_ = 1e6;
    min_var_state_ = 0;

    max_force_ = 0;
    max_force_state_ = 0;

    control_index_ = 0;

    if(play_log_path_) planFromFile();

    transform_controller_ = transform_controller;

    control_flag_ = false;

  }

  MotionControl::~MotionControl() {}

  void MotionControl::planStoring(const ompl::base::StateStorage* plan_states, int planning_mode, int locomotion_mode, const std::vector<double>& start_state, const std::vector<double>& goal_state,  double best_cost, double calculation_time)
  {
    best_cost_ = best_cost;
    calculation_time_ = calculation_time;
    planning_mode_ = planning_mode;
    locomotion_mode_ = locomotion_mode;

    conf_values state;
    //use start state to initialize the state
    state.state_values = start_state;

    int state_list = (int)plan_states->size();

    for(int i = 0; i < state_list; i++)
      {
        if(planning_mode == gap_passing::PlanningMode::ONLY_BASE_MODE)
          {
            if(locomotion_mode_ == gap_passing::PlanningMode::SE2)
              {
                state.state_values[0] = plan_states->getState(i)->as<ompl::base::SE2StateSpace::StateType>()->getX();
                state.state_values[1] = plan_states->getState(i)->as<ompl::base::SE2StateSpace::StateType>()->getY();
                tf::Quaternion q = tf::createQuaternionFromYaw(plan_states->getState(i)->as<ompl::base::SE2StateSpace::StateType>()->getYaw());
                state.state_values[3] = q.x();
                state.state_values[4] = q.y();
                state.state_values[5] = q.z();
                state.state_values[6] = q.w();
              }
            if(locomotion_mode_ == gap_passing::PlanningMode::SE3)
              {
                state.state_values[0] = plan_states->getState(i)->as<ompl::base::SE3StateSpace::StateType>()->getX();
                state.state_values[1] = plan_states->getState(i)->as<ompl::base::SE3StateSpace::StateType>()->getY();
                state.state_values[2] = plan_states->getState(i)->as<ompl::base::SE3StateSpace::StateType>()->getZ();
                state.state_values[3] = plan_states->getState(i)->as<ompl::base::SE3StateSpace::StateType>()->rotation().x;
                state.state_values[4] = plan_states->getState(i)->as<ompl::base::SE3StateSpace::StateType>()->rotation().y;
                state.state_values[5] = plan_states->getState(i)->as<ompl::base::SE3StateSpace::StateType>()->rotation().z;
                state.state_values[6] = plan_states->getState(i)->as<ompl::base::SE3StateSpace::StateType>()->rotation().w;
              }
          }
        else if(planning_mode == gap_passing::PlanningMode::JOINTS_AND_BASE_MODE)
          {
            const ompl::base::CompoundState* state_tmp = dynamic_cast<const ompl::base::CompoundState*>(plan_states->getState(i));

            /* SE2 */
            if(locomotion_mode_ == gap_passing::PlanningMode::SE2)
              {
                state.state_values[0] = state_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->getX();
                state.state_values[1] = state_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->getY();
                tf::Quaternion q = tf::createQuaternionFromYaw(state_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->getYaw());
                state.state_values[3] = q.x();
                state.state_values[4] = q.y();
                state.state_values[5] = q.z();
                state.state_values[6] = q.w();
              }

            /* SE3 */
            if(locomotion_mode_ == gap_passing::PlanningMode::SE3)
              {
                state.state_values[0] = state_tmp->as<ompl::base::SE3StateSpace::StateType>(0)->getX();
                state.state_values[1] = state_tmp->as<ompl::base::SE3StateSpace::StateType>(0)->getY();
                state.state_values[2] = state_tmp->as<ompl::base::SE3StateSpace::StateType>(0)->getZ();
                state.state_values[3] = state_tmp->as<ompl::base::SE3StateSpace::StateType>(0)->rotation().x;
                state.state_values[4] = state_tmp->as<ompl::base::SE3StateSpace::StateType>(0)->rotation().y;
                state.state_values[5] = state_tmp->as<ompl::base::SE3StateSpace::StateType>(0)->rotation().z;
                state.state_values[6] = state_tmp->as<ompl::base::SE3StateSpace::StateType>(0)->rotation().w;
              }

            for(int j = 0; j < joint_num_; j++)
              state.state_values[7 + j] = state_tmp->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[j];
          }

        /* dist thre */
        sensor_msgs::JointState joint_state;
        /* special for the dragon kinematics */
        for(int j = 0; j < joint_num_ / 2; j++)
          {
            std::stringstream ss;
            ss << j + 1;
            joint_state.name.push_back(std::string("joint") + ss.str() + std::string("_pitch"));
            joint_state.position.push_back(state.state_values[7 + 2 * j]);
            joint_state.name.push_back(std::string("joint") + ss.str() + std::string("_yaw"));
            joint_state.position.push_back(state.state_values[7 + 2 * j + 1]);

          }

        /* gimbal */
        for(int j = 0; j <= joint_num_ / 2; j++)
          {
            std::stringstream ss;
            ss << j + 1;
            joint_state.name.push_back(std::string("gimbal") + ss.str() + std::string("_roll"));
            joint_state.position.push_back(0);
            joint_state.name.push_back(std::string("gimbal") + ss.str() + std::string("_pitch"));
            joint_state.position.push_back(0);
          }

        /* need to set the cog coodrinate for dragon */
        transform_controller_->setCogDesireOrientation(KDL::Rotation::Quaternion(state.state_values[3],
                                                                                 state.state_values[4],
                                                                                 state.state_values[5],
                                                                                 state.state_values[6]));
        transform_controller_->kinematics(joint_state);
        state.dist_var = transform_controller_->distThreCheck();
        if(state.dist_var < min_var_)
          {
            min_var_ = state.dist_var;
            min_var_state_ = i;
          }

        transform_controller_->modelling();
        state.max_force = transform_controller_->getStableState().maxCoeff();
        if(state.max_force > max_force_)
          {
            max_force_ = state.max_force;
            max_force_state_ = i;
          }

        /* nominal gimbal angles */
        std::vector<double> gimbal_nominal_angles = transform_controller_->getGimbalNominalAngles();
        for(auto itr = gimbal_nominal_angles.begin(); itr != gimbal_nominal_angles.end(); ++itr)
          {
            size_t index = std::distance(gimbal_nominal_angles.begin(), itr);
            state.state_values[7 + joint_num_ + index] = (*itr); // roll -> pitch
          }

        /* log out */
        tf::Matrix3x3 att(tf::Quaternion(state.state_values[3],
                                         state.state_values[4],
                                         state.state_values[5],
                                         state.state_values[6]));
        double r, p, y; att.getRPY(r, p, y);
        ROS_INFO("state: %d, dist_var: %f, max_force: %f, base pose: [%f, %f, %f] att: [%f, %f, %f]", i, state.dist_var, state.max_force, state.state_values[0], state.state_values[1], state.state_values[2], r, p, y);

        ROS_INFO(" j1: [%f, %f], j2:[%f, %f], j3:[%f, %f]",
                 state.state_values[7],
                 state.state_values[8],
                 state.state_values[9],
                 state.state_values[10],
                 state.state_values[11],
                 state.state_values[12]);

        /* interation */
        planning_path_.push_back(state);
        state.state_values.resize(start_state.size());
      }

    //file log
    if(log_flag_)
      {
        std::ofstream ofs;
        ofs.open( "dragon_planning_log.txt" );
        ofs << "start_state:";
        for(int j = 0; j < 7 + 2 * joint_num_ + 2; j++) ofs   << " " << start_state[j];
        ofs << std::endl;

        ofs << "goal_state:";
        for(int j = 0; j < 7 + 2 * joint_num_ + 2; j++) ofs   << " " << goal_state[j];
        ofs << std::endl;

        ofs << "states: " << state_list  << std::endl;
        ofs << "planning_mode: " << planning_mode << std::endl;
        ofs << "locomotion_mode: " << locomotion_mode << std::endl;
        ofs << "planning_time: " << calculation_time << std::endl;
        ofs << "motion_cost: " << best_cost << std::endl;
        ofs << "minimum_var: " << min_var_ << std::endl;
        ofs << "minimum_var_state_entry: " << min_var_state_  << std::endl;
        ofs << "maximum_force : " << max_force_ << std::endl;
        ofs << "maximum_force_state_entry: " << max_force_state_  << std::endl;

        for(int k = 0; k < state_list;  k++)
          {
            ofs << "state" << k << ":";
            for(int j = 0; j < 7 + 2 * joint_num_ + 2; j++) ofs << " "  << planning_path_[k].state_values[j];
            ofs<< " " <<planning_path_[k].dist_var << " " << planning_path_[k].max_force <<  std::endl;
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

    std::vector<double> start_state(7 + 2 * joint_num_ + 2, 0);
    std::vector<double> goal_state(7 + 2 * joint_num_ + 2, 0);
    int state_list;
    std::stringstream ss[11];
    std::string str;
    std::string header;
    //1 start and goal state
    std::getline(ifs, str);
    ss[0].str(str);
    ss[0] >> header;
    for(int i = 0; i < 7 + 2 * joint_num_ + 2; i++) ss[0] >> start_state[i];
    std::cout << header << std::endl;
    std::getline(ifs, str);
    ss[1].str(str);
    ss[1] >> header;
    for(int i = 0; i < 7 + 2 * joint_num_ + 2; i++) ss[1] >> goal_state[i];
    std::cout << header << std::endl;
    ROS_WARN("from [%f, %f, %f] [%f, %f, %f, %f] to [%f, %f, %f] [%f, %f, %f, %f]",
             start_state[0], start_state[1], start_state[2],
             start_state[3], start_state[4], start_state[5], start_state[6],
             goal_state[0], goal_state[1], goal_state[2],
             goal_state[3], goal_state[4], goal_state[5], goal_state[6]);

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
    ss[3].str(str);
    ss[3] >> header >> locomotion_mode_;
    std::cout << header << locomotion_mode_ <<std::endl;
    std::getline(ifs, str);
    ss[5].str(str);
    ss[5] >> header >> calculation_time_;
    std::cout << header << calculation_time_ <<std::endl;
    std::getline(ifs, str);
    ss[6].str(str);
    ss[6] >> header >> best_cost_;
    std::cout << header << best_cost_ <<std::endl;
    std::getline(ifs, str);
    ss[7].str(str);
    ss[7] >> header >> min_var_;
    std::cout << header << min_var_ << std::endl;
    std::getline(ifs, str);
    ss[8].str(str);
    ss[8] >> header >> min_var_state_;
    std::cout << header << min_var_state_  <<std::endl;
    std::getline(ifs, str);
    ss[9].str(str);
    ss[9] >> header >> max_force_;
    std::cout << header << max_force_ << std::endl;
    std::getline(ifs, str);
    ss[10].str(str);
    ss[10] >> header >> max_force_state_;
    std::cout << header << max_force_state_  <<std::endl;

    planning_path_.resize(0);
    for(int k = 0; k < state_list;  k++)
      {
        std::stringstream ss_tmp[3];
        conf_values state;

        state.state_values.resize(7 + 2 * joint_num_ + 2, 0);

        std::getline(ifs, str);
        ss_tmp[0].str(str);

        /* se3 base */
        ss_tmp[0] >> header;
        /* joint */
        for(int j = 0; j < 7 + 2 * joint_num_ + 2; j++) ss_tmp[0] >> state.state_values[j];
        /* dist */
        ss_tmp[0] >> state.dist_var >> state.max_force;
        planning_path_.push_back(state);
      }
  }

}
