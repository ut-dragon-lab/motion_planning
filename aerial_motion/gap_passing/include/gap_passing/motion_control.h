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

#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

/* ros */
#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <sensor_msgs/JointState.h>
#include <gap_passing/PlanningMode.h>
#include <aerial_robot_base/FlightNav.h>
#include <hydrus/transform_control.h>

/* ompl */
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/StateStorage.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/config.h>

/* utils */
#include <iostream>
#include <valarray>
#include <limits>
#include <vector>
#include <sstream>
#include <fstream>
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>

struct configuration_space{
  std::vector<double> state_values; //x,y,theta,joints
  int stable_mode; //0: full-stable, 1: semi-stable
  int dist_thre_value; //0: bad, 1; good
  int control_mode;
  configuration_space()
  {
    state_values.resize(0);
    stable_mode = 0;
    dist_thre_value = 0;
    control_mode = gap_passing::PlanningMode::POSITION_MODE;
  }

};
typedef struct configuration_space conf_values;


class MotionControl
{
  /*func
    1) log
      |- the path info
      |-- joints entry
      |-- best cost/ calculation time
      |-- full/semi stable
      |- calculate the gains and rotates angles for each state
      |  path_states * (link_num * (size_of_float + 12state_gains * size_of_float))  : 1000 * (4* (4 + 12 * 4) ) = 208000bit
    2) control
     |- get control info
     |-- get the path and gains from file
     |-- get the path and gains realtimely

     |- get real-robot-state info
     |-- joints 
     |-- position 

     |- transform control
     |-- use the real joints values to search the best entry in the path/gains table, send to robot system(transform)
     - moving control
     -- use the real position to calculate the best velocity to robot system

   */
 public:
  MotionControl(ros::NodeHandle nh, ros::NodeHandle nhp, boost::shared_ptr<TransformController>  transform_controller);
  ~MotionControl();
  void planStoring(const ompl::base::StateStorage* plan_states, int planning_mode, const std::vector<double>& start_state, const std::vector<double>& goal_state, double best_cost, double calculation_time);

  void planFromFile();

  void getPlanningPath(std::vector<conf_values> & planning_path)
  {
    for(int i = 0; i < planning_path_.size(); i++)
      planning_path.push_back(planning_path_[i]);
  }

  conf_values getState(int index){      return planning_path_[index];  }
  inline int getPathSize(){return  planning_path_.size();}
  inline double getMotionCost(){return best_cost_;}
  inline float getPlanningTime(){return calculation_time_;}
  inline int getSemiStableStates(){return semi_stable_states_;}
  inline float getMinVar() {return  min_var_; }
  inline int  getMinVarStateIndex() {return min_var_state_;}

  ros::Publisher joint_cmd_pub_; //joints control

  static const int HYDRUS = 0;
  static const int DRAGON = 1;

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;

  //control component
  ros::Publisher move_cmd_pub_; // move base control

  ros::Subscriber control_flag_sub_;
  ros::Subscriber joint_values_sub_;

  //thread components
  boost::thread joint_cmd_thread_;
  boost::thread move_cmd_thread_;

  boost::shared_ptr<TransformController> transform_controller_;
  std::vector<conf_values> planning_path_;

  bool control_flag_;
  double joint_cmd_rate_;
  double move_cmd_rate_;

  int control_index_;

  bool play_log_path_;//if true, use file, if false, get form realtime thing
  bool log_flag_;//log info to file?

  std::string file_name_;

  int robot_type_;
  int link_num_;

  //some additional
  double best_cost_;
  int planning_mode_;
  double calculation_time_;
  int semi_stable_states_;
  float min_var_;
  int min_var_state_;

  //joint cmd
  int backward_offset_;
  int forward_offset_;

  void jointStateCallback(const sensor_msgs::JointStateConstPtr& joint_state);
  void controlFlagCallback(const std_msgs::UInt8ConstPtr& control_msg);

  void setJointStates(std::vector<double> joint_states);

  void jointCmd();
  void moveCmd();
};

#endif
