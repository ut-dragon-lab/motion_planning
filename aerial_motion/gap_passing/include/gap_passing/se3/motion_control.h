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
#include <dragon/transform_control.h>

/* ompl */
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/StateStorage.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
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
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>

namespace se3
{
  struct configuration_space{
    std::vector<double> state_values; //se3(x,y,z, q_x, q_y, q_z, q_w) joints
    float dist_var;
    float max_force;
    configuration_space()
    {
      state_values.resize(0);
      dist_var = 0;
      max_force = 0;
    }

  };
  typedef struct configuration_space conf_values;


  class MotionControl
  {
  public:
    MotionControl(ros::NodeHandle nh, ros::NodeHandle nhp, boost::shared_ptr<DragonTransformController>  transform_controller);
    ~MotionControl();
    void planStoring(const ompl::base::StateStorage* plan_states, int planning_mode, int locomotion_mode, const std::vector<double>& start_state, const std::vector<double>& goal_state, double best_cost, double calculation_time);

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
    inline float getMinVar() {return  min_var_; }
    inline int  getMinVarStateIndex() {return min_var_state_;}
    inline float getMaxForce() {return  max_force_; }
    inline int  getMaxForceStateIndex() {return max_force_state_;}

    ros::Publisher joint_cmd_pub_; //joints control

  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;

    boost::shared_ptr<DragonTransformController> transform_controller_;
    std::vector<conf_values> planning_path_;

    bool control_flag_;
    double joint_cmd_rate_;
    double move_cmd_rate_;

    int control_index_;

    bool play_log_path_;//if true, use file, if false, get form realtime thing
    bool log_flag_;//log info to file?

    std::string file_name_;

    int robot_type_;
    int joint_num_;

    //some additional
    double best_cost_;
    int planning_mode_;
    int locomotion_mode_;
    double calculation_time_;
    float min_var_;
    int min_var_state_;
    float max_force_;
    int max_force_state_;
    float max_depression_;
    int max_depression_state_;

    //joint cmd
    int backward_offset_;
    int forward_offset_;

    void setJointStates(std::vector<double> joint_states);
  };
};
#endif
