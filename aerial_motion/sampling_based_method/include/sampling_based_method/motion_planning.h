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

#ifndef MOTION_PLANNING_H_
#define MOTION_PLANNING_H_

/* ros */
#include <ros/ros.h>
#include <sampling_based_method/PlanningMode.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Empty.h>

/* basic header */
#include <aerial_motion_planning_msgs/multilink_state.h>
#include <aerial_robot_model/model/transformable_aerial_robot_model.h>

/* moveit for FCL and visualization */
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/robot_state/conversions.h>

/* ompl */
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/StateStorage.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/config.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>
#include <ompl/base/DiscreteMotionValidator.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

/* utils */
#include <iostream>
#include <valarray>
#include <limits>
#include <vector>
#include <sstream>
#include <fstream>

namespace sampling_base
{
  class MotionPlanning
    {

    public:
      MotionPlanning(ros::NodeHandle nh, ros::NodeHandle nhp, boost::shared_ptr<aerial_robot_model::transformable::RobotModel> transform_controller);
      ~MotionPlanning(){}

      static const int RRT_START_MODE = 0;

      const std::vector<MultilinkState>& getPathConst() const { return path_;}
      const MultilinkState& getStateConst(int index) const { return path_.at(index);}

      inline int getPathSize(){return  path_.size();}
      inline double getMotionCost(){return best_cost_;}
      inline float getPlanningTime(){return calculation_time_;}

      inline double getMinForce() {return  min_force_; }
      inline int  getMinForcetateIndex() {return min_force_state_index_;}
      inline double getMaxForce() {return  max_force_; }
      inline int  getMaxForcetateIndex() {return max_force_state_index_;}
      inline double getMinVar() {return  min_var_; }
      inline int  getMinVarStateIndex() {return min_var_state_index_;}
      inline void setStartState(const MultilinkState& state) { start_state_ = state;}
      inline void setGoalState(const MultilinkState& state) { goal_state_ = state;}

      void setScene(const moveit_msgs::CollisionObject& collision_object);
      bool plan();
      bool loadPath();
      void motionSequence();
      bool checkCollision(const MultilinkState& state);

    protected:
      /* ros */
      ros::NodeHandle nh_;
      ros::NodeHandle nhp_;
      ros::Publisher planning_scene_diff_pub_;
      ros::Publisher joint_state_pub_;       /* debug */

      /* flag */
      bool headless_;
      bool save_path_flag_;
      bool path_tf_debug_;
      int motion_type_;
      std::string robot_type_;

      /* moveit */
      boost::shared_ptr<planning_scene::PlanningScene> planning_scene_;
      moveit_msgs::PlanningScene planning_scene_msg_;
      collision_detection::AllowedCollisionMatrix acm_;

      /* robot */
      boost::shared_ptr<aerial_robot_model::transformable::RobotModel> robot_model_ptr_; //TODO change to RobotModel
      std::string baselink_name_;

      /* path */
      MultilinkState start_state_;
      MultilinkState goal_state_;
      std::vector<MultilinkState> path_;

      double file_state_offset_x_;
      double file_state_offset_y_;
      double file_state_offset_z_;
      std::string file_name_;

      /* OMPL */
      ompl::base::StateSpacePtr config_space_;
      ompl::base::SpaceInformationPtr space_information_;
      ompl::base::ProblemDefinitionPtr pdef_;
      std::array<double,3> position_lower_limits_, position_upper_limits_;
      std::vector<double> joint_lower_limits_, joint_upper_limits_;

      double calculation_time_;
      double state_validity_check_res_;
      int valid_segment_count_factor_;
      double solving_time_limit_;
      int ompl_mode_;
      double length_cost_thre_;
      double best_cost_;
      int planning_mode_;
      double min_var_;
      int min_var_state_index_;
      double max_force_, min_force_;
      int max_force_state_index_, min_force_state_index_;

      ompl::base::Cost onlyJointPathLimit();

      ompl::base::ValidStateSamplerPtr allocValidStateSampler(const ompl::base::SpaceInformation *si)
      {
        return ompl::base::ValidStateSamplerPtr(new ompl::base::ObstacleBasedValidStateSampler(si));
      }

      void addState(MultilinkState state) { path_.push_back(state); }

      virtual bool planInit();
      virtual bool isStateValid(const ompl::base::State *state);
      virtual void savePath();

      virtual void rosParamInit();

      virtual void addState(ompl::base::State *ompl_state);
    };
};
#endif
