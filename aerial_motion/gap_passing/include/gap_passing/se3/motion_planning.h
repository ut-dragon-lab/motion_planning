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
#include <pluginlib/class_loader.h>
#include <tf/transform_listener.h>

/* basic header */
#include <gap_passing/se3/motion_control.h>
#include <dragon/transform_control.h>


/* moveit for FCL and visualization */
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>

/* ompl */
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/StateStorage.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/config.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/DiscreteMotionValidator.h>
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>

/* utils */
#include <iostream>
#include <valarray>
#include <limits>
#include <vector>
#include <sstream>

namespace se3
{
  class MotionPlanning
  {

  public:
    MotionPlanning(ros::NodeHandle nh, ros::NodeHandle nhp);
    ~MotionPlanning();

    static const int RRT_START_MODE = 0;
    static const int LBKPIECE1_MODE = 1;

    static const uint8_t HORIZONTAL_GAP = 0;
    static const uint8_t VERTICAL_GAP = 1;
  private:

    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;
    ros::Publisher planning_scene_diff_pub_;
    ros::Subscriber real_robot_state_sub_;
    ros::Publisher  flight_nav_;

    boost::shared_ptr<DragonTransformController> transform_controller_;

    //*** optimation objective
    ompl::base::PathLengthOptimizationObjective* path_length_opt_objective_;

    //*** planning control
    se3::MotionControl* motion_control_;

    robot_model_loader::RobotModelLoader *robot_model_loader_;
    robot_model::RobotModelPtr kinematic_model_;
    planning_scene::PlanningScene* planning_scene_;
    //moveit_msgs::CollisionObject collision_object_;
    moveit_msgs::PlanningScene planning_scene_msg_;
    collision_detection::AllowedCollisionMatrix acm_;

    double  motion_sequence_rate_;
    ros::Timer motion_sequence_timer_;
    float calculation_time_;

    //gap env
    int gap_type_;

    //original planning
    double x_low_bound_;
    double x_high_bound_;
    double y_low_bound_;
    double y_high_bound_;
    double z_low_bound_;
    double z_high_bound_;
    double pitch_joint_low_bound_;
    double pitch_joint_high_bound_;
    double yaw_joint_low_bound_;
    double yaw_joint_high_bound_;

    std::vector<double> start_state_; //SE2: 3 + n; SE3: 7 +n
    std::vector<double> goal_state_;  //SE2: 3 + n; SE3: 7 +n

    double state_validity_check_res_;
    int valid_segment_count_factor_;

    ompl::base::StateSpacePtr config_space_;
    ompl::base::SpaceInformationPtr space_information_;
    ompl::base::PathPtr path_;
    ompl::geometric::RRTstar* rrt_start_planner_;
    bool solved_;
    double solving_time_limit_;

    //visualization
    ompl::base::StateStorage* plan_states_;
    int planning_mode_;
    int locomotion_mode_;
    int ompl_mode_;

    double stability_cost_thre_;
    double length_cost_thre_;

    int joint_num_;

    double length_opt_weight_;
    double stability_opt_weight_;

    bool play_log_path_;//if true, use file, if false, get form realtime thing

    double best_cost_;

    void motionSequenceFunc(const ros::TimerEvent &e);

    bool isStateValid(const ompl::base::State *state);

    void rosParamInit();

    void gapEnvInit();
    void Planning();
    ompl::base::Cost onlyJointPathLimit();
    robot_state::RobotState setRobotState2Moveit(std::vector<double> robot_state);

    ompl::base::ValidStateSamplerPtr allocValidStateSampler(const ompl::base::SpaceInformation *si)
    {
      return ompl::base::ValidStateSamplerPtr(new ompl::base::ObstacleBasedValidStateSampler(si));
    }

  };

};
#endif