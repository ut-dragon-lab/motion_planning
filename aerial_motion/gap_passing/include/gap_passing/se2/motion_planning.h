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

#ifndef SE2_MOTION_PLANNING_H_
#define SE2_MOTION_PLANNING_H_

/* ros */
#include <ros/ros.h>
#include <gap_passing/Keyposes.h>
#include <gap_passing/Endposes.h>
#include <gap_passing/PlanningMode.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Empty.h>

/* basic header */
#include <hydrus/transform_control.h>

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

class State
{
public:
  State(): cog_state(7, 0), root_state(7, 0), joint_states(0), joint_names(0) {}
  ~State(){}

  void setRootRPY(double r, double p, double y)
  {
    tf::Quaternion q;
    q.setRPY(r,p,y);
    root_state.at(3) = q.x();
    root_state.at(4) = q.y();
    root_state.at(5) = q.z();
    root_state.at(6) = q.w();
  }

  void setCogRPY(double r, double p, double y)
  {
    tf::Quaternion q;
    q.setRPY(r,p,y);
    cog_state.at(3) = q.x();
    cog_state.at(4) = q.y();
    cog_state.at(5) = q.z();
    cog_state.at(6) = q.w();
  }

  void getRootRPY(double& r, double& p, double& y)
  {
    tf::Matrix3x3 att(tf::Quaternion(root_state.at(3),
                                     root_state.at(4),
                                     root_state.at(5),
                                     root_state.at(6)));
    att.getRPY(r, p, y);
  }

  void getCogRPY(double& r, double& p, double& y)
  {
    tf::Matrix3x3 att(tf::Quaternion(cog_state.at(3),
                                     cog_state.at(4),
                                     cog_state.at(5),
                                     cog_state.at(6)));
    att.getRPY(r, p, y);
  }

  std::vector<double> cog_state; // x, y, z, q_x, q_y, q_z, q_w
  std::vector<double> root_state; // x, y, z, q_x, q_y, q_z, q_w
  std::vector<double> joint_states;
  std::vector<std::string> joint_names;
};

namespace se2
{
  class MotionPlanning
  {

  public:
    MotionPlanning(ros::NodeHandle nh, ros::NodeHandle nhp);
    ~MotionPlanning(){}

    void baseInit();

    static const int RRT_START_MODE = 0;

    const std::vector<State>& getPath() { return path_;}
    const State& getState(int index){ return path_[index];}

    inline int getPathSize(){return  path_.size();}
    inline double getMotionCost(){return best_cost_;}
    inline float getPlanningTime(){return calculation_time_;}
    inline float getMinVar() {return  min_var_; }
    inline int  getMinVarStateIndex() {return min_var_state_;}
    inline int getRootMotionDof() {return root_motion_dof_; }

    virtual State cog2root(const std::vector<double> &keypose); // transfer cog link keypose to rootlink
    virtual State root2cog(const std::vector<double> &keypose); // transfer root link keypose to cog link
    boost::shared_ptr<TransformController> getTransformController() { return transform_controller_;}

  protected:
    /* ros */
    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;
    ros::Publisher planning_scene_diff_pub_;
    ros::ServiceServer keyposes_server_;

    ros::Timer motion_sequence_timer_;
    double  motion_sequence_rate_;

    /* flag */
    bool save_path_flag_;
    bool load_path_flag_;
    bool play_path_flag_;
    bool endposes_from_rosserice_flag_;

    /* moveit */
    //boost::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
    boost::shared_ptr<planning_scene::PlanningScene> planning_scene_;
    moveit_msgs::PlanningScene planning_scene_msg_;
    collision_detection::AllowedCollisionMatrix acm_;

    /* gap env */
    double gap_left_x_, gap_left_y_;
    double gap_x_offset_;
    double gap_y_offset_;
    double gap_left_width_;
    double gap_right_width_;
    tf::Vector3 left_half_corner;
    tf::Vector3 right_half_corner;

    /* robot */
    std::string base_link_;
    tf::Quaternion baselink_desired_att_;
    boost::shared_ptr<TransformController> transform_controller_;
    int joint_num_;
    //int ex_joint_num_; //the extended joint number for moveit robot state
    int root_motion_dof_;

    /* path */
    State start_state_;
    State goal_state_;
    std::vector<State> path_;
    double file_state_offset_x_;
    double file_state_offset_y_;
    double file_state_offset_z_;
    std::string file_name_;

    /* OMPL */
    ompl::base::StateSpacePtr config_space_;
    ompl::base::SpaceInformationPtr space_information_;
    ompl::base::ProblemDefinitionPtr pdef_;
    double x_low_bound_;
    double x_high_bound_;
    double y_low_bound_;
    double y_high_bound_;
    double joint_low_bound_;
    double joint_high_bound_;

    bool solved_;
    double state_validity_check_res_;
    int valid_segment_count_factor_;
    double solving_time_limit_;
    int ompl_mode_;
    double length_cost_thre_;
    double best_cost_;
    int planning_mode_;
    double calculation_time_;
    float min_var_;
    int min_var_state_;

    void plan();
    void sceneInit();

    virtual void motionSequenceFunc(const ros::TimerEvent &e);

    ompl::base::Cost onlyJointPathLimit();

    ompl::base::ValidStateSamplerPtr allocValidStateSampler(const ompl::base::SpaceInformation *si)
    {
      return ompl::base::ValidStateSamplerPtr(new ompl::base::ObstacleBasedValidStateSampler(si));
    }

    bool getKeyposes(gap_passing::Keyposes::Request &req, gap_passing::Keyposes::Response &res);
    void addState(State state) { path_.push_back(state); }

    virtual void planInit();
    virtual bool isStateValid(const ompl::base::State *state);
    virtual void savePath();
    virtual void loadPath();

    virtual void rosParamInit();
    virtual void robotInit();
    virtual void gapEnvInit();
    virtual void addState(ompl::base::State *ompl_state);
    virtual robot_state::RobotState setRobotState2Moveit(State state, boost::shared_ptr<planning_scene::PlanningScene> planning_scene);

    /* replay mode */
    bool realtime_path_flag_;
    ros::Subscriber continous_path_sub_;
    boost::shared_ptr<planning_scene::PlanningScene> real_state_scene_;
    moveit_msgs::PlanningScene real_state_scene_msg_;
    ros::Publisher real_state_scene_diff_pub_;
    ros::Subscriber robot_cog_odom_sub_;
    ros::Subscriber robot_joint_states_sub_;
    ros::Subscriber baselink_desired_attitude_sub_;
    nav_msgs::Odometry robot_cog_odom_;

    void robotOdomCallback(const nav_msgs::OdometryConstPtr& msg);
    void robotJointStatesCallback(const sensor_msgs::JointStateConstPtr& joint_msg);
    void continousPathCallback(const std_msgs::Float64MultiArrayConstPtr& msg);
    void desireCoordinateCallback(const aerial_robot_base::DesireCoordConstPtr & msg);
  };

};
#endif
