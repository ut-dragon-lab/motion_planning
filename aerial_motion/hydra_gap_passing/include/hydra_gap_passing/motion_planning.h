/* Author: Sachin Chitta */
#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/StateStorage.h>
#include <ompl/base/spaces/SE2StateSpace.h>
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
#include "ompl/base/samplers/ObstacleBasedValidStateSampler.h"

#include <hydra_transform_control/transform_control.h>
#include <aerial_robot_base/States.h>

//#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
//#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <iostream>
#include <valarray>
#include <limits>
#include <vector>


struct configuration_space{
  float x;
  float y;
  float theta;
  float joint1;
  float joint2;
  float joint3;
};
typedef struct configuration_space conf_values;

class StabilityObjective : public ompl::base::StateCostIntegralObjective
{
 public:
 StabilityObjective(const ompl::base::SpaceInformationPtr& si, TransformController*  transform_controller,   int planning_mode): ompl::base::StateCostIntegralObjective(si, true)
    {
      transform_controller_ =  transform_controller;
      planning_mode_  = planning_mode
    }
  ~StabilityObjective();

  ompl::base::Cost stateCost(const ompl::base::State* s) const;

  bool distThreCheck(std::vector<double> joint_values);


 private:
  TransformController*  transform_controller_;

  int planning_mode_;

};


class MotionPlanning
{

public:
  MotionPlanning(ros::NodeHandle nh, ros::NodeHandle nhp);
  ~MotionPlanning();


  inline tf::Vector3 world2GapCoord(tf::Vector3 world_info)
  {
    return tf::Vector3(-world_info.getY(), world_info.getX(), world_info.getZ());
  }
  inline tf::Vector3 gap2WorldCoord(tf::Vector3 gap_coord_info)
  {
    return tf::Vector3(gap_coord_info.getY(),-gap_coord_info.getX(), gap_coord_info.getZ());

  }


  static const int ONLY_JOINTS_MODE = 0;
  static const int ONLY_BASE_MODE = 1;
  static const int JOINTS_AND_BASE_MODE = 2;
  static const int ORIGINAL_MODE = 3;

  static const int RRT_START_MODE = 0;
  static const int LBKPIECE1_MODE = 1;

  static const int NO_OVERLAP = 0; 
  static const int Y_AXIS_OVERLAP = 1; //gap coord
  static const int X_AXIS_OVERLAP = 2; //gap coord



private:

  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  ros::Publisher planning_scene_diff_pub_;
  ros::Subscriber real_robot_state_sub_;
  ros::Publisher  flight_nav_;

  //+++ hydra
  TransformController*  transform_controller_;

  //*** optimation objective
  ompl::base::PathLengthOptimizationObjective* path_length_opt_objective_;
  StabilityObjective* stability_objective_;

  robot_model_loader::RobotModelLoader *robot_model_loader_;
  robot_model::RobotModelPtr kinematic_model_;
  planning_scene::PlanningScene* planning_scene_;
  //planning_scene::PlanningScenePtr planning_scene_;
  moveit_msgs::CollisionObject collision_object_;
  moveit_msgs::PlanningScene planning_scene_msg_;
  collision_detection::AllowedCollisionMatrix acm_;

  double  motion_sequence_rate_;
  ros::Timer motion_sequence_timer_;
  float calculation_time_;

  double tolerance_;

  //real robot 
  bool real_robot_move_base_;
  bool get_init_state_;
  double mocap_center_to_joint_x_, mocap_center_to_joint_y_;

  //gap env
  //int gap_overlap_type_;
  double gap_left_x_, gap_left_y_;
  double gap_x_offset_;
  double gap_y_offset_;
  double gap_left_width_;
  double gap_right_width_;
  tf::Vector3 left_half_corner;
  tf::Vector3 right_half_corner;

  //original planning
  std::vector<conf_values> original_path_;
  std::vector<conf_values> real_robot_path_;
  std::vector<double> start_state_;
  std::vector<double> goal_state_;

  double state_validity_check_res_;
  int valid_segment_count_factor_;

  ompl::base::StateSpacePtr hydra_space_;
  ompl::base::SpaceInformationPtr space_information_;
  ompl::base::PathPtr path_;
  bool solved_;
  double solving_time_limit_;

  //visualization
  int state_index_;
  ompl::base::StateStorage* plan_states_;
  int planning_mode_;
  int ompl_mode_;
  double coefficient_rate_;

  double semi_stable_cost_;
  double full_stable_cost_;

  double length_opt_weight_;
  double stability_opt_weight_;

  void motionSequenceFunc(const ros::TimerEvent &e);

  bool isStateValid(const ompl::base::State *state);

  void rosParamInit();

  // ompl::base::OptimizationObjectivePtr getThresholdPathLengthObj(const ompl::base::SpaceInformationPtr& si)
  // {
  //   ompl::base::OptimizationObjectivePtr obj(new ompl::base::PathLengthOptimizationObjective(si));
  //   obj->setCostThreshold(ompl::base::Cost(1.51));
  //   return obj;
  // }

  ompl::base::ValidStateSamplerPtr allocValidStateSampler(const ompl::base::SpaceInformation *si)
    {
      return ompl::base::ValidStateSamplerPtr(new ompl::base::ObstacleBasedValidStateSampler(si));
    }

  float distance(float x1, float x2, float y1, float y2)
  {
    return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
  }


};
