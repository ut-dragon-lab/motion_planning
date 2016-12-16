#ifndef POSTURE_OPTIMIZATION_H
#define POSTURE_OPTIMIZATION_H


/*
1. most of the consturctor of pointer should be shared pointer!!
  e.g. transform_controller
 */

#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <hydrus_gap_passing/PlanningMode.h>
#include <hydrus_gap_passing/motion_control.h>

// MoveIt!
#include <hydrus_transform_control/transform_control.h>
#include <aerial_robot_base/States.h>

#include <iostream>
#include <valarray>
#include <limits>
#include <vector>
#include <sstream>

class PostureOptimization 
{
public:
  PostureOptimization(ros::NodeHandle nh, ros::NodeHandle nhp);
  ~PostureOptimization();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
};

#endif
