#ifndef POSTURE_OPTIMIZATION_H
#define POSTURE_OPTIMIZATION_H


/*
1. most of the consturctor of pointer should be shared pointer!!
  e.g. transform_controller
 */

#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>


// MoveIt!
#include <hydrus_transform_control/transform_control.h>
#include <aerial_robot_base/States.h>
#include <aerial_robot_base/FlightNav.h>

#include <iostream>
#include <valarray>
#include <limits>
#include <vector>
#include <algorithm>
#include <string>
#include <boost/lexical_cast.hpp>
#include <sensor_msgs/JointState.h>
#include <thread>

using namespace Eigen;

class PostureOptimization 
{
public:
  PostureOptimization(ros::NodeHandle nh, ros::NodeHandle nhp);
  ~PostureOptimization();
  void process();
  void check();
private:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;

  ros::Publisher joint_pub_;
  ros::ServiceClient add_extra_module_client_;

  boost::shared_ptr<TransformController> transform_controller_;
  double alpha_;
  double d_theta_;
  double grad_f_thresh_;
  double v_thresh_;
  double time_thresh_;
  int thread_num_;
  std::vector<double> initial_theta_;
  int extra_module_link_num_1_;
  int extra_module_link_num_2_;
  double extra_module_mass_;
  double extra_module_offset_;
  int link_num_;
  double ring_radius_;
  double linkend_radius_;
  std::vector<double> theta_;
  std::vector<double> grad_f_;
  double last_variance_;
  std::vector<double> valid_theta_;
  VectorXd getX(TransformController& transform_controller, std::vector<double> theta);
  bool collisionCheck(TransformController& transform_controller, std::vector<double> theta);
  void rosParamInit();
  void steepestDescent(std::vector<double> initial_theta, std::vector<double>& optimized_theta, double& optimized_variance);
};

#endif