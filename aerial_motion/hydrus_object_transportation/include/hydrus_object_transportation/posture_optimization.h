#ifndef POSTURE_OPTIMIZATION_H
#define POSTURE_OPTIMIZATION_H

/*
1. most of the consturctor of pointer should be shared pointer!!
  e.g. transform_controller
 */

#include <ros/ros.h>

#include <hydrus_transform_control/transform_control.h>
#include <aerial_robot_base/States.h>

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
  int extra_module_num_;
  std::vector<int> extra_module_link_num_;
  std::vector<double> extra_module_mass_;
  std::vector<double> extra_module_offset_;
  bool use_initial_theta_;
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
  void addExtraModule(bool reset, double extra_module_link_num, double extra_module_mass, double extra_module_offset);
};

#endif
