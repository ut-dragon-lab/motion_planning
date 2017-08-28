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

#ifndef FORM_OPTIMIZATION_H
#define FORM_OPTIMIZATION_H

#include <ros/ros.h>

/* ros message */
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>

/* stl */
#include <iostream>
#include <vector>
#include <algorithm>
#include <string>
#include <thread>

#include <hydrus/transform_control.h>

using namespace Eigen;

class FormOptimization 
{
public:
  FormOptimization(ros::NodeHandle nh, ros::NodeHandle nhp);
  ~FormOptimization();
  void process();
  
private:
  /* ros */
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;

  /* ros publisher */
  ros::Publisher joint_pub_;
  ros::Publisher visualization_marker_pub_;
  /* tf listener */
  tf::TransformListener listener;
  
  /* ros param */
  std::string joint_states_topic_name_;
  double alpha_;
  double d_joint_angle_;
  double v_thresh_;
  double time_thresh_;
  int thread_num_;
  int extra_module_num_;
  bool use_initial_joint_angle_;
  int joint_num_;
  double ring_radius_;
  double linkend_radius_;
  bool verbose_;
  bool visualization_;
  std::vector<double> initial_joint_angle_;
  std::vector<int> extra_module_link_num_;
  std::vector<double> extra_module_mass_;
  std::vector<double> extra_module_offset_;

  Eigen::VectorXd g_;
  std::vector<std::string> joint_names_;

  VectorXd getU(TransformController& transform_controller, std::vector<double> joint_angle, bool& is_stable);
  VectorXd getU(TransformController& transform_controller, std::vector<double> joint_angle);
  bool collisionCheck(TransformController& transform_controller, std::vector<double> joint_angle);
  
  void steepestDescent(std::vector<double> initial_joint_angle, std::vector<double>& optimized_joint_angle, double& optimized_variance);
  
  void visualization(std::vector<double> joint_angle);
};

#endif
