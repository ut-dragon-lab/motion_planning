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

#include <hydrus_object_transportation/form_optimization.h>

FormOptimization::FormOptimization(ros::NodeHandle nh, ros::NodeHandle nhp) :nh_(nh), nhp_(nhp)
{
  /* ros params */
  nhp_.param("joint_states_topic_name", joint_states_topic_name_, std::string("/hydrusx/joint_states"));
  nhp_.param("alpha", alpha_, 0.1);
  nhp_.param("d_joint_angle", d_joint_angle_, 0.01);
  nhp_.param("v_thresh", v_thresh_, 0.001);
  nhp_.param("time_thresh", time_thresh_, 10.0);
  nhp_.param("thread_num", thread_num_, 1);
  nhp_.param("extra_module_num", extra_module_num_, 2);
  nhp_.param("use_initial_joint_angle_", use_initial_joint_angle_, false);
  nhp_.param("ring_radius", ring_radius_, 0.2);
  nhp_.param("linkend_radius", linkend_radius_, 0.02);
  nhp_.param("verbose", verbose_, false);
  nhp_.param("visualization", visualization_, true);
  nh_.param("/hydrusx/joint_num", joint_num_, 3);
 
  if (use_initial_joint_angle_) {
    thread_num_ = 1;  
    initial_joint_angle_.resize(joint_num_);
    for (int i = 0; i < joint_num_; i++) {   
      nhp_.param(std::string("initial_joint_angle") + boost::to_string(i), initial_joint_angle_[i], 0.0);
    }
  }
  
  extra_module_link_num_.resize(extra_module_num_);
  extra_module_mass_.resize(extra_module_num_);
  extra_module_offset_.resize(extra_module_num_);
  for (int i = 0; i < extra_module_num_; i++) {
    nhp_.param(std::string("extra_module_link_num") + boost::to_string(i), extra_module_link_num_.at(i), 0);
    nhp_.param(std::string("extra_module_mass") + boost::to_string(i), extra_module_mass_.at(i), 0.5);
    nhp_.param(std::string("extra_module_offset") + boost::to_string(i), extra_module_offset_.at(i), 0.3);
  }
  
  /* ros publisher */
  joint_pub_ = nh_.advertise<sensor_msgs::JointState>(joint_states_topic_name_, 1);
  visualization_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/visualization_marker", 1);
  
  g_ = Eigen::VectorXd(4);
  g_ << 0.0, 0.0, 9.8, 0.0;

  joint_names_.resize(joint_num_);
  for (int i = 0; i < joint_num_; i++) {
    joint_names_.at(i) = std::string("joint") + boost::to_string(i + 1);
  }
}


FormOptimization::~FormOptimization()
{
}

Eigen::VectorXd FormOptimization::getU(TransformController& transform_controller, std::vector<double> joint_angle, bool& is_stable) 
{
  sensor_msgs::JointState joint_state;
  joint_state.position = joint_angle;
  joint_state.name = joint_names_;
  transform_controller.kinematics(joint_state);
  is_stable = transform_controller.modelling(false);
  Eigen::MatrixXd P = transform_controller.getP();
  Eigen::FullPivLU<Eigen::MatrixXd> solver((P * P.transpose())); 
  Eigen::VectorXd lamda;
  lamda = solver.solve(g_);
  return P.transpose() * lamda;
}

Eigen::VectorXd FormOptimization::getU(TransformController& transform_controller, std::vector<double> joint_angle) 
{
  bool dummy;
  return getU(transform_controller, joint_angle, dummy);
}

bool FormOptimization::collisionCheck(TransformController& transform_controller, std::vector<double> joint_angle)
{
  std::vector<Eigen::Vector3d> rotors_origin_from_cog;
  std::vector<Eigen::Vector3d> linkends_origin_from_cog;
  transform_controller.getRotorsFromCog(rotors_origin_from_cog);
  //transform_controller.getLinkendsOriginFromCog(linkends_origin_from_cog);
  for (int i = 0; i < rotors_origin_from_cog.size() - 1; i++) {
    for (int j = i + 1; j < rotors_origin_from_cog.size(); j++) {
      Eigen::Vector3d diff_vec = rotors_origin_from_cog.at(i) - rotors_origin_from_cog.at(j);
      if (diff_vec.norm() < ring_radius_ * 2) return false;
    }
  }
  /*
  for (int i = 0; i < linkends_origin_from_cog.size() - 1; i++) {
    for (int j = i + 1; j < linkends_origin_from_cog.size(); j++) {
      Eigen::Vector3d diff_vec = linkends_origin_from_cog[i] - linkends_origin_from_cog[j];
      if (diff_vec.norm() < linkend_radius_ * 2) return false;
    }
  }
  
  for (int i = 0; i < links_origin_from_cog.size(); i++) {
    for (int j = 0; j < linkends_origin_from_cog.size(); j++) {
      Eigen::Vector3d diff_vec = links_origin_from_cog[i] - linkends_origin_from_cog[j];
      if (diff_vec.norm() < linkend_radius_ + ring_radius_) return false;
    }
  }
  */
  return true;
}


void FormOptimization::steepestDescent(std::vector<double> initial_joint_angle, std::vector<double>& optimized_joint_angle, double& optimized_variance)
{
  std::vector<double> joint_angle = initial_joint_angle;
  std::vector<double> valid_joint_angle = initial_joint_angle;
  std::vector<double> grad_f(joint_num_);
  double last_variance = 1000000.0;
  TransformController transform_controller(nh_, nhp_, false);
  for (int i = 0; i < extra_module_num_; i++) {
    transform_controller.addExtraModule(extra_module_link_num_.at(i), extra_module_mass_.at(i), extra_module_offset_.at(i));
  }

  ros::Time start_time = ros::Time::now();
  {
    VectorXd u = getU(transform_controller, joint_angle); //propeller force vector
    double variance = u.squaredNorm() / u.size() - u.sum() * u.sum() / u.size() / u.size();
    if (verbose_)
      std::cout << "initial variance:" <<  variance << std::endl;
  }

  while (true) {
    double current_f = getU(transform_controller, joint_angle).norm();
    
    //calc gradient
    for (unsigned int i = 0; i < grad_f.size(); i++) {
      std::vector<double> tmp_joint_angle = joint_angle;
      tmp_joint_angle[i] += d_joint_angle_;
      grad_f[i] = (getU(transform_controller, tmp_joint_angle).norm() - current_f) / d_joint_angle_;
    }
    
    //update joint_angle
    for (unsigned i = 0; i < joint_angle.size(); i++) {
      joint_angle[i] = std::max(-M_PI/2, std::min(M_PI / 2, joint_angle[i] - alpha_ * grad_f[i]));
    }

    bool is_stable;
    VectorXd u = getU(transform_controller, joint_angle, is_stable); //execute transform_controller.kinematics(theta) and transform_controller.modelling(false) in this function
     
    //validity check
    bool is_valid = true;
    if (!transform_controller.distThreCheck()) {
      if (verbose_) ROS_WARN("dist is too small");
      is_valid = false;
    }
 
    if (!is_stable) {
      if (verbose_) ROS_WARN("not stable");
      is_valid = false;
    }
   
    if (!collisionCheck(transform_controller, joint_angle)) {
      if (verbose_) ROS_WARN("collision");
      is_valid = false;
    }

    for (int i = 0; i < u.size(); i++) {
      if (u(i) < 0) {
        is_valid = false;
        if (verbose_) ROS_WARN("invalid U");
        break;
      }
    }

    double variance = u.squaredNorm() / u.size() - u.sum() * u.sum() / u.size() / u.size();
    if (is_valid && (last_variance >= variance)) {
      last_variance = variance;
      valid_joint_angle = joint_angle;
    }

    //break condition
    if (last_variance < v_thresh_ || (ros::Time::now().toSec() - start_time.toSec()) > time_thresh_) {
      break;
    }
  }
  optimized_joint_angle = valid_joint_angle;
  optimized_variance = last_variance;
}

void FormOptimization::process()
{
  /*
  for (int i = 0; i < extra_module_num_; i++) {
    addExtraModule(extra_module_link_num_.at(i), extra_module_mass_.at(i), extra_module_offset_.at(i));
  } 
  */ 
  std::vector<std::pair<std::vector<double>, double> > joint_angle_and_variance(thread_num_);
  std::vector<std::thread> threads;
  std::vector<std::vector<double> > initial_joint_angle(thread_num_);

  for (int i = 0; i < thread_num_; i++) {
    initial_joint_angle.at(i).resize(joint_num_);
    for (int j = 0; j < joint_num_; j++) {
      if (!use_initial_joint_angle_) {
	initial_joint_angle.at(i).at(j) = i * 0.1;
      } else {
	initial_joint_angle.at(i).at(j) = initial_joint_angle_.at(j);
      }
    }
  }
  
  for (int i = 0; i < thread_num_; i++) {
    threads.push_back(std::thread(&FormOptimization::steepestDescent, this, initial_joint_angle.at(i), std::ref(joint_angle_and_variance.at(i).first), std::ref(joint_angle_and_variance.at(i).second)));
  }
  
  for (int i = 0; i < threads.size(); i++) {
    if (threads.at(i).joinable()) {
      threads.at(i).join();
    }
  }
  
  //result 
  double min_variance = joint_angle_and_variance.at(0).second;
  int min_variance_index = 0;
  for (int i = 1; i < thread_num_; i++) {
    if (min_variance > joint_angle_and_variance.at(i).second) {
      min_variance = joint_angle_and_variance.at(i).second;
      min_variance_index = i;
    }
  }
  std::vector<double> optimized_joint_angle = joint_angle_and_variance.at(min_variance_index).first;
  double optimized_variance = min_variance;

  ROS_INFO("last joint angle");
  for (int i = 0; i < joint_num_; i++) {
    std::cout << optimized_joint_angle.at(i) << " ";
  }
  std::cout << std::endl;
  ROS_INFO("last variance:%f", optimized_variance);

  if (visualization_) {
    visualization(optimized_joint_angle);
  }
}

void FormOptimization::visualization(std::vector<double> joint_angle)
{
  ros::Rate loop_rate(10);
  sensor_msgs::JointState joint_state;
  joint_state.name = joint_names_;
  joint_state.position = joint_angle;
  
  while (ros::ok()) {
    //joint state
    joint_state.header.stamp = ros::Time::now();
    joint_pub_.publish(joint_state);
    
    ros::spinOnce();
    loop_rate.sleep();
    
    //extra module
    std::vector<tf::Vector3> extra_module_origin(extra_module_num_);
    tf::StampedTransform transform;
    try {
      for (int i = 0; i < extra_module_num_; i++) {
	listener.lookupTransform("/link1", std::string("link") + boost::to_string(extra_module_link_num_.at(i)), ros::Time(0), transform);
	tf::Vector3 extra_module_offset_vec(extra_module_offset_.at(i), 0.0, 0.0);
	extra_module_origin.at(i) = transform.getOrigin() + transform.getBasis() * extra_module_offset_vec;
      }
    } catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
       
    visualization_msgs::MarkerArray msg;

    for (int i = 0; i < extra_module_origin.size(); i++) {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "/link1";
      marker.header.stamp = ros::Time::now();
      marker.ns = "extra_module";
      marker.id = i;
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = extra_module_origin.at(i).x();
      marker.pose.position.y = extra_module_origin.at(i).y();
      marker.pose.position.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = marker.scale.y = marker.scale.z = 0.15;
      marker.color.r = 1.0;
      marker.color.g = 0.647;
      marker.color.b = 0.0;
      marker.color.a = 1.0;
      marker.lifetime = ros::Duration(0.1);
      msg.markers.push_back(marker);
    }
    
    visualization_marker_pub_.publish(msg); 
  }
}
    
