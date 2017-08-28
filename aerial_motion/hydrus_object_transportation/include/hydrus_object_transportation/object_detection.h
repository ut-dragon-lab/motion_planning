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

#ifndef OBJECT_DETECTION_H
#define OBJECT_DETECTION_H

#include <ros/ros.h>

/* ros message */
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/CameraInfo.h>

/* cfg */
#include <dynamic_reconfigure/server.h>
#include <hydrus_object_transportation/ObjectDetectionConfig.h>

/* stl */
#include <iostream>
#include <vector>
#include <algorithm>
#include <string>

/* cv */
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

/* tf */
#include <tf/transform_broadcaster.h>

class ObjectDetection
{
public:
  ObjectDetection(ros::NodeHandle nh, ros::NodeHandle nhp);
  ~ObjectDetection();
  typedef hydrus_object_transportation::ObjectDetectionConfig Config;
  
private:
  /* ros */
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;

  /* ros publisher */
  ros::Publisher object_pos_pub_;
  ros::Publisher object_image_pub_;
  
  /* ros subscriber */
  ros::Subscriber image_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber camera_info_sub_;

  /* ros service server */
  boost::shared_ptr<dynamic_reconfigure::Server<Config> > srv_;
  
  /* ros param */
  std::string object_pos_pub_topic_name_;
  std::string image_sub_topic_name_;
  std::string odom_sub_topic_name_;
  std::string camera_info_sub_topic_name_;
  std::string object_image_pub_topic_name_;
  double contour_area_size_, contour_area_margin_;
  double camera_height_offset_;
  double uav_z_offset_;
  
  int hsv_lower_bound_[3];
  int hsv_upper_bound_[3];
  double camera_fx_, camera_fy_;
  tf::Matrix3x3 camera_intrinsic_matrix_, camera_intrinsic_matrix_inv_;
  bool camera_info_update_, odom_update_;
  double object_distance_;
  
  void odomCallback(const nav_msgs::OdometryConstPtr& msg);
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
  void configCallback(Config &new_config, uint32_t level);
  void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);
  
};

#endif
