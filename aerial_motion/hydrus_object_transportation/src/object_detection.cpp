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

#include <hydrus_object_transportation/object_detection.h>

ObjectDetection::ObjectDetection(ros::NodeHandle nh, ros::NodeHandle nhp) :nh_(nh), nhp_(nhp)
{
  /* ros params */
  nhp_.param("object_pos_pub_topic_name", object_pos_pub_topic_name_, std::string("/object_pos"));
  nhp_.param("object_image_pub_topic_name", object_image_pub_topic_name_, std::string("/object_image"));
  nhp_.param("image_sub_topic_name", image_sub_topic_name_, std::string("/downward_cam/camera/image"));
  nhp_.param("odom_sub_topic_name", odom_sub_topic_name_, std::string("/uav/baselink/odom"));
  nhp_.param("camera_info_sub_topic_name", camera_info_sub_topic_name_, std::string("/downward_cam/camera/camera_info"));
  nhp_.param("contour_area_size", contour_area_size_, 0.08);
  nhp_.param("contour_area_margin", contour_area_margin_, 0.01);
  nhp_.param("camera_height_offset", camera_height_offset_, 0.2);
  nhp_.param("uav_z_offset", uav_z_offset_, 0.1);
  
  /* ros publisher */
  object_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(object_pos_pub_topic_name_, 1);
  object_image_pub_ = nh_.advertise<sensor_msgs::Image>(object_image_pub_topic_name_, 1);

  /* ros subscriber */
  image_sub_ = nh_.subscribe<sensor_msgs::Image>(image_sub_topic_name_, 1, &ObjectDetection::imageCallback, this);
  odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(odom_sub_topic_name_, 1, &ObjectDetection::odomCallback, this);
  camera_info_sub_ = nh_.subscribe<sensor_msgs::CameraInfo>(camera_info_sub_topic_name_, 1, &ObjectDetection::cameraInfoCallback, this);

  /* ros service server */
  srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> >(nhp_);
  dynamic_reconfigure::Server<Config>::CallbackType f = boost::bind(&ObjectDetection::configCallback, this, _1, _2);
  srv_->setCallback (f);

  hsv_lower_bound_[0] = 0;
  hsv_lower_bound_[1] = 7;
  hsv_lower_bound_[2] = 182;
  hsv_upper_bound_[0] = 122;
  hsv_upper_bound_[1] = 187;
  hsv_upper_bound_[2] = 255;

  camera_info_update_ = false;
  odom_update_ = false;
}


ObjectDetection::~ObjectDetection()
{
}

void ObjectDetection::configCallback(Config &new_config, uint32_t level)
{
  if(!new_config.enable) return;
  
  ROS_WARN("New config");
  
  hsv_lower_bound_[0] = new_config.color_hsv_h_min;
  hsv_upper_bound_[0] = new_config.color_hsv_h_max;
  hsv_lower_bound_[1] = new_config.color_hsv_s_min;
  hsv_upper_bound_[1] = new_config.color_hsv_s_max;
  hsv_lower_bound_[2] = new_config.color_hsv_v_min;
  hsv_upper_bound_[2] = new_config.color_hsv_v_max;
}

void ObjectDetection::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg)
{
  camera_fx_ = msg->K[0];
  camera_fy_ = msg->K[4];

  camera_intrinsic_matrix_.setValue(msg->K[0], msg->K[1], msg->K[2], msg->K[3], msg->K[4], msg->K[5], msg->K[6], msg->K[7], msg->K[8]);
  camera_intrinsic_matrix_inv_ = camera_intrinsic_matrix_.inverse();
  
  if(msg->K[0] > 0)
    {
      camera_info_update_ = true;
      camera_info_sub_.shutdown();
    }
}

void ObjectDetection::odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
  odom_update_ = true;
  object_distance_ = msg->pose.pose.position.z + uav_z_offset_ - camera_height_offset_;
}

void ObjectDetection::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  if (!camera_info_update_ || !odom_update_) return;
  
  cv::Mat src_image = cv_bridge::toCvCopy(msg, msg->encoding)->image;
  cv::Mat hsv_image, hsv_image_mask;
  cv::cvtColor(src_image, hsv_image, CV_RGB2HSV);
  cv::inRange(hsv_image, cv::Scalar(hsv_lower_bound_[0], hsv_lower_bound_[1], hsv_lower_bound_[2]), cv::Scalar(hsv_upper_bound_[0], hsv_upper_bound_[1], hsv_upper_bound_[2]), hsv_image_mask);
  
  std::vector<std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(hsv_image_mask, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0,0));

  geometry_msgs::PoseStamped object_pos_msg;
  object_pos_msg.header = msg->header;
  double dist_from_center_min = 1e6;
  int object_contour_index = -1;
  
  for(int i = 0; i < contours.size(); ++i) {
      double real_contour_area = cv::contourArea(contours[i]) * (object_distance_ * object_distance_) / (camera_fx_ * camera_fy_); 
      if(std::abs(real_contour_area - contour_area_size_) < contour_area_margin_) {
	cv::Moments contour_moments = cv::moments(contours[i], true);
	geometry_msgs::Pose pose;
	pose.position.x = contour_moments.m10 / contour_moments.m00;
	pose.position.y = contour_moments.m01 / contour_moments.m00;
	double dist_from_center = (pose.position.x - src_image.cols/2) * (pose.position.x - src_image.cols/2) + (pose.position.y - src_image.rows/2) * (pose.position.y - src_image.rows/2); 
	if (dist_from_center < dist_from_center_min) {
	  pose.orientation.w = real_contour_area;
	  object_pos_msg.pose = pose;
	  dist_from_center_min = dist_from_center;
	  object_contour_index = i;
	}
      }
  }
  
  if (object_contour_index != -1) {
    cv::drawContours(src_image, contours, object_contour_index, cv::Scalar(255, 255, 255), 3);
  }
    
  object_image_pub_.publish(cv_bridge::CvImage(msg->header, msg->encoding, src_image).toImageMsg());

  tf::Vector3 object_uv(object_pos_msg.pose.position.x, object_pos_msg.pose.position.y, 1);
  tf::Vector3 object_pos_in_optical_frame = camera_intrinsic_matrix_inv_ * object_uv * object_distance_;

  object_pos_msg.pose.position.x = object_pos_in_optical_frame.x();
  object_pos_msg.pose.position.y = object_pos_in_optical_frame.y();
  object_pos_msg.pose.position.z = object_pos_in_optical_frame.z();
  object_pos_pub_.publish(object_pos_msg);
}
