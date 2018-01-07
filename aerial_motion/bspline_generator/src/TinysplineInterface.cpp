// -*- Mode: c++ -*-
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

#include <bspline_generator/TinysplineInterface.h>

namespace tinyspline_interface{
  TinysplineInterface::TinysplineInterface(ros::NodeHandle nh, ros::NodeHandle nhp, std::string spline_path_pub_topic_name, std::string path_frame_id){
    nh_ = nh;
    nhp_ = nhp;
    debug_ = true;
    polygon_display_flag_ = false;
    path_frame_id_ = path_frame_id;

    pub_spline_path_ = nh_.advertise<nav_msgs::Path>(spline_path_pub_topic_name, 1);
    pub_reconstructed_path_markers_ = nh_.advertise<visualization_msgs::MarkerArray>("reconstructed_path_markers", 1);
  }

  void TinysplineInterface::bsplineParamInput(bspline_generator::ControlPoints* msg)
  {
    /* Init */
    if (spline_ptr_)
      delete spline_ptr_;

    is_uniform_ = msg->is_uniform;
    deg_ = msg->degree;
    dim_ = msg->dim;
    if (dim_ < 3)
      ROS_WARN("[TinysplineInterface] Input data dimension is less than 3, display function is infeasible.");

    controlpts_num_ = msg->num;
    knots_num_ = controlpts_num_ + deg_ + 1;

    if (controlpts_num_ <= deg_){
      ROS_WARN("Control points is LESS than degree!");
      return;
    }

    if (is_uniform_)
      spline_ptr_ = new tinyspline::BSpline(deg_, dim_, controlpts_num_, TS_CLAMPED);
    else
      spline_ptr_ = new tinyspline::BSpline(deg_, dim_, controlpts_num_, TS_NONE);

    time_start_ = msg->start_time;
    time_end_ = msg->end_time;

    knotpts_ = spline_ptr_->knots();

    /* Manually setting knots value is needed if not uniform bspline */
    if (!is_uniform_){
      for (int i = 0; i < knots_num_; ++i){
        knotpts_[i] = msg->knots.data[i];
      }
      spline_ptr_->setKnots(knotpts_);
    }

    /* Set control points value */
    controlpts_ = spline_ptr_->ctrlp();
    for (int i = 0; i < controlpts_num_ * dim_; ++i)
      controlpts_[i] = msg->control_pts.data[i];
    spline_ptr_->setCtrlp(controlpts_);

    if (debug_){
      std::cout << "B-spline input control points number: " << controlpts_num_ << "\n";
      std::cout << "Control point num: " << controlpts_num_ << ", degree: "
                << deg_ << "\n";
      std::cout << "Time region: ["<< time_start_ << ", " << time_end_ << "]\n";
      std::cout << "Start position: ";
      for (int i = 0; i < dim_; ++i)
        std::cout << msg->control_pts.data[i] << ", ";
      std::cout << "\nEnd position: ";
      for (int i = 0; i < dim_; ++i)
        std::cout << msg->control_pts.data[i + (controlpts_num_-1) * dim_] << ", ";
      std::cout << "\n[check knots]: \n";
      for (int i = 0; i < knots_num_; ++i)
        std::cout << knotpts_[i] << ", ";
      std::cout << "\n";
    }
  }

  void TinysplineInterface::splinePathDisplay()
  {
    if (dim_ < 3){
      ROS_ERROR("[TinysplineInterface] Input data dimension is less than 3, path display could not work.");
      return;
    }
    nav_msgs::Path spline_path;
    spline_path.header.frame_id = path_frame_id_;
    spline_path.header.stamp = ros::Time().now();
    float sample_gap;
    spline_path.poses.clear();
    int n_sample = 50;
    sample_gap = (time_end_-time_start_) / n_sample;
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = spline_path.header;
    pose_stamped.pose.orientation.x = 0.0f;
    pose_stamped.pose.orientation.y = 0.0f;
    pose_stamped.pose.orientation.z = 0.0f;
    pose_stamped.pose.orientation.w = 1.0f;

    /* transfer between bspline and beziers curve */
    //tinyspline::BSpline beziers = spline_ptr_->derive().toBeziers();

    for (int i = 0; i <= n_sample; ++i){
      std::vector<double> result = evaluate(time_start_ + i*sample_gap);
      pose_stamped.pose.position.x = result[0];
      pose_stamped.pose.position.y = result[1];
      pose_stamped.pose.position.z = result[2];
      spline_path.poses.push_back(pose_stamped);
    }
    pub_spline_path_.publish(spline_path);
  }

  void TinysplineInterface::controlPolygonDisplay(){
    if (dim_ < 3){
      ROS_ERROR("[TinysplineInterface] Input data dimension is less than 3, control polygon display could not work.");
      return;
    }
    if (polygon_display_flag_)
      controlPolygonDisplayInterface(0);
    controlPolygonDisplayInterface(1);
    polygon_display_flag_ = true;
  }

  void TinysplineInterface::controlPolygonDisplayInterface(int mode){
    int control_points_num = controlpts_num_;
    //std::cout << "[Display] Control points number: " << control_points_num << "\n";
    int id_cnt = 0;
    visualization_msgs::MarkerArray path_markers;
    visualization_msgs::Marker control_point_marker, line_list_marker, triangle_list_marker;
    control_point_marker.ns = line_list_marker.ns = "control_polygon";
    control_point_marker.header.frame_id = line_list_marker.header.frame_id = path_frame_id_;
    control_point_marker.header.stamp = line_list_marker.header.stamp = ros::Time().now();
    if (mode == 1)
      control_point_marker.action = line_list_marker.action = visualization_msgs::Marker::ADD;
    else
      control_point_marker.action = line_list_marker.action = visualization_msgs::Marker::DELETE;

    triangle_list_marker.header = line_list_marker.header;
    triangle_list_marker.action = line_list_marker.action;
    triangle_list_marker.ns = line_list_marker.ns;

    control_point_marker.type = visualization_msgs::Marker::SPHERE;
    line_list_marker.type = visualization_msgs::Marker::LINE_LIST;
    triangle_list_marker.type = visualization_msgs::Marker::TRIANGLE_LIST;

    /* triangle edges */
    line_list_marker.id = id_cnt;
    ++id_cnt;
    line_list_marker.scale.x = 0.07;
    line_list_marker.color.r = 0.0;
    line_list_marker.color.g = 1.0;
    line_list_marker.color.b = 0.0;
    line_list_marker.color.a = 0.3;
    geometry_msgs::Point pt;
    arrayConvertToPoint(0, pt);
    line_list_marker.points.push_back(pt);
    arrayConvertToPoint(1, pt);
    line_list_marker.points.push_back(pt);
    for (int i = 2; i < control_points_num; ++i){
      arrayConvertToPoint(i-2, pt);
      line_list_marker.points.push_back(pt);
      arrayConvertToPoint(i, pt);
      line_list_marker.points.push_back(pt);
      arrayConvertToPoint(i-1, pt);
      line_list_marker.points.push_back(pt);
      arrayConvertToPoint(i, pt);
      line_list_marker.points.push_back(pt);
    }
    path_markers.markers.push_back(line_list_marker);

    /* triangle vertices */
    for (int i = 0; i < control_points_num; ++i){
      control_point_marker.id = id_cnt;
      ++id_cnt;
      control_point_marker.pose.position.x = controlpts_[dim_*i];
      control_point_marker.pose.position.y = controlpts_[dim_*i+1];
      control_point_marker.pose.position.z = controlpts_[dim_*i+2];
      control_point_marker.pose.orientation.x = 0.0;
      control_point_marker.pose.orientation.y = 0.0;
      control_point_marker.pose.orientation.z = 0.0;
      control_point_marker.pose.orientation.w = 1.0;
      if (i == 0 || i == control_points_num-1){
        control_point_marker.scale.x = 0.1;
        control_point_marker.scale.y = 0.1;
        control_point_marker.scale.z = 0.1;
        control_point_marker.color.a = 1;
        if (i == 0){
          control_point_marker.color.r = 0.0f;
          control_point_marker.color.g = 0.0f;
          control_point_marker.color.b = 1.0f;
        }
        else{
          control_point_marker.color.r = 1.0f;
          control_point_marker.color.g = 0.0f;
          control_point_marker.color.b = 0.0f;
        }
        path_markers.markers.push_back(control_point_marker);
      }
      else{
        control_point_marker.scale.x = 0.05;
        control_point_marker.scale.y = 0.05;
        control_point_marker.scale.z = 0.05;
        control_point_marker.color.a = 1;
        control_point_marker.color.r = 0.0f;
        control_point_marker.color.g = 1.0f;
        control_point_marker.color.b = 0.0f;
        path_markers.markers.push_back(control_point_marker);
      }
    }

    /* triangle list */
    triangle_list_marker.scale.x = 1.0;
    triangle_list_marker.scale.y = 1.0;
    triangle_list_marker.scale.z = 0.0;
    triangle_list_marker.color.a = 0.3;
    srand (time(NULL));
    for (int i = 2; i < control_points_num; ++i){
      triangle_list_marker.id = id_cnt;
      ++id_cnt;
      triangle_list_marker.color.r = rand() / (double)RAND_MAX * 1.0;
      triangle_list_marker.color.g = rand() / (double)RAND_MAX * 1.0;
      triangle_list_marker.color.b = rand() / (double)RAND_MAX * 1.0;
      arrayConvertToPoint(i-2, pt);
      triangle_list_marker.points.push_back(pt);
      arrayConvertToPoint(i-1, pt);
      triangle_list_marker.points.push_back(pt);
      arrayConvertToPoint(i, pt);
      triangle_list_marker.points.push_back(pt);
      path_markers.markers.push_back(triangle_list_marker);
      triangle_list_marker.points.clear();
    }

    pub_reconstructed_path_markers_.publish(path_markers);
  }

  void TinysplineInterface::getDerive()
  {
    spline_derive_ = spline_ptr_->derive();
  }

  std::vector<double> TinysplineInterface::evaluate(double t)
  {
    /* Over range adjust */
    if (t > time_end_)
      t = time_end_;
    else if (t < time_start_)
      t = time_start_;

    /* In uniform mode, curve time is defaultly from 0.0 to 1.0 */
    if (is_uniform_)
      t = (t - time_start_) / (time_end_ - time_start_);

    std::vector<tinyspline::rational> result = spline_ptr_->evaluate(t).result();
    std::vector<double> res_d;
    for (int i = 0; i < result.size(); ++i)
      res_d.push_back(result[i]);
    return res_d;
  }

  std::vector<double> TinysplineInterface::evaluateDerive(double t)
  {
    /* Over range adjust */
    if (t > time_end_)
      t = time_end_;
    else if (t < time_start_)
      t = time_start_;

    /* In uniform mode, curve time is defaultly from 0.0 to 1.0 */
    if (is_uniform_)
      t = (t - time_start_) / (time_end_ - time_start_);

    std::vector<tinyspline::rational> result = spline_derive_.evaluate(t).result();
    std::vector<double> res_d;
    for (int i = 0; i < result.size(); ++i)
      res_d.push_back(result[i] / (time_end_ - time_start_));
    return res_d;
  }

  void TinysplineInterface::arrayConvertToPoint(int id, geometry_msgs::Point& point){
    point.x = controlpts_[dim_*id];
    point.y = controlpts_[dim_*id+1];
    point.z = controlpts_[dim_*id+2];
  }
}
