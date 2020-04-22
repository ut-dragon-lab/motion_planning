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

#include <bspline_ros/bspline_ros.h>

BsplineRos::BsplineRos(ros::NodeHandle nh, ros::NodeHandle nhp):
  nh_(nh), nhp_(nhp)
{
  spline_path_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("reconstructed_path_markers", 1);
  nhp_.param("display_frame", display_frame_, std::string("world"));
}

bool BsplineRos::initialize(bool uniform, double start_time, double end_time, int degree, const  std::vector<std::vector<double> >& control_point_list, const std::vector<double>& knot_point_list)
{
  if (control_point_list.size() <= degree)
    {
      ROS_WARN_STREAM_NAMED("BsplineRos", "BsplineRos: control points " << control_point_list.at(0).size() << " is less than desired degree " << degree);
      return false;
    }

  splines_.clear();

  uniform_ = uniform;
  time_start_ = start_time;
  time_end_ = end_time;
  int control_point_num = control_point_list.size();
  int dim = control_point_list.at(0).size();

  tsBSplineType flag = TS_NONE;
  if (uniform_) flag = TS_CLAMPED;
  splines_.push_back(tinyspline::BSpline(degree, dim, control_point_num, flag));

  /* Set control points value */
  auto control_points = splines_.at(0).ctrlp();
  for (int i = 0; i < control_point_num; ++i)
    {
      for (int j = 0; j < dim; ++j)
        control_points.at(i * dim + j) = control_point_list.at(i).at(j);
    }
  splines_.at(0).setCtrlp(control_points);

  /* Set knots value is needed if not uniform bspline */
  if (!uniform_)
    {
      auto knotpts = splines_.at(0).knots();
      for (int i = 0; i < knotpts.size(); ++i) knotpts.at(i) = knot_point_list.at(i);
      splines_.at(0).setKnots(knotpts);
    }

  /* Initialize all spline as much as possible by deriviation dimension */
  for(int i = 1; i < degree; i++) splines_.push_back(splines_.back().derive());
}

 std::vector<double> BsplineRos::evaluate(double t, int derive)
{
  /* Over range adjust */
  if (t > time_end_)
    t = time_end_;
  else if (t < time_start_)
    t = time_start_;

  /* In uniform mode, curve time is defaultly from 0.0 to 1.0 */
  if (uniform_) t = (t - time_start_) / (time_end_ - time_start_);

  if(derive > splines_.size())
    {
      ROS_ERROR_STREAM_NAMED("BsplineRos", "BsplineRos: derivative dimension " << derive << " exceeds the dim of b-spline " << splines_.size());
      return std::vector<double>();
    }

  std::vector<tinyspline::rational> result = splines_.at(derive).evaluate(t).result();
  /* Scale back */
  std::vector<double> rescaled_result;
  for (auto val: result) rescaled_result.push_back(val / std::pow(time_end_ - time_start_, derive));
  return rescaled_result;
}

void BsplineRos::display3dPath(std::vector<int> indices, int sample_num)
{
  if(indices.size() != 3)
    {
      ROS_ERROR_STREAM_NAMED("BsplineRos", "BsplineRos: the indices size for display 3D path should be 3");
      return;
    }

  visualization_msgs::MarkerArray spline_path;
  double sample_gap = (time_end_ - time_start_) / sample_num;

  visualization_msgs::Marker path_point_marker;
  path_point_marker.type = visualization_msgs::Marker::SPHERE;
  path_point_marker.pose.orientation.w = 1.0;
  path_point_marker.header.frame_id = "world";
  path_point_marker.header.stamp = ros::Time().now();

  for (int i = 0; i < sample_num; ++i)
    {
      auto result = evaluate(time_start_ + i * sample_gap);
      path_point_marker.id = i;
      path_point_marker.pose.position.x = result.at(indices.at(0));
      path_point_marker.pose.position.y = result.at(indices.at(1));
      path_point_marker.pose.position.z = result.at(indices.at(2));

      if (i == 0 || i == sample_num - 1)
        {
          path_point_marker.scale.x = 0.1;
          path_point_marker.scale.y = 0.1;
          path_point_marker.scale.z = 0.1;
          path_point_marker.color.a = 1;

          if (i == 0)
            {
              path_point_marker.color.r = 0.0f;
              path_point_marker.color.g = 0.0f;
              path_point_marker.color.b = 1.0f;
            }
          else
            {
              path_point_marker.color.r = 1.0f;
              path_point_marker.color.g = 0.0f;
              path_point_marker.color.b = 0.0f;
            }
        }
      else
        {
          path_point_marker.scale.x = 0.05;
          path_point_marker.scale.y = 0.05;
          path_point_marker.scale.z = 0.05;
          path_point_marker.color.a = 1;
          path_point_marker.color.r = 0.0f;
          path_point_marker.color.g = 1.0f;
          path_point_marker.color.b = 0.0f;
        }
      spline_path.markers.push_back(path_point_marker);
    }
  spline_path_pub_.publish(spline_path);
}

