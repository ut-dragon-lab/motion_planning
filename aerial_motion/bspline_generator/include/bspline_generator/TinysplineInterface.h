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

#ifndef BSPLINE_GENERATE_H_
#define BSPLINE_GENERATE_H_
#include <iostream>
/*we need set the following flag to disable c++11 for linking the tinyspline */
#define TINYSPLINE_DISABLE_CXX11_FEATURES
#include <tinysplinecpp.h>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <unistd.h>
#include <stdlib.h>
#include <bspline_generator/ControlPoints.h>

namespace tinyspline_interface{
  class TinysplineInterface
  {
  public:
    tinyspline::BSpline* spline_ptr_;
    tinyspline::BSpline spline_derive_;
    std::vector<tinyspline::rational> controlpts_;
    std::vector<tinyspline::rational> knotpts_;
    int controlpts_num_;
    int knots_num_;
    int deg_;
    int dim_;
    bool is_uniform_;
    float time_start_;
    float time_end_;
    bool polygon_display_flag_;
    bool debug_;
    std::string path_frame_id_;

    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;

    ros::Publisher pub_spline_path_;
    ros::Publisher pub_reconstructed_path_markers_;

    TinysplineInterface(ros::NodeHandle nh, ros::NodeHandle nhp, std::string spline_path_pub_topic_name = std::string("/spline_path"),
                        std::string path_frame = std::string("/world"));
    void pathGridPointsCallback(const bspline_generator::ControlPointsConstPtr& msg);
    void splinePathDisplay();
    void bsplineParamInput(bspline_generator::ControlPoints* msg);
    void getDerive();
    std::vector<double> evaluate(double t);
    std::vector<double> evaluateDerive(double t);
    void controlPolygonDisplay();
    void controlPolygonDisplayInterface(int mode = 1);
    void arrayConvertToPoint(int id, geometry_msgs::Point& point);
  };
}
#endif
