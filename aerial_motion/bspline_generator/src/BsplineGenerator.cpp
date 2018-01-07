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

#include <bspline_generator/BsplineGenerator.h>

namespace bspline_generator{
  BsplineGenerator::BsplineGenerator(ros::NodeHandle nh, ros::NodeHandle nhp, double period, int bspline_degree){
    nh_ = nh;
    nhp_ = nhp;
    period_ = period;
    deg_ = bspline_degree;
    sampling_plannar_client_ = nh_.serviceClient<gap_passing::Keyposes>("keyposes_server");
    bspline_ptr_ = boost::shared_ptr<TinysplineInterface>(new TinysplineInterface(nh_, nhp_, std::string("/path"),
                                                                                  std::string("/nav"))); // Hydrus odometry frame id is /nav

    control_pts_ptr_ = new ControlPoints();
    control_pts_ptr_->control_pts.layout.dim.push_back(std_msgs::MultiArrayDimension());
    control_pts_ptr_->control_pts.layout.dim.push_back(std_msgs::MultiArrayDimension());
    control_pts_ptr_->control_pts.layout.dim[0].label = "height";
    control_pts_ptr_->control_pts.layout.dim[1].label = "width";

    sleep(1);
    waitForKeyposes();
  }

  BsplineGenerator::~BsplineGenerator(){
  }

  void BsplineGenerator::waitForKeyposes(){
    gap_passing::Keyposes keyposes_srv;
    keyposes_srv.request.inquiry = true;
    while (!sampling_plannar_client_.call(keyposes_srv)){
      // waiting for keyposes
    }
    if (!keyposes_srv.response.available_flag){
      ROS_WARN("keyposes is not available, try again");
      waitForKeyposes();
    }
    else if (keyposes_srv.response.states_cnt == 0){
      ROS_WARN("keyposes size is 0, try again");
      waitForKeyposes();
    }
    else{
      control_pts_ptr_->num = keyposes_srv.response.states_cnt + 2;
      control_pts_ptr_->dim = keyposes_srv.response.dim;
      control_pts_ptr_->degree = deg_;
      control_pts_ptr_->is_uniform = true; // todo
      control_pts_ptr_->start_time = 0.0;
      control_pts_ptr_->end_time = period_;

      control_pts_ptr_->control_pts.layout.dim[0].size = control_pts_ptr_->num;
      control_pts_ptr_->control_pts.layout.dim[1].size = control_pts_ptr_->dim;
      control_pts_ptr_->control_pts.layout.dim[0].stride = control_pts_ptr_->num * control_pts_ptr_->dim;
      control_pts_ptr_->control_pts.layout.dim[1].stride = control_pts_ptr_->dim;
      control_pts_ptr_->control_pts.layout.data_offset = 0;

      control_pts_ptr_->control_pts.data.resize(0);

      for (int i = -1; i < keyposes_srv.response.states_cnt + 1; ++i){
        // add one more start & end keypose to guarantee speed 0
        int id = i;
        if (id < 0)
          id = 0;
        else if (id >= keyposes_srv.response.states_cnt)
          id = keyposes_srv.response.states_cnt - 1;
        int index_s = id * keyposes_srv.response.dim;
        for (int j = 0; j < keyposes_srv.response.dim; ++j)
          control_pts_ptr_->control_pts.data.push_back(keyposes_srv.response.data.data[index_s + j]);
      }

      displayBspline();
    }
  }

  void BsplineGenerator::manuallySetControlPts(){
    control_pts_ptr_->num = 5;
    control_pts_ptr_->dim = 3;
    control_pts_ptr_->degree = 2;
    control_pts_ptr_->is_uniform = false;

    control_pts_ptr_->control_pts.layout.dim[0].size = control_pts_ptr_->num;
    control_pts_ptr_->control_pts.layout.dim[1].size = control_pts_ptr_->dim;
    control_pts_ptr_->control_pts.layout.dim[0].stride = control_pts_ptr_->num * control_pts_ptr_->dim;
    control_pts_ptr_->control_pts.layout.dim[1].stride = control_pts_ptr_->dim;
    control_pts_ptr_->control_pts.layout.data_offset = 0;

    // pt1
    control_pts_ptr_->control_pts.data.push_back(-2.0);
    control_pts_ptr_->control_pts.data.push_back(4.0);
    control_pts_ptr_->control_pts.data.push_back(1.0);
    // pt2
    control_pts_ptr_->control_pts.data.push_back(0.0);
    control_pts_ptr_->control_pts.data.push_back(1.0);
    control_pts_ptr_->control_pts.data.push_back(1.0);
    // pt3
    control_pts_ptr_->control_pts.data.push_back(2.0);
    control_pts_ptr_->control_pts.data.push_back(4.0);
    control_pts_ptr_->control_pts.data.push_back(1.0);
    // pt4
    control_pts_ptr_->control_pts.data.push_back(5.0);
    control_pts_ptr_->control_pts.data.push_back(3.0);
    control_pts_ptr_->control_pts.data.push_back(1.0);
    // pt5
    control_pts_ptr_->control_pts.data.push_back(6.0);
    control_pts_ptr_->control_pts.data.push_back(5.0);
    control_pts_ptr_->control_pts.data.push_back(1.0);

    int knots_num = control_pts_ptr_->num + control_pts_ptr_->degree + 1;
    for (int i = 0; i <= control_pts_ptr_->degree; ++i)
      control_pts_ptr_->knots.data.push_back(0.0);
    // mid
    control_pts_ptr_->knots.data.push_back(0.4);
    control_pts_ptr_->knots.data.push_back(0.8);
    for (int i = 0; i <= control_pts_ptr_->degree; ++i)
      control_pts_ptr_->knots.data.push_back(1.0);
  }

  void BsplineGenerator::displayBspline(){
    bspline_ptr_->bsplineParamInput(control_pts_ptr_);
    bspline_ptr_->getDerive();
    bspline_ptr_->splinePathDisplay();
    bspline_ptr_->controlPolygonDisplay();
    ROS_INFO("Spline display finished.");
  }

  double BsplineGenerator::getContinousYaw(double yaw, int id){
    static double prev_yaw = 0.0;
    double new_yaw = yaw;
    if (id == 0){
      prev_yaw = new_yaw;
    }
    else{
      if (fabs(new_yaw - prev_yaw) > 3.0){ // jumping gap in yaw angle
        if (new_yaw > prev_yaw){
          while (fabs(new_yaw - prev_yaw) > 3.0){ // Adjust yaw
            new_yaw -= 2 * PI;
            if (new_yaw < prev_yaw - 2 * PI){ // adjust overhead
              ROS_ERROR("Could not find suitable yaw. previous yaw: %f, current yaw: %f", prev_yaw, yaw);
              new_yaw += 2 * PI;
              break;
            }
          }
        }
        else{
          while (fabs(new_yaw - prev_yaw) > 3.0){
            new_yaw += 2 * PI;
            if (new_yaw > prev_yaw + 2 * PI){
              ROS_ERROR("Could not find suitable yaw. previous yaw: %f, current yaw: %f", prev_yaw, yaw);
              new_yaw -= 2 * PI;
              break;
            }
          }
        }
      }
      prev_yaw = new_yaw;
    }
    return new_yaw;
  }

  std::vector<double> BsplineGenerator::getPosition(double time){
    return bspline_ptr_->evaluate(time);
  }

  std::vector<double> BsplineGenerator::getVelocity(double time){
    return bspline_ptr_->evaluateDerive(time);
  }

  std::vector<double> BsplineGenerator::getKeypose(int id){
    std::vector<double> keypose;
    int start_index = id * control_pts_ptr_->dim;
    if (start_index < 0){
      ROS_ERROR("[BsplineGenerator] visited keypose id%d is less than 0.", id);
      start_index = 0;
    }
    else if (start_index > control_pts_ptr_->control_pts.data.size() - control_pts_ptr_->dim){
      ROS_ERROR("[BsplineGenerator] visited keypose id%d is larger than bound.", id);
      start_index = control_pts_ptr_->control_pts.data.size() - control_pts_ptr_->dim;
    }
    for (int i = 0; i < control_pts_ptr_->dim; ++i)
      keypose.push_back(control_pts_ptr_->control_pts.data[start_index + i]);

    return keypose;
  }
}
