// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, JSK Lab
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

#ifndef CONTINUOUS_PATH_GENERATOR_H
#define CONTINUOUS_PATH_GENERATOR_H

#include <ros/ros.h>
#include <dragon/model/hydrus_like_robot_model.h> // TODO: change to full vectoring model
#include <aerial_motion_planning_msgs/multilink_state.h>

/* continous path generator */
#include <kalman_filter/lpf_filter.h>
#include <bspline_ros/bspline_ros.h>

class ContinuousPathGenerator
{
public:
  ContinuousPathGenerator() {}
  ContinuousPathGenerator(ros::NodeHandle nh, ros::NodeHandle nhp, boost::shared_ptr<aerial_robot_model::transformable::RobotModel> robot_model_ptr);
  ~ContinuousPathGenerator() {}

  const double getPathDuration() const { return bspline_->getEndTime();}
  const std::vector<double> getPositionVector(double t);
  const std::vector<double> getVelocityVector(double t);

  /* discrete path refine methods */
  const std::vector<MultilinkState> discretePathSmoothing(const std::vector<MultilinkState>& raw_path) const ;
  const std::vector<MultilinkState> discretePathResampling(const std::vector<MultilinkState>& raw_path) const;

  /* bspline based */
  void calcContinuousPath(const std::vector<MultilinkState>& discrete_path, double trajectory_period);

protected:

  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;

  bool debug_verbose_;

  boost::shared_ptr<aerial_robot_model::transformable::RobotModel> robot_model_ptr_;

  /* continuous path generator */
  boost::shared_ptr<BsplineRos> bspline_;

};


#endif
