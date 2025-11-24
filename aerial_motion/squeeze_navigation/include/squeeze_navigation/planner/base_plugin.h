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

#ifndef SQUEEZE_MOTION_PLANNER_PLUGIN_H
#define SQUEEZE_MOTION_PLANNER_PLUGIN_H

#include <ros/ros.h>
#include <dragon/model/hydrus_like_robot_model.h> // TODO: change to full vectoring model
#include <aerial_motion_planning_msgs/multilink_state.h>
#include <aerial_motion_planning_msgs/continuous_path_generator.h>

namespace squeeze_motion_planner
{
  class Base
  {
  public:
    Base(): discrete_path_(0) {}
    ~Base() {}

    void virtual initialize(ros::NodeHandle nh, ros::NodeHandle nhp, boost::shared_ptr<aerial_robot_model::transformable::RobotModel> robot_model_ptr)
    {
      nh_ = nh;
      nhp_ = nhp;

      robot_model_ptr_ = robot_model_ptr;
      baselink_name_ = robot_model_ptr_->getBaselinkName();

      nhp_.param("debug_verbose", debug_verbose_, false);

      /* continous path generator */
      continuous_path_generator_ = boost::make_shared<ContinuousPathGenerator>(nh_, nhp_, robot_model_ptr_);
    }

    bool plan()
    {
      if(!corePlan()) return false;

      /* post process */

      /*-- 1. smoothing the discrete path --*/
      bool discrete_path_filter_flag;
      nhp_.param("discrete_path_filter_flag", discrete_path_filter_flag, false);
      if(discrete_path_filter_flag) discrete_path_ = continuous_path_generator_->discretePathSmoothing(discrete_path_);

      /*-- 2. resampling the discrete path --*/
      bool discrete_path_resampling_flag;
      nhp_.param("discrete_path_resampling_flag", discrete_path_resampling_flag, false);
      if(discrete_path_resampling_flag) discrete_path_ = continuous_path_generator_->discretePathResampling(discrete_path_);

      /*-- 3. get continous path --*/
      double squeeze_trajectory_period;
      nhp_.param("squeeze_trajectory_period", squeeze_trajectory_period, 100.0);
      continuous_path_generator_->calcContinuousPath(discrete_path_, squeeze_trajectory_period);

      return true;
    }


    virtual void setInitState(const MultilinkState& state) = 0;
    virtual void reset() { discrete_path_.clear(); }

    virtual bool corePlan() = 0;
    virtual bool loadPath() { return false; }
    virtual void visualizeFunc() = 0;
    virtual void checkCollision(MultilinkState state) = 0;

    const std::vector<MultilinkState>& getDiscretePath() const { return discrete_path_;}
    virtual const MultilinkState& getDiscreteState(int index) const { return discrete_path_.at(index); }

    /* continuous path */
    const boost::shared_ptr<ContinuousPathGenerator> getContinuousPath() const { return continuous_path_generator_;}
    const double getPathDuration() const { return continuous_path_generator_->getPathDuration();}
    const std::vector<double> getPositionVector(double t) { return continuous_path_generator_->getPositionVector(t); }
    const std::vector<double> getVelocityVector(double t) { return continuous_path_generator_->getVelocityVector(t); }

    static const uint8_t HORIZONTAL_GAP = 0;
    static const uint8_t VERTICAL_GAP = 1;

    const tf::Transform& getOpenningCenterFrame() const { return openning_center_frame_;}
    void setOpenningCenterFrame(const tf::Transform& openning_center_frame)
    {
      ROS_ERROR("set openning center");
      openning_center_frame_ = openning_center_frame;
    }

  protected:

    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;

    bool debug_verbose_;

    boost::shared_ptr<aerial_robot_model::transformable::RobotModel> robot_model_ptr_;
    std::string baselink_name_;
    tf::Transform openning_center_frame_;

    std::vector<MultilinkState> discrete_path_;
    boost::shared_ptr<ContinuousPathGenerator> continuous_path_generator_;
  };
};

#endif
