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

#include <squeeze_navigation/planner/base_plugin.h>
#include <sampling_based_method/se3/motion_planning.h>

namespace squeeze_motion_planner
{
  class SamplingBasedMethod : public Base
  {
  public:
    SamplingBasedMethod(){}
    ~SamplingBasedMethod() {}

    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp, boost::shared_ptr<TransformController> robot_model_ptr)
    {
      Base::initialize(nh, nhp, robot_model_ptr);
      int motion_type;
      nhp_.param("motion_type", motion_type, 0);
      if (motion_type == motion_type::SE2) //SE2
        planner_core_ = boost::shared_ptr<sampling_base::se2::MotionPlanning>(new sampling_base::se2::MotionPlanning(nh, nhp, robot_model_ptr_));
      else // Se3
        planner_core_ = boost::shared_ptr<sampling_base::se2::MotionPlanning>(new sampling_base::se3::MotionPlanning(nh, nhp, robot_model_ptr_));
    };

    const std::vector<MultilinkState>& getPathConst() const
    { planner_core_->getPathConst(); }

    const MultilinkState& getStateConst(int index) const
    { planner_core_->getStateConst(index); }

    bool plan (bool debug) {return planner_core_->plan();}
    bool loadPath() { return planner_core_->loadPath();}

    void visualizeFunc() {}
    void checkCollision(MultilinkState state) { planner_core_->checkCollision(state); }

  private:
    boost::shared_ptr<sampling_base::se2::MotionPlanning> planner_core_;
  };
};

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(squeeze_motion_planner::SamplingBasedMethod, squeeze_motion_planner::Base);
