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

#ifndef SE3_MOTION_PLANNING_H_
#define SE3_MOTION_PLANNING_H_

#include <sampling_based_method/se2/motion_planning.h>
#include <dragon/transform_control.h>
#include <ompl/base/spaces/SE3StateSpace.h>

#include <kdl_conversions/kdl_msg.h>
namespace sampling_base
{
  namespace se3
  {
    class MotionPlanning :public se2::MotionPlanning
    {

    public:
      MotionPlanning(ros::NodeHandle nh, ros::NodeHandle nhp, boost::shared_ptr<HydrusRobotModel> robot_model_ptr);
      ~MotionPlanning(){};

      static const uint8_t HORIZONTAL_GAP = 0;
      static const uint8_t VERTICAL_GAP = 1;

    private:

      /* gap env */
      int gap_type_;
      int locomotion_mode_;
      double max_force_;
      int max_force_state_;
      double z_low_bound_;
      double z_high_bound_;
      std::vector<double> joint_lower_limits_, joint_upper_limits_;

      void planInit();
      bool isStateValid(const ompl::base::State *state);

      void rosParamInit();
      void gapEnvInit();
      void addState(ompl::base::State *ompl_state);
    };

  };
};
#endif
