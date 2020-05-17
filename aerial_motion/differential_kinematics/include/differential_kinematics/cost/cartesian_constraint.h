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

#ifndef DIFFERENTIA_KINEMATICS_CARTESIAN_CONSTRAINT_PLUGIN_H
#define DIFFERENTIA_KINEMATICS_CARTESIAN_CONSTRAINT_PLUGIN_H

#include <differential_kinematics/cost/base_plugin.h>

/* kinemtiacs */
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>

/* linear math */
#include <tf_conversions/tf_kdl.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>

namespace MaskAxis
{
  enum
    {
      TRAN_X,
      TRAN_Y,
      TRAN_Z,
      ROT_X,
      ROT_Y,
      ROT_Z,
    };
};

namespace differential_kinematics
{
  namespace cost
  {
    class CartersianConstraint :public Base
    {

    public:
      CartersianConstraint()
      {
        free_axis_mask_ = Eigen::VectorXd::Ones(6);
      }

      ~CartersianConstraint(){}

      void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                      boost::shared_ptr<differential_kinematics::Planner> planner, std::string cost_name,
                      bool orientation, bool full_body);

      bool getHessianGradient(bool& convergence, Eigen::MatrixXd& H, Eigen::VectorXd& g, bool debug = false);

      void setReferenceFrame(const std::string parent_link, const KDL::Frame reference_frame)
      {
        parent_link_ = parent_link;
        reference_frame_ = reference_frame;
      }

      void setReferenceFrame(const std::string parent_link, const tf::Transform reference_frame)
      {
        KDL::Frame reference_frame_kdl;
        tf::transformTFToKDL(reference_frame, reference_frame_kdl);
        setReferenceFrame(parent_link, reference_frame_kdl);
      }

      void setTargetFrame(const KDL::Frame target_frame)
      {
        target_frame_ = target_frame;
      }

      void setTargetFrame(const tf::Transform target_frame)
      {
        KDL::Frame target_frame_kdl;
        tf::transformTFToKDL(target_frame, target_frame_kdl);
        setTargetFrame(target_frame_kdl);
      }

      void setFreeAxis(const std::vector<int>& free_axis_list)
      {
        resetAxisMask();

        for(const auto& axis: free_axis_list)
          {
            if(axis > MaskAxis::ROT_Z)
              {
                ROS_ERROR("the free axis index: %d is invalid", axis);
                return;
              }
            free_axis_mask_(axis) = 0;
          }
      }


      void setFreeAxis(int free_axis)
      {
        if(free_axis > MaskAxis::ROT_Z)
          {
            ROS_ERROR("the free axis index: %d is invalid", free_axis);
            return;
          }

        resetAxisMask();

        free_axis_mask_(free_axis) = 0;
      }

      void resetAxisMask() { free_axis_mask_ = Eigen::VectorXd::Ones(6); }

    protected:
      double pos_convergence_thre_;
      double rot_convergence_thre_;
      double pos_err_max_;
      double angle_err_max_;
      double w_pos_err_constraint_;
      double w_att_err_constraint_;

      // KDL::Chain chain_;
      //std::vector<int> chain_joint_index_;

      std::string parent_link_;
      Eigen::MatrixXd W_cartesian_err_constraint_;
      KDL::Frame reference_frame_;
      KDL::Frame target_frame_;
      Eigen::VectorXd free_axis_mask_;


      bool calcJointJacobian(Eigen::MatrixXd& jacobian, bool debug);
    };
  };
};

#endif
