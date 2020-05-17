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

#include <differential_kinematics/constraint/base_plugin.h>


/* robot model */
#include <urdf/model.h>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/treejnttojacsolver.hpp>
#include <sensor_msgs/JointState.h>

#include <tf_conversions/tf_eigen.h>
#include <tf_conversions/tf_kdl.h>
#include <sensor_msgs/JointState.h>

namespace differential_kinematics
{
  namespace constraint
  {
    class LinkAttitude :public Base
    {
    public:
      LinkAttitude(): roll_max_(0), pitch_max_(0)
      {}
      ~LinkAttitude(){}

      void virtual initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                              boost::shared_ptr<differential_kinematics::Planner> planner,
                              std::string constraint_name,
                              bool orientation, bool full_body)
      {
        Base::initialize(nh, nhp, planner, constraint_name, orientation, full_body);
        nc_ = 2 * rotor_num_; //roll + pitch

        getParam<double>("roll_angle_thre", roll_angle_thre_, 1.0);
        getParam<double>("pitch_angle_thre", pitch_angle_thre_, 1.0);
        getParam<double>("attitude_change_vel_thre", attitude_change_vel_thre_, 0.1);
        getParam<double>("attitude_constraint_range", attitude_constraint_range_, 0.17);
        getParam<double>("attitude_forbidden_range", attitude_forbidden_range_, 0.02);
      }

      bool getConstraint(Eigen::MatrixXd& A, Eigen::VectorXd& lb, Eigen::VectorXd& ub, bool debug = false)
      {
        auto robot_model = planner_->getRobotModelPtr();
        const auto joint_positions = planner_->getTargetJointVector<KDL::JntArray>();
        const auto& seg_frames = robot_model->getSegmentsTf();
        const auto root_pose = planner_->getTargetRootPose<KDL::Frame>();
        double j_ndof = robot_model->getLinkJointIndices().size();
        A = Eigen::MatrixXd::Zero(nc_, j_ndof + 6);
        lb = Eigen::VectorXd::Constant(nc_, -attitude_change_vel_thre_);
        ub = Eigen::VectorXd::Constant(nc_, attitude_change_vel_thre_);

        for(int i = 0; i < rotor_num_; i++)
          {
            std::string link = std::string("link") + std::to_string(i+1);
            Eigen::MatrixXd jacobian = robot_model->getJacobian(joint_positions, link).bottomRows(3);

            double r,p,y;
            (root_pose.M * seg_frames.at(link).M).GetRPY(r, p, y);
            if(debug) std::cout << link << ": roll: " << r << ", pitch: " << p << std::endl;
            Eigen::Matrix3d r_convert;
            r_convert << 0, - sin(y), cos(y) * cos(p), 0, cos(y), sin(y) * cos(p), 1, 0, -sin(p);
            if(debug) std::cout << "constraint (" << constraint_name_.c_str()  << "): r_convert inverse: \n" << r_convert.inverse() << std::endl;
            jacobian = r_convert.inverse() * jacobian;

            /* yaw -> pitch -> roll => pitch -> roll */
            A.middleRows(i * 2, 2) = jacobian.bottomRows(2);

            /* pitch */
            lb(i * 2) = damplingBound(p - (-pitch_angle_thre_), -attitude_change_vel_thre_, attitude_constraint_range_, attitude_forbidden_range_);
            ub(i * 2) = damplingBound(pitch_angle_thre_ - p, attitude_change_vel_thre_, attitude_constraint_range_, attitude_forbidden_range_);
            /* roll */
            lb(i * 2 + 1) = damplingBound(r - (-roll_angle_thre_), -attitude_change_vel_thre_, attitude_constraint_range_, attitude_forbidden_range_);
            ub(i * 2 + 1) = damplingBound(roll_angle_thre_ - r, attitude_change_vel_thre_, attitude_constraint_range_, attitude_forbidden_range_);


            /* update result */
            if(roll_max_ < fabs(r))
              {
                roll_max_ = fabs(r);
                roll_max_link_ = link;
              }
            if(pitch_max_ < fabs(p))
              {
                pitch_max_ = fabs(p);
                pitch_max_link_ = link;
              }

          }

        if(debug)
          {
            std::cout << "constraint (" << constraint_name_.c_str()  << "): matrix A: \n" << A << std::endl;
            std::cout << "constraint name: " << constraint_name_ << ", lb: \n" << lb.transpose() << std::endl;
            std::cout << "constraint name: " << constraint_name_ << ", ub: \n" << ub.transpose() << std::endl;
          }

        return true;
      }

      void result()
      {
        std::cout << constraint_name_ << "\n"
                  << "max roll angle: " << roll_max_ << " at " << roll_max_link_ << "\n"
                  << "max pitch angle: " << pitch_max_ << " at " << pitch_max_link_
                  << std::endl;
      }

      bool directConstraint(){return false;}

    protected:
      double roll_angle_thre_;
      double pitch_angle_thre_;
      double attitude_change_vel_thre_;
      double attitude_constraint_range_;
      double attitude_forbidden_range_;

      double roll_max_;
      double pitch_max_;
      std::string roll_max_link_;
      std::string pitch_max_link_;

    };
  };
};

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(differential_kinematics::constraint::LinkAttitude, differential_kinematics::constraint::Base);

