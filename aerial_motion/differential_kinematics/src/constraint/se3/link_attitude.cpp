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
      LinkAttitude(): result_roll_max_(0), result_pitch_max_(0)
      {}
      ~LinkAttitude(){}

      void virtual initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                              boost::shared_ptr<differential_kinematics::Planner> planner,
                              std::string constraint_name,
                              bool orientation, bool full_body)
      {
        Base::initialize(nh, nhp, planner, constraint_name, orientation, full_body);
        nc_ = 2 * rotor_num_; //roll + pitch

        nhp_.param ("roll_angle_thre", roll_angle_thre_, 1.0);
        if(verbose_) std::cout << "roll_angle_thre: " << std::setprecision(3) << roll_angle_thre_ << std::endl;
        nhp_.param ("pitch_angle_thre", pitch_angle_thre_, 1.0);
        if(verbose_) std::cout << "pitch_angle_thre: " << std::setprecision(3) << pitch_angle_thre_ << std::endl;

        nhp_.param ("attitude_change_vel_thre", attitude_change_vel_thre_, 0.1);
        if(verbose_) std::cout << "attitude_change_vel_thre: " << std::setprecision(3) << attitude_change_vel_thre_ << std::endl;
        nhp_.param ("attitude_constraint_range", attitude_constraint_range_, 0.17);
        if(verbose_) std::cout << "attitude_constraint_range: " << std::setprecision(3) << attitude_constraint_range_ << std::endl;
        nhp_.param ("attitude_forbidden_range", attitude_forbidden_range_, 0.02);
        if(verbose_) std::cout << "attitude_forbidden_range: " << std::setprecision(3) << attitude_forbidden_range_ << std::endl;

      }

      bool getConstraint(Eigen::MatrixXd& A, Eigen::VectorXd& lb, Eigen::VectorXd& ub, bool debug = false)
      {
        double j_ndof = planner_->getRobotModelPtr()->getLinkJointIndices().size();
        A = Eigen::MatrixXd::Zero(nc_, j_ndof + 6);
        lb = Eigen::VectorXd::Constant(nc_, -attitude_change_vel_thre_);
        ub = Eigen::VectorXd::Constant(nc_, attitude_change_vel_thre_);

        /* 1. calculate the matrix converting from euler to angular velocity */
        KDL::Rotation root_att;
        tf::quaternionTFToKDL(planner_->getTargetRootPose().getRotation(), root_att);

        Eigen::Matrix3d r_root;
        tf::matrixTFToEigen(planner_->getTargetRootPose().getBasis(), r_root);
        //planner_->getTargetRootPose().getBasis().getRPY(r, p, y);

        /* 2. get jacobian w.r.t root link */
        Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(3, j_ndof + 6);

        /* calculate the jacobian */
        KDL::TreeJntToJacSolver jac_solver(planner_->getRobotModelPtr()->getTree());
        KDL::Jacobian jac(planner_->getRobotModelPtr()->getTree().getNrOfJoints());

        if(jac_solver.JntToJac(planner_->getTargetJointVector<KDL::JntArray>(),
                               jac, std::string("link") + std::to_string(rotor_num_)) == KDL::SolverI::E_NOERROR)
          {
            if(debug) std::cout << "raw jacobian: \n" << jac.data << std::endl;

            /* joint reassignment  */
            for(size_t i = 0; i < j_ndof; i++)
              jacobian.block(0, 6 + i , 3, 1) = jac.data.block(3, planner_->getRobotModelPtr()->getLinkJointIndices().at(i), 3, 1);

            /* full body */
            if(full_body_)
              jacobian.block(0, 3, 3, 3) = Eigen::MatrixXd::Identity(3, 3);

            if(debug) std::cout << "jacobian: \n" << jacobian << std::endl;
          }
        else
          {
            ROS_WARN("constraint (%s) can not calculate the jacobian", constraint_name_.c_str());
            return false;
          }

        /* set A and lb/ub */
        auto full_fk_result = planner_->getRobotModelPtr()->fullForwardKinematics(planner_->getTargetJointVector<KDL::JntArray>());
        int joint_offset = j_ndof / (rotor_num_ - 1 );
        std::vector<double> roll_vec; // for debug
        std::vector<double> pitch_vec; // for debug
        std::vector<KDL::Frame> f_link_vec; // for debug

        for(int index = 0; index < rotor_num_; index++)
          {
            /* lb/ub */
            KDL::Frame f_link = full_fk_result.at(std::string("link") + std::to_string(index + 1));
            f_link_vec.push_back(f_link);
            double r,p,y;
            (root_att * f_link.M).GetRPY(r, p, y);
            if(debug) std::cout << std::string("link") + std::to_string(index + 1) << ": roll: " << r << ", pitch: " << p << std::endl;
            Eigen::Matrix3d r_convert;
            r_convert << 0, - sin(y), cos(y) * cos(p), 0, cos(y), sin(y) * cos(p), 1, 0, -sin(p);
            if(debug) std::cout << "constraint (" << constraint_name_.c_str()  << "): r_convert inverse: \n" << r_convert.inverse() << std::endl;
            Eigen::MatrixXd jacobian_euler = r_convert.inverse() * r_root * jacobian;
            //if(debug) std::cout << "constraint (" << constraint_name_.c_str()  << "): jacobian_euler: \n" << jacobian_euler << std::endl;

            /* yaw -> pitch -> roll => pitch -> roll */
            /* pitch -> roll */
            A.block(index * 2, 0, 2, 6 + index * joint_offset) = jacobian_euler.block(1, 0, 2, 6 + index * joint_offset);

            /* pitch */
            if(p - (-pitch_angle_thre_) < attitude_constraint_range_)
              lb(index * 2 ) *= ((p - (-pitch_angle_thre_) - attitude_forbidden_range_ ) / (attitude_constraint_range_ - attitude_forbidden_range_));
            if(pitch_angle_thre_ - p < attitude_constraint_range_)
              ub(index * 2 ) *= ((pitch_angle_thre_ - p - attitude_forbidden_range_ ) / (attitude_constraint_range_ - attitude_forbidden_range_));
            /* roll */
            if(r - (-roll_angle_thre_) < attitude_constraint_range_)
              lb(index * 2 + 1) *= ((r - (-roll_angle_thre_) - attitude_forbidden_range_ ) / (attitude_constraint_range_ - attitude_forbidden_range_));
            if(roll_angle_thre_ - r < attitude_constraint_range_)
              ub(index * 2 + 1) *= ((roll_angle_thre_ - r - attitude_forbidden_range_ ) / (attitude_constraint_range_ - attitude_forbidden_range_));

            /* update result */
            if(result_roll_max_ < fabs(r))
              {
                result_roll_max_ = fabs(r);
                result_roll_max_link_ = index + 1;
              }
            if(result_pitch_max_ < fabs(p))
              {
                result_pitch_max_ = fabs(p);
                result_pitch_max_link_ = index + 1;
              }

            /* for debug */
            roll_vec.push_back(r);
            pitch_vec.push_back(p);
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
                  << "max roll angle: " << result_roll_max_ << " at link" << result_roll_max_link_ << "\n"
                  << "max pitch angle: " << result_pitch_max_ << " at link" << result_pitch_max_link_
                  << std::endl;
      }

      bool directConstraint(){return false;}

    protected:
      double roll_angle_thre_;
      double pitch_angle_thre_;
      double attitude_change_vel_thre_;
      double attitude_constraint_range_;
      double attitude_forbidden_range_;

      double result_roll_max_;
      double result_pitch_max_;
      int result_roll_max_link_;
      int result_pitch_max_link_;

    };
  };
};

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(differential_kinematics::constraint::LinkAttitude, differential_kinematics::constraint::Base);

