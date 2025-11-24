// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, JSK Lab
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

namespace differential_kinematics
{
  namespace constraint
  {
    class StaticThrust :public Base
    {
    public:
      StaticThrust():
        f_min_(1e6), f_max_(0)
      {}
      ~StaticThrust(){}

      void virtual initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                              boost::shared_ptr<differential_kinematics::Planner> planner, std::string constraint_name,
                              bool orientation, bool full_body)
      {
        Base::initialize(nh, nhp, planner, constraint_name, orientation, full_body);

        nc_ = rotor_num_;
        const auto robot_model = planner_->getRobotModelPtr();
        getParam<double>("f_min_thre", f_min_thre_, robot_model->getThrustLowerLimit());
        getParam<double>("f_max_thre", f_max_thre_, robot_model->getThrustUpperLimit());
        getParam<double>("force_vel_thre", force_vel_thre_, 0.1);
        getParam<double>("force_constraint_range", force_constraint_range_, 0.2);
        getParam<double>("force_forbidden_range", force_forbidden_range_, 0.1);
      }

      bool getConstraint(Eigen::MatrixXd& A, Eigen::VectorXd& lb, Eigen::VectorXd& ub, bool debug = false)
      {         
        auto robot_model = planner_->getRobotModelPtr();

        const Eigen::VectorXd& static_thrust = robot_model->getStaticThrust();
        /* fill the lb/ub */
        lb = Eigen::VectorXd::Constant(nc_, -force_vel_thre_);
        ub = Eigen::VectorXd::Constant(nc_, force_vel_thre_);

        for(int index = 0; index < rotor_num_; index++)
          {
            lb(index) = damplingBound(static_thrust(index) - f_min_thre_, -force_vel_thre_,  force_constraint_range_,  force_forbidden_range_);
            ub(index) = damplingBound(f_max_thre_ - static_thrust(index), force_vel_thre_,  force_constraint_range_,  force_forbidden_range_);
            // lb(index) = damplingBound(10.0 - f_min_thre_, -force_vel_thre_,  force_constraint_range_,  force_forbidden_range_);
            // ub(index) = damplingBound(f_max_thre_ - 10.0, force_vel_thre_,  force_constraint_range_,  force_forbidden_range_);
          }
        A = robot_model->getLambdaJacobian();
        if(!full_body_) A.leftCols(6).setZero();

        if(f_min_ > static_thrust.minCoeff()) f_min_ = static_thrust.minCoeff(&f_min_rotor_);
        if(f_max_ < static_thrust.maxCoeff()) f_max_ = static_thrust.maxCoeff(&f_max_rotor_);

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
                  << "min f: " << f_min_ << " at rotor" << f_min_rotor_ << "\n"
                  << "max f: " << f_max_ << " at rotor" << f_max_rotor_ << std::endl;
      }

      bool directConstraint(){return false;}

    protected:
      double f_min_thre_, f_max_thre_;

      double force_vel_thre_;
      double force_constraint_range_;
      double force_forbidden_range_;

      double f_min_;
      double f_max_;
      Eigen::MatrixXd::Index f_min_rotor_;
      Eigen::MatrixXd::Index f_max_rotor_;
    };
  };
};

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(differential_kinematics::constraint::StaticThrust, differential_kinematics::constraint::Base);

