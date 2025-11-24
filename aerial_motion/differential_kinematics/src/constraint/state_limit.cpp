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

namespace differential_kinematics
{
  namespace constraint
  {
    class StateLimit :public Base
    {
    public:
      StateLimit() {}
      ~StateLimit(){}

      void virtual initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                              boost::shared_ptr<differential_kinematics::Planner> planner,
                              std::string constraint_name,
                              bool orientation, bool full_body)
      {
        Base::initialize(nh, nhp, planner, constraint_name, orientation, full_body);
        nc_ = 6 + planner_->getRobotModelPtr()->getLinkJointIndices().size();

        nhp_.param ("root_translational_vel_thre", root_translational_vel_thre_, 0.05);
        if(verbose_) std::cout << "root_translational_vel_thre: " << std::setprecision(3) << root_translational_vel_thre_ << std::endl;
        nhp_.param ("root_rotational_vel_thre", root_rotational_vel_thre_, 0.1);
        if(verbose_) std::cout << "root_rotational_vel_thre: " << std::setprecision(3) << root_rotational_vel_thre_ << std::endl;
        nhp_.param ("joint_vel_thre", joint_vel_thre_, 0.1);
        if(verbose_) std::cout << "joint_vel_thre: " << std::setprecision(3) << joint_vel_thre_ << std::endl;
        nhp_.param ("joint_vel_constraint_range", joint_vel_constraint_range_, 0.2);
        if(verbose_) std::cout << "joint_vel_constraint_range: " << std::setprecision(3) << joint_vel_constraint_range_ << std::endl;
        nhp_.param ("joint_vel_forbidden_range", joint_vel_forbidden_range_, 0.1);
        if(verbose_) std::cout << "joint_vel_forbidden_range: " << std::setprecision(3) << joint_vel_forbidden_range_ << std::endl;

        auto robot_model_ptr = planner_->getRobotModelPtr();
        joint_lower_limits_.resize(robot_model_ptr->getLinkJointNames().size());
        joint_upper_limits_.resize(robot_model_ptr->getLinkJointNames().size());
        for(int i = 0; i < robot_model_ptr->getLinkJointNames().size(); i++)
          {
            nhp_.param(robot_model_ptr->getLinkJointNames().at(i) + std::string("_lower_limit"), joint_lower_limits_.at(i), robot_model_ptr->getLinkJointLowerLimits().at(i));
            nhp_.param(robot_model_ptr->getLinkJointNames().at(i) + std::string("_upper_limit"), joint_upper_limits_.at(i), robot_model_ptr->getLinkJointUpperLimits().at(i));

            if(verbose_) std::cout << robot_model_ptr->getLinkJointNames().at(i) + std::string("_lower_limit") << ": " << std::setprecision(3) << robot_model_ptr->getLinkJointLowerLimits().at(i) << std::endl;
            if(verbose_) std::cout << robot_model_ptr->getLinkJointNames().at(i) + std::string("_upper_limit") << ": " << std::setprecision(3) << robot_model_ptr->getLinkJointUpperLimits().at(i) << std::endl;
          }
      }

      bool getConstraint(Eigen::MatrixXd& A, Eigen::VectorXd& lb, Eigen::VectorXd& ub, bool debug = false)
      {
        int j_ndof = planner_->getRobotModelPtr()->getLinkJointIndices().size();
        A = Eigen::MatrixXd::Zero(nc_, j_ndof + 6);
        lb = Eigen::VectorXd::Constant(nc_, 1);
        ub = Eigen::VectorXd::Constant(nc_, 1);

        /* root */
        if(full_body_)
          {
            lb.segment(0, 3) *= -root_translational_vel_thre_;
            ub.segment(0, 3) *= root_translational_vel_thre_;
            lb.segment(3, 3) *= -root_rotational_vel_thre_;
            ub.segment(3, 3) *= root_rotational_vel_thre_;

            if(planner_->getMultilinkType() == motion_type::SE2)
              {
                lb.segment(2, 3) *= 0;
                ub.segment(2, 3) *= 0;
              }
          }
        else
          {
            lb.head(6) *= 0;
            ub.head(6) *= 0;
          }

        /* joint */
        auto joint_vector = planner_->getTargetJointVector<KDL::JntArray>();

        lb.tail(j_ndof) *= -joint_vel_thre_;
        ub.tail(j_ndof) *= joint_vel_thre_;
        for(int i = 0; i < j_ndof; i ++)
          {
            auto index = planner_->getRobotModelPtr()->getLinkJointIndices().at(i);

            /* min */
            if(joint_vector(index) - planner_->getRobotModelPtr()->getLinkJointLowerLimits().at(i) < joint_vel_constraint_range_)
              lb(i + 6) *= (joint_vector(index) - planner_->getRobotModelPtr()->getLinkJointLowerLimits().at(i) - joint_vel_forbidden_range_) / (joint_vel_constraint_range_ - joint_vel_forbidden_range_);

            /* max */
            if(planner_->getRobotModelPtr()->getLinkJointUpperLimits().at(i) - joint_vector(index)  < joint_vel_constraint_range_)
              ub(i + 6) *= (planner_->getRobotModelPtr()->getLinkJointUpperLimits().at(i) - joint_vector(index) - joint_vel_forbidden_range_) / (joint_vel_constraint_range_ - joint_vel_forbidden_range_);
          }
        if(debug)
          {
            std::cout << "constraint name: " << constraint_name_ << ", lb: \n" << lb.transpose() << std::endl;
            std::cout << "constraint name: " << constraint_name_ << ", ub: \n" << ub.transpose() << std::endl;
          }
        return true;
      }

      void result() {}

      bool directConstraint(){return true;}

    protected:
      double root_translational_vel_thre_;
      double root_rotational_vel_thre_;
      double joint_vel_thre_;
      double joint_vel_constraint_range_;
      double joint_vel_forbidden_range_;

      std::vector<double> joint_lower_limits_, joint_upper_limits_;
    };
  };
};

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(differential_kinematics::constraint::StateLimit, differential_kinematics::constraint::Base);



