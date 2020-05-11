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
    class JointTorque :public Base
    {
    public:
      JointTorque():
        t_min_(1e6), t_max_(0)
      {}
      ~JointTorque(){}

      void virtual initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                              boost::shared_ptr<differential_kinematics::Planner> planner, std::string constraint_name,
                              bool orientation, bool full_body)
      {
        Base::initialize(nh, nhp, planner, constraint_name, orientation, full_body);

        const auto robot_model = planner_->getRobotModelPtr();

        nc_ = robot_model->getLinkJointIndices().size();

        getParam<double>("t_min_thre", t_min_thre_, -1.0);
        getParam<double>("t_max_thre", t_max_thre_, 1.0);
        getParam<double>("torque_vel_thre", torque_vel_thre_, 0.2);
        getParam<double>("torque_constraint_range", torque_constraint_range_, 0.2);
        getParam<double>("torque_forbidden_range", torque_forbidden_range_, 0.1);
      }

      bool getConstraint(Eigen::MatrixXd& A, Eigen::VectorXd& lb, Eigen::VectorXd& ub, bool debug = false)
      {
        auto robot_model = planner_->getRobotModelPtr();
        const auto& joint_indices = robot_model->getJointIndices();
        const auto& link_joint_indices = robot_model->getLinkJointIndices();
        const auto& joint_torque = robot_model->getJointTorque();
        const auto& full_jacobian = robot_model->getJointTorqueJacobian();
        /* fill the lb/ub */
        lb = Eigen::VectorXd::Constant(nc_, -torque_vel_thre_);
        ub = Eigen::VectorXd::Constant(nc_, torque_vel_thre_);
        A = Eigen::MatrixXd::Zero(nc_, full_jacobian.cols());

        int index = 0;

        for(int i = 0; i < joint_indices.size(); i++)
          {
            if(link_joint_indices.at(index) == joint_indices.at(i))
              {
                A.row(index) = full_jacobian.row(i);
                lb(index) = damplingBound(joint_torque(i) - t_min_thre_, -torque_vel_thre_,  torque_constraint_range_,  torque_forbidden_range_);
                ub(index) = damplingBound(t_max_thre_ - joint_torque(i), torque_vel_thre_,  torque_constraint_range_,  torque_forbidden_range_);
                index++;

                if(index == link_joint_indices.size()) break;
              }
          }


        if(!full_body_) A.leftCols(6).setZero();

        if(t_min_ > joint_torque.minCoeff()) t_min_ = joint_torque.minCoeff(&t_min_rotor_);
        if(t_max_ < joint_torque.maxCoeff()) t_max_ = joint_torque.maxCoeff(&t_max_rotor_);

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
                  << "min t: " << t_min_ << " at rotor" << t_min_rotor_ << "\n"
                  << "max t: " << t_max_ << " at rotor" << t_max_rotor_ << std::endl;
      }

      bool directConstraint(){return false;}

    protected:
      double t_min_thre_, t_max_thre_;

      double torque_vel_thre_;
      double torque_constraint_range_;
      double torque_forbidden_range_;

      double t_min_;
      double t_max_;
      Eigen::MatrixXd::Index t_min_rotor_;
      Eigen::MatrixXd::Index t_max_rotor_;
    };
  };
};

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(differential_kinematics::constraint::JointTorque, differential_kinematics::constraint::Base);

