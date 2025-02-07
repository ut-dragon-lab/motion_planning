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

#include <tf_conversions/tf_kdl.h>
#include <sensor_msgs/JointState.h>

namespace differential_kinematics
{
  namespace constraint
  {
    class CogMotion :public Base
    {
    public:
      CogMotion()
      {}
      ~CogMotion(){}

      void virtual initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                              boost::shared_ptr<differential_kinematics::Planner> planner,
                              std::string constraint_name,
                              bool orientation, bool full_body)
      {
        Base::initialize(nh, nhp, planner, constraint_name, orientation, full_body);

        nc_ = 6; // cog velocity  cog angular velocity

        nhp_.getParam("cog_velocity_limit", velocity_limit_);
        nhp_.getParam("cog_angular_limit", angular_limit_);

        if(velocity_limit_.size() == 0 || angular_limit_.size() == 0)
          throw std::runtime_error("the size of velocity limit or angular limit is zero");

        /* heuristic condition */
        if(!full_body)
          {
            double relax_rate = 100;
            for_each(velocity_limit_.begin(), velocity_limit_.end(),
                     [&](double &i){i *= relax_rate;});
            for_each(angular_limit_.begin(), angular_limit_.end(),
                     [&](double &i){i *= relax_rate;});
          }

        if(verbose_)
          {
            std::cout << "cog velocity limit: [";
            for(const auto& it: velocity_limit_) std::cout << std::setprecision(3) << it << ", ";
            std::cout <<  "]" << std::endl;

            std::cout << "cog angular limit: [";
            for(const auto& it: angular_limit_) std::cout << std::setprecision(3) << it << ", ";
            std::cout <<  "]" << std::endl;
          }
      }


      bool getConstraint(Eigen::MatrixXd& A, Eigen::VectorXd& lb, Eigen::VectorXd& ub, bool debug = false)
      {
        std::cout<<"cod"<<std::endl;
        const auto robot_model = planner_->getRobotModelPtr();
        const auto inertia = robot_model->getInertia<Eigen::Matrix3d>();

        lb = Eigen::VectorXd::Constant(nc_, 0);
        ub = Eigen::VectorXd::Constant(nc_, 0);
        /* cog */
        ub.head(3) = Eigen::Map<Eigen::Vector3d>(velocity_limit_.data(), 3);
        lb.head(3) = - ub.head(3);
        /* L momentum: approximate to a rigid body has a inertial same with the robot model */
        ub.tail(3) = inertia * Eigen::Map<Eigen::Vector3d>(angular_limit_.data(), 3);
        lb.tail(3) = - ub.tail(3);
        A = Eigen::MatrixXd::Zero(nc_, robot_model->getLinkJointIndices().size() + 6);
        A.topRows(3) = robot_model->getCOGJacobian();
        A.bottomRows(3) = robot_model->getLMomentumJacobian();
        if(!full_body_) A.leftCols(6).setZero();
        if(planner_->getMultilinkType() == motion_type::SE2) A.middleRows(2, 3).setZero();
        if(debug)
          {
            std::cout << "constraint name: " << constraint_name_  << ", A: \n" << A << std::endl;
            std::cout << "constraint name: " << constraint_name_ << ", lb: \n" << lb.transpose() << std::endl;
            std::cout << "constraint name: " << constraint_name_ << ", ub: \n" << ub.transpose() << std::endl;
          }

        return true;
      }

      void result()
      {
      }

      bool directConstraint(){return false;}

    protected:
      std::vector<double> velocity_limit_, angular_limit_;

    };
  };
};

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(differential_kinematics::constraint::CogMotion, differential_kinematics::constraint::Base);

