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
    class Stability :public Base
    {
    public:
      Stability():
        result_f_min_(1e6), result_f_max_(0),
        result_p_det_min_(1e6), result_stability_margin_min_(1e6)
      {}
      ~Stability(){}

      void virtual initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                              boost::shared_ptr<differential_kinematics::Planner> planner, std::string constraint_name,
                              bool orientation, bool full_body)
      {
        Base::initialize(nh, nhp, planner, constraint_name, orientation, full_body);

        nc_ = 2 + rotor_num_;

        stability_margin_thre_ = planner_->getRobotModelPtr()->getStabilityMaginThresh();
        p_det_thre_ = planner_->getRobotModelPtr()->getPDetThresh();
        f_min_ = planner_->getRobotModelPtr()->getThrustLowerLimit();
        f_max_ = planner_->getRobotModelPtr()->getThrustUpperLimit();

        nhp_.param ("stability_margin_decrease_vel_thre", stability_margin_decrease_vel_thre_, -0.01);
        if(verbose_) std::cout << "stability_margin_decrease_vel_thre: " << std::setprecision(3) << stability_margin_decrease_vel_thre_ << std::endl;
        nhp_.param ("stability_margin_constraint_range", stability_margin_constraint_range_, 0.02);
        if(verbose_) std::cout << "stability_margin_constraint_range: " << std::setprecision(3) << stability_margin_constraint_range_ << std::endl;
        nhp_.param ("stability_margin_forbidden_range", stability_margin_forbidden_range_, 0.005);
        if(verbose_) std::cout << "stability_margin_forbidden_range: " << std::setprecision(3) << stability_margin_forbidden_range_ << std::endl;

        nhp_.param ("force_vel_thre", force_vel_thre_, 0.1);
        if(verbose_) std::cout << "force_vel_thre: " << std::setprecision(3) << force_vel_thre_ << std::endl;
        nhp_.param ("force_constraint_range", force_constraint_range_, 0.2);
        if(verbose_) std::cout << "force_constraint_range: " << std::setprecision(3) << force_constraint_range_ << std::endl;
        nhp_.param ("force_forbidden_range", force_forbidden_range_, 0.1);
        if(verbose_) std::cout << "force_forbidden_range: " << std::setprecision(3) << force_forbidden_range_ << std::endl;

      }

      bool getConstraint(Eigen::MatrixXd& A, Eigen::VectorXd& lb, Eigen::VectorXd& ub, bool debug = false)
      {
        auto robotModelUpdate = [this](KDL::Rotation root_att, KDL::JntArray actuator_vector)
          {
            planner_->getRobotModelPtr()->setCogDesireOrientation(root_att);
            planner_->getRobotModelPtr()->updateRobotModel(actuator_vector);
            planner_->getRobotModelPtr()->stabilityMarginCheck();
            planner_->getRobotModelPtr()->modelling();
          };

        KDL::Rotation curr_root_att;
        tf::quaternionTFToKDL(planner_->getTargetRootPose().getRotation(), curr_root_att);
        auto curr_actuator_vector =  planner_->getTargetActuatorVector<KDL::JntArray>();


        //debug = true;
        A = Eigen::MatrixXd::Zero(nc_, planner_->getRobotModelPtr()->getLinkJointIndex().size() + 6);
        lb = Eigen::VectorXd::Constant(nc_, -0.1);
        ub = Eigen::VectorXd::Constant(nc_, 0.1);

        /* 1. stability margin */
        double nominal_stability_margin = planner_->getRobotModelPtr()->getStabilityMargin();
        /* fill lb */
        lb(0) = stability_margin_decrease_vel_thre_;
        if(nominal_stability_margin - stability_margin_thre_  < stability_margin_constraint_range_)
          lb(0) *=  ((nominal_stability_margin - stability_margin_thre_ - stability_margin_forbidden_range_) / (stability_margin_constraint_range_ - stability_margin_forbidden_range_));

        /* 2. singularity */
        double nominal_p_det = planner_->getRobotModelPtr()->getPdeterminant();
        /* fill ub */
        lb(1) =  p_det_thre_ - nominal_p_det;

        /*************************************************************************************
        3. optimal hovering thrust constraint (including singularity check)
           f_min < F + delta_f < f_max =>   delta_f =  (f(q + d_q) - f(q)) / d_q * delta_q
           TODO: use virutal state (beta) to maximize the flight stability
        ****************************************************************************************/

        Eigen::VectorXd nominal_hovering_f =  planner_->getRobotModelPtr()->getOptimalHoveringThrust();
        /* fill the lb/ub */
        lb.segment(2, rotor_num_) = Eigen::VectorXd::Constant(rotor_num_, -force_vel_thre_);
        ub.segment(2, rotor_num_) = Eigen::VectorXd::Constant(rotor_num_, force_vel_thre_);
        for(int index = 0; index < rotor_num_; index++)
          {
            if(nominal_hovering_f(index) - f_min_ < force_constraint_range_)
              lb(2 + index) *= ((nominal_hovering_f(index) - f_min_ - force_forbidden_range_)/ (force_constraint_range_ - force_forbidden_range_));
            if(f_max_ - nominal_hovering_f(index)  < force_constraint_range_)
              ub(2 + index) *= ((f_max_ - nominal_hovering_f(index) - force_forbidden_range_)/ (force_constraint_range_ - force_forbidden_range_));
          }

        if(debug)
          {
            std::cout << "constraint name: " << constraint_name_ << ", nominal stability margin : \n" << nominal_stability_margin << std::endl;
            std::cout << "constraint name: " << constraint_name_ << ", nominal p det : \n" << nominal_p_det << std::endl;
            std::cout << "constraint name: " << constraint_name_ << ", nominal f: \n" << nominal_hovering_f.transpose() << std::endl;
          }

        /* update the result */
        if(result_p_det_min_ > nominal_p_det) result_p_det_min_ = nominal_p_det;
        if(result_stability_margin_min_ > nominal_stability_margin) result_stability_margin_min_ = nominal_stability_margin;
        if(result_f_min_ > nominal_hovering_f.minCoeff())
          result_f_min_ = nominal_hovering_f.minCoeff(&result_f_min_rotor_);
        if(result_f_max_ < nominal_hovering_f.maxCoeff())
          result_f_max_ = nominal_hovering_f.maxCoeff(&result_f_max_rotor_);

        /* joint */
        double delta_angle = 0.001; // [rad]
        for(int index = 0; index < planner_->getRobotModelPtr()->getLinkJointIndex().size(); index++)
          {
            KDL::JntArray perturbation_actuator_vector = curr_actuator_vector;
            perturbation_actuator_vector(planner_->getRobotModelPtr()->getLinkJointIndex().at(index)) += delta_angle;
            robotModelUpdate(curr_root_att, perturbation_actuator_vector);

            /* stability margin */
            A(0, 6 + index) = (planner_->getRobotModelPtr()->getStabilityMargin() - nominal_stability_margin) /delta_angle;
            /* singularity */
            A(1, 6 + index) = (planner_->getRobotModelPtr()->getPdeterminant() - nominal_p_det) /delta_angle;
            /* hovering thrust */
            A.block(2, 6 + index, rotor_num_, 1) = (planner_->getRobotModelPtr()->getOptimalHoveringThrust() - nominal_hovering_f) / delta_angle;
          }

        if(debug)
          std::cout << "constraint (" << constraint_name_.c_str()  << "): matrix A: \n" << A << std::endl;

        if(full_body_)
          {
            /* root */
            /* roll */
            robotModelUpdate(curr_root_att * KDL::Rotation::RPY(delta_angle, 0, 0), curr_actuator_vector);
            /* stability margin */
            A(0, 3) = (planner_->getRobotModelPtr()->getStabilityMargin() - nominal_stability_margin) / delta_angle;
            /* singularity */
            A(1, 3) = (planner_->getRobotModelPtr()->getPdeterminant() - nominal_p_det) / delta_angle;
            /* hovering thrust */
            A.block(2, 3, rotor_num_, 1) = (planner_->getRobotModelPtr()->getOptimalHoveringThrust() - nominal_hovering_f) / delta_angle;

            /*  pitch */
            robotModelUpdate(curr_root_att * KDL::Rotation::RPY(0, delta_angle, 0), curr_actuator_vector);
            /* stability margin */
            A(0, 4) = (planner_->getRobotModelPtr()->getStabilityMargin() - nominal_stability_margin) / delta_angle;
            /* singularity */
            A(1, 4) = (planner_->getRobotModelPtr()->getPdeterminant() - nominal_p_det) / delta_angle;
            /* hovering thrust */
            A.block(2, 4, rotor_num_, 1) = (planner_->getRobotModelPtr()->getOptimalHoveringThrust() - nominal_hovering_f) / delta_angle;

            /* yaw */
            robotModelUpdate(curr_root_att * KDL::Rotation::RPY(0, 0, delta_angle), curr_actuator_vector);

            /* stability margin */
            A(0, 5) = (planner_->getRobotModelPtr()->getStabilityMargin() - nominal_stability_margin) / delta_angle;
            /* singularity */
            A(1, 5) = (planner_->getRobotModelPtr()->getPdeterminant() - nominal_p_det) / delta_angle;
            /* hovering thrust */
            A.block(2, 5, rotor_num_, 1) = (planner_->getRobotModelPtr()->getOptimalHoveringThrust() - nominal_hovering_f) / delta_angle;
          }

        /* 0. revert the robot model with current state */
        robotModelUpdate(curr_root_att, curr_actuator_vector);

        if(debug)
          std::cout << "constraint (" << constraint_name_.c_str()  << "): matrix A: \n" << A << std::endl;
        if(debug)
          {
            std::cout << "constraint name: " << constraint_name_ << ", lb: \n" << lb.transpose() << std::endl;
            std::cout << "constraint name: " << constraint_name_ << ", ub: \n" << ub.transpose() << std::endl;
          }

        return true;
      }

      void result()
      {
        std::cout << constraint_name_
                  << "min p determinant: " << result_p_det_min_ << "\n"
                  << "min stability margin: " << result_stability_margin_min_ << "\n"
                  << "min f: " << result_f_min_ << " at rotor" << result_f_min_rotor_ << "\n"
                  << "max f: " << result_f_max_ << " at rotor" << result_f_max_rotor_ << std::endl;
      }

      bool directConstraint(){return false;}

    protected:
      double stability_margin_thre_;
      double p_det_thre_;
      double f_min_, f_max_;

      double force_vel_thre_;
      double force_constraint_range_;
      double force_forbidden_range_;

      double stability_margin_decrease_vel_thre_;
      double stability_margin_constraint_range_;
      double stability_margin_forbidden_range_;

      double result_f_min_;
      double result_f_max_;
      Eigen::MatrixXd::Index result_f_min_rotor_;
      Eigen::MatrixXd::Index result_f_max_rotor_;

      double result_p_det_min_;
      double result_stability_margin_min_;
    };
  };
};

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(differential_kinematics::constraint::Stability, differential_kinematics::constraint::Base);

