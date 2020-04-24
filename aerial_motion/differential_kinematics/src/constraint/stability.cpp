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
        f_min_(1e6), f_max_(0),
        p_det_min_(1e6), control_margin_min_(1e6)
      {}
      ~Stability(){}

      void virtual initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                              boost::shared_ptr<differential_kinematics::Planner> planner, std::string constraint_name,
                              bool orientation, bool full_body)
      {
        Base::initialize(nh, nhp, planner, constraint_name, orientation, full_body);

        nc_ = 2 + rotor_num_;
        const auto hydrus_robot_model = boost::dynamic_pointer_cast<HydrusRobotModel>(planner_->getRobotModelPtr());
        getParam<double>("f_min_thre", f_min_thre_, hydrus_robot_model->getThrustLowerLimit());
        getParam<double>("f_max_thre", f_max_thre_, hydrus_robot_model->getThrustUpperLimit());
        getParam<double>("control_margin_thre", control_margin_thre_, hydrus_robot_model->getControlMarginThresh());
        getParam<double>("p_det_thre", p_det_thre_, hydrus_robot_model->getPDetThresh());
        getParam<double>("control_margin_decrease_vel_thre", control_margin_decrease_vel_thre_, -0.01);
        getParam<double>("control_margin_constraint_range", control_margin_constraint_range_, 0.02);
        getParam<double>("control_margin_forbidden_range", control_margin_forbidden_range_, 0.005);
        getParam<double>("force_vel_thre", force_vel_thre_, 0.1);
        getParam<double>("force_constraint_range", force_constraint_range_, 0.2);
        getParam<double>("force_forbidden_range", force_forbidden_range_, 0.1);
      }

      bool getConstraint(Eigen::MatrixXd& A, Eigen::VectorXd& lb, Eigen::VectorXd& ub, bool debug = false)
      {
        //debug = true;

        KDL::Rotation curr_root_att;
        tf::quaternionTFToKDL(planner_->getTargetRootPose().getRotation(), curr_root_att);
        auto curr_joint_vector =  planner_->getTargetJointVector<KDL::JntArray>();
        auto robot_model = planner_->getRobotModelPtr();

        A = Eigen::MatrixXd::Zero(nc_, robot_model->getLinkJointIndices().size() + 6);
        lb = Eigen::VectorXd::Constant(nc_, -0.1);
        ub = Eigen::VectorXd::Constant(nc_, 0.1);

        numericalUpdate(robot_model, curr_root_att, curr_joint_vector, A, lb, ub, debug);

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
        std::cout << constraint_name_
                  << "min p determinant: " << p_det_min_ << "\n"
                  << "min control margin: " << control_margin_min_ << "\n"
                  << "min f: " << f_min_ << " at rotor" << f_min_rotor_ << "\n"
                  << "max f: " << f_max_ << " at rotor" << f_max_rotor_ << std::endl;
      }

      bool directConstraint(){return false;}

    protected:
      double control_margin_thre_;
      double p_det_thre_;
      double f_min_thre_, f_max_thre_;

      double force_vel_thre_;
      double force_constraint_range_;
      double force_forbidden_range_;

      double control_margin_decrease_vel_thre_;
      double control_margin_constraint_range_;
      double control_margin_forbidden_range_;

      double f_min_;
      double f_max_;
      Eigen::MatrixXd::Index f_min_rotor_;
      Eigen::MatrixXd::Index f_max_rotor_;
      double p_det_min_;
      double control_margin_min_;

    public:

      void numericalUpdate(boost::shared_ptr<aerial_robot_model::RobotModel> robot_model, const KDL::Rotation& curr_root_att, const KDL::JntArray& curr_joint_vector, Eigen::MatrixXd& A, Eigen::VectorXd& lb, Eigen::VectorXd& ub, bool debug = false)
      {
          const auto& joint_indices = robot_model->getLinkJointIndices();
          const auto hydrus_robot_model = boost::dynamic_pointer_cast<HydrusRobotModel>(robot_model);

          auto robotModelUpdate = [&hydrus_robot_model](KDL::Rotation root_att, KDL::JntArray joint_vector)
            {
              hydrus_robot_model->setCogDesireOrientation(root_att);
              hydrus_robot_model->updateRobotModel(joint_vector);
              hydrus_robot_model->updateStatics();
              hydrus_robot_model->stabilityCheck();
            };

          /* 1. stability margin */
          double nominal_control_margin = hydrus_robot_model->getControlMargin();
          /* fill lb */
#if 0
          lb(0) = control_margin_decrease_vel_thre_;
          if(nominal_control_margin - control_margin_thre_  < control_margin_constraint_range_)
            lb(0) *=  ((nominal_control_margin - control_margin_thre_ - control_margin_forbidden_range_) / (control_margin_constraint_range_ - control_margin_forbidden_range_));
#endif
          lb(0) = damplingBound(nominal_control_margin - control_margin_thre_, control_margin_decrease_vel_thre_, control_margin_constraint_range_, control_margin_forbidden_range_);

          /* 2. singularity */
          double nominal_p_det = hydrus_robot_model->getPdeterminant();
          /* fill ub */
          lb(1) =  p_det_thre_ - nominal_p_det;

          /*
           3. optimal hovering thrust constraint (including singularity check)
           f_min < F + delta_f < f_max =>   delta_f =  (f(q + d_q) - f(q)) / d_q * delta_q
          */
          const Eigen::VectorXd nominal_hovering_f =  hydrus_robot_model->getOptimalHoveringThrust();
          /* fill the lb/ub */
          lb.segment(2, rotor_num_) = Eigen::VectorXd::Constant(rotor_num_, -force_vel_thre_);
          ub.segment(2, rotor_num_) = Eigen::VectorXd::Constant(rotor_num_, force_vel_thre_);
          for(int index = 0; index < rotor_num_; index++)
            {
              lb(2 + index) = damplingBound(nominal_hovering_f(index) - f_min_thre_, -force_vel_thre_,  force_constraint_range_,  force_forbidden_range_);
              ub(2 + index) = damplingBound(f_max_thre_ - nominal_hovering_f(index), force_vel_thre_,  force_constraint_range_,  force_forbidden_range_);
            }

          if(debug)
            {
              std::cout << "constraint name: " << constraint_name_ << ", nominal control margin : \n" << nominal_control_margin << std::endl;
              std::cout << "constraint name: " << constraint_name_ << ", nominal p det : \n" << nominal_p_det << std::endl;
              std::cout << "constraint name: " << constraint_name_ << ", nominal f: \n" << nominal_hovering_f.transpose() << std::endl;
            }

          /* update the result */
          if(p_det_min_ > nominal_p_det) p_det_min_ = nominal_p_det;
          if(control_margin_min_ > nominal_control_margin) control_margin_min_ = nominal_control_margin;
          if(f_min_ > nominal_hovering_f.minCoeff())
            f_min_ = nominal_hovering_f.minCoeff(&f_min_rotor_);
          if(f_max_ < nominal_hovering_f.maxCoeff())
            f_max_ = nominal_hovering_f.maxCoeff(&f_max_rotor_);

          /* joint */
          double delta_angle = 0.001; // [rad]
          for(int index = 0; index < joint_indices.size(); index++)
            {
              KDL::JntArray perturbation_joint_vector = curr_joint_vector;
              perturbation_joint_vector(joint_indices.at(index)) += delta_angle;
              robotModelUpdate(curr_root_att, perturbation_joint_vector);

              /* control margin */
              A(0, 6 + index) = (hydrus_robot_model->getControlMargin() - nominal_control_margin) /delta_angle;
              /* singularity */
              A(1, 6 + index) = (hydrus_robot_model->getPdeterminant() - nominal_p_det) /delta_angle;
              /* hovering thrust */
              A.block(2, 6 + index, rotor_num_, 1) = (hydrus_robot_model->getOptimalHoveringThrust() - nominal_hovering_f) / delta_angle;
            }

          if(debug)
            std::cout << "constraint (" << constraint_name_.c_str()  << "): matrix A: \n" << A << std::endl;

          if(full_body_)
            {
              /* root */
              /* roll */
              robotModelUpdate(curr_root_att * KDL::Rotation::RPY(delta_angle, 0, 0), curr_joint_vector);
              /* control margin */
              A(0, 3) = (hydrus_robot_model->getControlMargin() - nominal_control_margin) / delta_angle;
              /* singularity */
              A(1, 3) = (hydrus_robot_model->getPdeterminant() - nominal_p_det) / delta_angle;
              /* hovering thrust */
              A.block(2, 3, rotor_num_, 1) = (hydrus_robot_model->getOptimalHoveringThrust() - nominal_hovering_f) / delta_angle;

              /*  pitch */
              robotModelUpdate(curr_root_att * KDL::Rotation::RPY(0, delta_angle, 0), curr_joint_vector);
              /* control margin */
              A(0, 4) = (hydrus_robot_model->getControlMargin() - nominal_control_margin) / delta_angle;
              /* singularity */
              A(1, 4) = (hydrus_robot_model->getPdeterminant() - nominal_p_det) / delta_angle;
              /* hovering thrust */
              A.block(2, 4, rotor_num_, 1) = (hydrus_robot_model->getOptimalHoveringThrust() - nominal_hovering_f) / delta_angle;

              /* yaw */
              robotModelUpdate(curr_root_att * KDL::Rotation::RPY(0, 0, delta_angle), curr_joint_vector);

              /* control margin */
              A(0, 5) = (hydrus_robot_model->getControlMargin() - nominal_control_margin) / delta_angle;
              /* singularity */
              A(1, 5) = (hydrus_robot_model->getPdeterminant() - nominal_p_det) / delta_angle;
              /* hovering thrust */
              A.block(2, 5, rotor_num_, 1) = (hydrus_robot_model->getOptimalHoveringThrust() - nominal_hovering_f) / delta_angle;
            }

          /* 0. revert the robot model with current state */
          robotModelUpdate(curr_root_att, curr_joint_vector);
        }
    };
  };
};

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(differential_kinematics::constraint::Stability, differential_kinematics::constraint::Base);

