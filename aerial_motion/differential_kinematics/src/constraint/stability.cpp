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
        wrench_mat_det_min_(1e6), control_margin_min_(1e6),
        wrench_margin_t_min_(1e6), wrench_margin_roll_pitch_min_(1e6)
      {}
      ~Stability(){}

      void virtual initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                              boost::shared_ptr<differential_kinematics::Planner> planner, std::string constraint_name,
                              bool orientation, bool full_body)
      {
        Base::initialize(nh, nhp, planner, constraint_name, orientation, full_body);

        nc_ = 2;
        const auto robot_model = boost::dynamic_pointer_cast<HydrusRobotModel>(planner_->getRobotModelPtr());
        getParam<double>("control_margin_thre", control_margin_thre_, robot_model->getControlMarginThresh());
        getParam<double>("wrench_mat_det_thre", wrench_mat_det_thre_, robot_model->getWrenchMatDetThresh());
        getParam<double>("control_margin_decrease_vel_thre", control_margin_decrease_vel_thre_, -0.01);
        getParam<double>("control_margin_constraint_range", control_margin_constraint_range_, 0.02);
        getParam<double>("control_margin_forbidden_range", control_margin_forbidden_range_, 0.005);
      }

      bool getConstraint(Eigen::MatrixXd& A, Eigen::VectorXd& lb, Eigen::VectorXd& ub, bool debug = false)
      {
        //debug = true;
        auto curr_root_att = planner_->getTargetRootPose<KDL::Frame>().M;
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
                  << "min wremch matrix determinant: " << wrench_mat_det_min_ << "\n"
                  << "min control margin: " << control_margin_min_
                  << "; control_margin_min_roll_pitch_min_" << control_margin_min_roll_pitch_min_ << std::endl;
        std::cout << constraint_name_
                  << "min wrench_margin_t_min_: " << wrench_margin_t_min_
                  << "; wrench_margin_t_min_control_margin_min_: " << wrench_margin_t_min_control_margin_min_  << std::endl;
        std::cout << constraint_name_
                  << "min wrench_margin_roll_pitch_min_: " << wrench_margin_roll_pitch_min_
                  << "; wrench_margin_roll_pitch_min_control_margin_min_: " << wrench_margin_roll_pitch_min_control_margin_min_  << std::endl;
      }

      bool directConstraint(){return false;}

    protected:
      double control_margin_thre_;
      double wrench_mat_det_thre_;

      double control_margin_decrease_vel_thre_;
      double control_margin_constraint_range_;
      double control_margin_forbidden_range_;

      double wrench_mat_det_min_;
      double control_margin_min_;
      double control_margin_min_roll_pitch_min_;

      double wrench_margin_t_min_;
      double wrench_margin_roll_pitch_min_;
      double wrench_margin_t_min_control_margin_min_;
      double wrench_margin_roll_pitch_min_control_margin_min_;
    public:

      void numericalUpdate(boost::shared_ptr<aerial_robot_model::RobotModel> robot_model, const KDL::Rotation& curr_root_att, const KDL::JntArray& curr_joint_vector, Eigen::MatrixXd& A, Eigen::VectorXd& lb, Eigen::VectorXd& ub, bool debug = false)
      {
          const auto& joint_indices = robot_model->getLinkJointIndices();
          const auto hydrus_robot_model = boost::dynamic_pointer_cast<HydrusRobotModel>(robot_model);

          /* 1. stability margin */
          double nominal_control_margin = hydrus_robot_model->getControlMargin();
          /* fill lb */
          lb(0) = damplingBound(nominal_control_margin - control_margin_thre_, control_margin_decrease_vel_thre_, control_margin_constraint_range_, control_margin_forbidden_range_);

          /* 2. singularity */
          double nominal_wrench_mat_det = hydrus_robot_model->getWrenchMatDeterminant();
          /* fill ub */
          lb(1) =  wrench_mat_det_thre_ - nominal_wrench_mat_det;

          /* update the result */
          double wrench_margin_t_min = hydrus_robot_model->getWrenchMarginTMin();
          double wrench_margin_roll_pitch_min = hydrus_robot_model->getWrenchMarginRollPitchMin();

          if(wrench_mat_det_min_ > nominal_wrench_mat_det) wrench_mat_det_min_ = nominal_wrench_mat_det;
          if(control_margin_min_ > nominal_control_margin)
            {
              control_margin_min_ = nominal_control_margin;
              control_margin_min_roll_pitch_min_ = wrench_margin_roll_pitch_min;
            }

          if(wrench_margin_t_min < wrench_margin_t_min_)
            {
              wrench_margin_t_min_ = wrench_margin_t_min;
              wrench_margin_t_min_control_margin_min_ = nominal_control_margin;
              ROS_ERROR("max wrench_margin_roll_pitch_min: %f", wrench_margin_roll_pitch_min);
            }
          if(wrench_margin_roll_pitch_min < wrench_margin_roll_pitch_min_)
            {
              wrench_margin_roll_pitch_min_ = wrench_margin_roll_pitch_min;
              wrench_margin_roll_pitch_min_control_margin_min_ = nominal_control_margin;
            }


          // numerical solution for control margin and wrench mat determinant
          double delta_angle = 0.0001; // [rad]
          auto perturbate = [&](int col, KDL::Rotation root_att, KDL::JntArray joint_vector)
            {
              hydrus_robot_model->setCogDesireOrientation(root_att);
              hydrus_robot_model->updateRobotModel(joint_vector);
              hydrus_robot_model->rollPitchPositionMarginCheck();
              hydrus_robot_model->wrenchMatrixDeterminantCheck();

              A(0, col) = (hydrus_robot_model->getControlMargin() - nominal_control_margin) /delta_angle;  // control margin
              A(1, col) = (hydrus_robot_model->getWrenchMatDeterminant() - nominal_wrench_mat_det) /delta_angle; // singularity
            };

          /* joint */
          int col_index = 6;
          for (const auto& joint_index : joint_indices)
            {
              KDL::JntArray perturbation_joint_vector = curr_joint_vector;
              perturbation_joint_vector(joint_index) += delta_angle;
              perturbate(col_index, curr_root_att, perturbation_joint_vector);
              col_index++;
            }

          if(full_body_)
            {
              /* root */
              /* roll */
              perturbate(3, curr_root_att * KDL::Rotation::RPY(delta_angle, 0, 0), curr_joint_vector);
              /* pitch */
              perturbate(4, curr_root_att * KDL::Rotation::RPY(0, delta_angle, 0), curr_joint_vector);

              /* yaw */
              perturbate(5, curr_root_att * KDL::Rotation::RPY(0, 0, delta_angle), curr_joint_vector);
            }

          /* 0. revert the robot model with current state */
          hydrus_robot_model->setCogDesireOrientation(curr_root_att);
          hydrus_robot_model->updateRobotModel(curr_joint_vector);
        }
    };
  };
};

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(differential_kinematics::constraint::Stability, differential_kinematics::constraint::Base);

