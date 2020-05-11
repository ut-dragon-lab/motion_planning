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
        wrench_mat_det_min_(1e6), old_control_margin_min_(1e6),
        wrench_margin_t_min_(1e6), wrench_margin_rp_min_(1e6)
      {}
      ~Stability(){}

      void virtual initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                              boost::shared_ptr<differential_kinematics::Planner> planner, std::string constraint_name,
                              bool orientation, bool full_body)
      {
        Base::initialize(nh, nhp, planner, constraint_name, orientation, full_body);

        nc_ = rotor_num_;

        const auto robot_model = boost::dynamic_pointer_cast<HydrusRobotModel>(planner_->getRobotModelPtr());

        getParam<bool>("check_wrench_margin_t", check_wrench_margin_t_, true);
        getParam<double>("wrench_margin_t_thre", wrench_margin_t_thre_, robot_model->getWrenchMarginTMinThre());
        getParam<double>("wrench_margin_t_decrease_vel_thre", wrench_margin_t_decrease_vel_thre_, -0.1);


        getParam<double>("wrench_margin_rp_thre", wrench_margin_rp_thre_, robot_model->getWrenchMarginRollPitchMinThre());
        getParam<double>("wrench_margin_rp_decrease_vel_thre", wrench_margin_rp_decrease_vel_thre_, -0.1);
        getParam<double>("wrench_margin_rp_constraint_range", wrench_margin_rp_constraint_range_, 0.2);
        getParam<double>("wrench_margin_rp_forbidden_range", wrench_margin_rp_forbidden_range_, 0.05);

        if(check_wrench_margin_t_) nc_ += rotor_num_;


        getParam<bool>("old_method", old_method_, false);
        getParam<double>("old_control_margin_thre", old_control_margin_thre_, robot_model->getOldControlMarginThresh());
        getParam<double>("wrench_mat_det_thre", wrench_mat_det_thre_, robot_model->getWrenchMatDetThresh());
        getParam<double>("old_control_margin_decrease_vel_thre", old_control_margin_decrease_vel_thre_, -0.01);
        getParam<double>("old_control_margin_constraint_range", old_control_margin_constraint_range_, 0.02);
        getParam<double>("old_control_margin_forbidden_range", old_control_margin_forbidden_range_, 0.005);

        if(old_method_) nc_ = 2;
      }

      bool getConstraint(Eigen::MatrixXd& A, Eigen::VectorXd& lb, Eigen::VectorXd& ub, bool debug = false)
      {
        //debug = true;
        const auto robot_model = boost::dynamic_pointer_cast<HydrusRobotModel>(planner_->getRobotModelPtr());

        A = Eigen::MatrixXd::Zero(nc_, robot_model->getLinkJointIndices().size() + 6);
        lb = Eigen::VectorXd::Constant(nc_, -0.1);
        ub = Eigen::VectorXd::Constant(nc_, 1e6);

        const auto& wrench_margin_rp_jacobian = robot_model->getWrenchMarginRollPitchJacobian();

        if(!old_method_)
          {
            Eigen::VectorXd wrench_margin_rp_vector = robot_model->getWrenchMarginRollPitchVector();
            for(int i = 0; i < rotor_num_; i++)
              {
                Eigen::MatrixXd::Index index;
                double rp_min = wrench_margin_rp_vector.minCoeff(&index);
                if(i == 0)
                  {
                    if(rp_min < wrench_margin_rp_min_)
                      {
                        wrench_margin_rp_min_ = rp_min;
                        wrench_margin_rp_min_old_control_margin_ = robot_model->getOldControlMargin();
                      }
                  }
                A.row(i) = wrench_margin_rp_jacobian.row(index);
                lb(i) = damplingBound(rp_min - wrench_margin_rp_thre_,
                                      wrench_margin_rp_decrease_vel_thre_,
                                      wrench_margin_rp_constraint_range_,
                                      wrench_margin_rp_forbidden_range_);

                wrench_margin_rp_vector(index) = 1e6; // reset
              }

            if(debug)
              {
                std::cout << "constraint (" << constraint_name_.c_str()  << "): wrench_margin_rp_vector: \n" << robot_model->getWrenchMarginRollPitchVector().transpose() << std::endl;
                std::cout << "constraint (" << constraint_name_.c_str()  << "): wrench_margin_rp_jacobian: \n" << wrench_margin_rp_jacobian << std::endl;
              }

            if(check_wrench_margin_t_)
              {
                Eigen::VectorXd wrench_margin_t_vector = robot_model->getWrenchMarginTVector();
                const auto& wrench_margin_t_jacobian = robot_model->getWrenchMarginTJacobian();

                for(int i = 0; i < rotor_num_; i++)
                  {
                    Eigen::MatrixXd::Index index;
                    double t_min = wrench_margin_t_vector.minCoeff(&index);
                    if(i == 0)
                      {
                        if(t_min < wrench_margin_t_min_)
                          {
                            wrench_margin_t_min_ = t_min;
                            wrench_margin_t_min_old_control_margin_ = robot_model->getOldControlMargin();
                          }
                      }
                    A.row(i + rotor_num_) = wrench_margin_t_jacobian.row(index);
                    double diff = wrench_margin_t_thre_ - t_min;
                    lb(i + rotor_num_) =  diff < wrench_margin_t_decrease_vel_thre_? wrench_margin_t_decrease_vel_thre_:diff;
                    wrench_margin_t_vector(index) = 1e6; // reset
                  }

                if(debug)
                  {
                    std::cout << "constraint (" << constraint_name_.c_str()  << "): wrench_margin_t_i: \n" << robot_model->getWrenchMarginTVector().transpose() << std::endl;
                    std::cout << "constraint (" << constraint_name_.c_str()  << "): wrench_margin_t_jacobian: \n" << wrench_margin_t_jacobian << std::endl;
                  }
              }
          }
        else
          {
            numericalUpdate(robot_model, A, lb, ub);
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
        std::cout << constraint_name_
                  << "min wrench_margin_rp_min_: " << wrench_margin_rp_min_
                  << "; wrench_margin_rp_min_old_control_margin_: " << wrench_margin_rp_min_old_control_margin_  << std::endl;
        if(check_wrench_margin_t_)
          {
            std::cout << constraint_name_
                      << "min wrench_margin_t_min_: " << wrench_margin_t_min_
                      << "; wrench_margin_t_min_old_control_margin_: " << wrench_margin_t_min_old_control_margin_  << std::endl;
          }

        if(old_method_)
          {
            std::cout << constraint_name_
                      << "min wremch matrix determinant: " << wrench_mat_det_min_ << "\n"
                      << "min control margin: " << old_control_margin_min_
                      << "; old_control_margin_min_rp_" << old_control_margin_min_rp_ << std::endl;
          }
      }

      bool directConstraint(){return false;}

    protected:

      bool check_wrench_margin_t_;
      double wrench_margin_t_thre_;
      double wrench_margin_t_decrease_vel_thre_;
      double wrench_margin_rp_thre_;
      double wrench_margin_rp_decrease_vel_thre_;
      double wrench_margin_rp_constraint_range_;
      double wrench_margin_rp_forbidden_range_;

      double wrench_margin_t_min_;
      double wrench_margin_rp_min_;
      double wrench_margin_t_min_old_control_margin_;
      double wrench_margin_rp_min_old_control_margin_;

      bool old_method_;
      double old_control_margin_thre_;
      double wrench_mat_det_thre_;

      double old_control_margin_decrease_vel_thre_;
      double old_control_margin_constraint_range_;
      double old_control_margin_forbidden_range_;

      double wrench_mat_det_min_;
      double old_control_margin_min_;
      double old_control_margin_min_rp_;

    public:

      void numericalUpdate(boost::shared_ptr<aerial_robot_model::RobotModel> robot_model, Eigen::MatrixXd& A, Eigen::VectorXd& lb, Eigen::VectorXd& ub)
      {
          const auto& joint_indices = robot_model->getLinkJointIndices();
          const auto hydrus_robot_model = boost::dynamic_pointer_cast<HydrusRobotModel>(robot_model);
          auto curr_root_att = planner_->getTargetRootPose<KDL::Frame>().M;
          auto curr_joint_vector =  planner_->getTargetJointVector<KDL::JntArray>();

          /* 1. stability margin */
          double nominal_old_control_margin = hydrus_robot_model->getOldControlMargin();
          /* fill lb */
          lb(0) = damplingBound(nominal_old_control_margin - old_control_margin_thre_, old_control_margin_decrease_vel_thre_, old_control_margin_constraint_range_, old_control_margin_forbidden_range_);

          /* 2. singularity */
          double nominal_wrench_mat_det = hydrus_robot_model->getWrenchMatDeterminant();
          /* fill ub */
          lb(1) =  wrench_mat_det_thre_ - nominal_wrench_mat_det;

          /* update the result */
          double wrench_margin_t_min = hydrus_robot_model->getWrenchMarginTMin();
          double wrench_margin_rp_min = hydrus_robot_model->getWrenchMarginRollPitchMin();

          if(wrench_mat_det_min_ > nominal_wrench_mat_det) wrench_mat_det_min_ = nominal_wrench_mat_det;
          if(old_control_margin_min_ > nominal_old_control_margin)
            {
              old_control_margin_min_ = nominal_old_control_margin;
              old_control_margin_min_rp_ = wrench_margin_rp_min;
            }

          if(wrench_margin_t_min < wrench_margin_t_min_)
            {
              wrench_margin_t_min_ = wrench_margin_t_min;
              wrench_margin_t_min_old_control_margin_ = nominal_old_control_margin;
              ROS_ERROR("max wrench_margin_rp_min: %f", wrench_margin_rp_min);
            }
          if(wrench_margin_rp_min < wrench_margin_rp_min_)
            {
              wrench_margin_rp_min_ = wrench_margin_rp_min;
              wrench_margin_rp_min_old_control_margin_ = nominal_old_control_margin;
            }


          // numerical solution for control margin and wrench mat determinant
          double delta_angle = 0.0001; // [rad]
          auto perturbate = [&](int col, KDL::Rotation root_att, KDL::JntArray joint_vector)
            {
              hydrus_robot_model->setCogDesireOrientation(root_att);
              hydrus_robot_model->updateRobotModel(joint_vector);
              hydrus_robot_model->rollPitchPositionMarginCheck();
              hydrus_robot_model->wrenchMatrixDeterminantCheck();

              A(0, col) = (hydrus_robot_model->getOldControlMargin() - nominal_old_control_margin) /delta_angle;  // control margin
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

