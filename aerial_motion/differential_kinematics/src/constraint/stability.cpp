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
#include <hydrus/hydrus_robot_model.h>

namespace differential_kinematics
{
  namespace constraint
  {
    class Stability :public Base
    {
    public:
      Stability():
        wrench_mat_det_min_(1e6), rp_position_margin_min_(1e6),
        fc_t_min_(1e6), fc_rp_min_(1e6)
      {}
      ~Stability(){}

      void virtual initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                              boost::shared_ptr<differential_kinematics::Planner> planner, std::string constraint_name,
                              bool orientation, bool full_body)
      {
        Base::initialize(nh, nhp, planner, constraint_name, orientation, full_body);

        nc_ = rotor_num_;

        const auto robot_model = boost::dynamic_pointer_cast<HydrusRobotModel>(planner_->getRobotModelPtr());

        getParam<bool>("check_fc_t", check_fc_t_, true);
        getParam<double>("fc_t_min_thre", fc_t_min_thre_, robot_model->getFeasibleControlTMinThre());
        getParam<double>("fc_t_dist_decrease_vel_thre", fc_t_dist_decrease_vel_thre_, -0.1);

        getParam<double>("fc_rp_min_thre", fc_rp_min_thre_, robot_model->getFeasibleControlRollPitchMinThre());
        getParam<double>("fc_rp_dist_decrease_vel_thre", fc_rp_dist_decrease_vel_thre_, -0.1);
        getParam<double>("fc_rp_dist_constraint_range", fc_rp_dist_constraint_range_, 0.2);
        getParam<double>("fc_rp_dist_forbidden_range", fc_rp_dist_forbidden_range_, 0.05);

        if(check_fc_t_) nc_ += rotor_num_;

        getParam<bool>("old_method", old_method_, false);
        getParam<double>("rp_position_margin_thre", rp_position_margin_thre_, robot_model->getRollPitchPositionMarginThresh());
        getParam<double>("wrench_mat_det_thre", wrench_mat_det_thre_, robot_model->getWrenchMatDetThresh());
        getParam<double>("rp_position_margin_decrease_vel_thre", rp_position_margin_decrease_vel_thre_, -0.01);
        getParam<double>("rp_position_margin_constraint_range", rp_position_margin_constraint_range_, 0.02);
        getParam<double>("rp_position_margin_forbidden_range", rp_position_margin_forbidden_range_, 0.005);

        if(old_method_) nc_ = 2;
      }

      bool getConstraint(Eigen::MatrixXd& A, Eigen::VectorXd& lb, Eigen::VectorXd& ub, bool debug = false)
      {
        const auto robot_model = boost::dynamic_pointer_cast<HydrusRobotModel>(planner_->getRobotModelPtr());

        A = Eigen::MatrixXd::Zero(nc_, robot_model->getLinkJointIndices().size() + 6);
        lb = Eigen::VectorXd::Constant(nc_, -0.1);
        ub = Eigen::VectorXd::Constant(nc_, 1e6);

        if(!old_method_)
          {
            const auto& fc_rp_dists_jacobian = robot_model->getFeasibleControlRollPitchDistsJacobian();
            Eigen::VectorXd fc_rp_dists = robot_model->getFeasibleControlRollPitchDists();
        
            for(int i = 0; i < rotor_num_; i++)
              {
             
                Eigen::MatrixXd::Index index;
                double rp_min = fc_rp_dists.minCoeff(&index);
                if(i == 0)
                  {
                   
                    if(rp_min < fc_rp_min_)
                      {
                        fc_rp_min_ = rp_min;
                        fc_rp_min_rp_position_margin_ = robot_model->getRollPitchPositionMargin();
                      }
                  }
                A.row(i) = fc_rp_dists_jacobian.row(index);
                 
                lb(i) = damplingBound(rp_min - fc_rp_min_thre_,
                                      fc_rp_dist_decrease_vel_thre_,
                                      fc_rp_dist_constraint_range_,
                                      fc_rp_dist_forbidden_range_);
                fc_rp_dists(index) = 1e6; // reset
              }

            if(debug)
              {
                std::cout << "constraint (" << constraint_name_.c_str()  << "): feasible control roll pitch convex distances: \n" << robot_model->getFeasibleControlRollPitchDists().transpose() << std::endl;
                std::cout << "constraint (" << constraint_name_.c_str()  << "): fc_rp_jacobian: \n" << fc_rp_dists_jacobian << std::endl;
              }

            if(check_fc_t_)
              {
                Eigen::VectorXd fc_t_dists = robot_model->getFeasibleControlTDists();
                const auto& fc_t_dists_jacobian = robot_model->getFeasibleControlTDistsJacobian();

                // only choose rotor_num component
                for(int i = 0; i < rotor_num_; i++)
                  {
                    Eigen::MatrixXd::Index index;
                    double t_min = fc_t_dists.minCoeff(&index);
                    if(i == 0)
                      {
                        if(t_min < fc_t_min_)
                          {
                            fc_t_min_ = t_min;
                            fc_t_min_rp_position_margin_ = robot_model->getRollPitchPositionMargin();
                          }
                      }
                    A.row(i + rotor_num_) = fc_t_dists_jacobian.row(index);
                    double diff = fc_t_min_thre_ - t_min;
                    lb(i + rotor_num_) =  diff < fc_t_dist_decrease_vel_thre_? fc_t_dist_decrease_vel_thre_:diff;
                    fc_t_dists(index) = 1e6; // reset
                  }
                if(debug)
                  {
                    std::cout << "constraint (" << constraint_name_.c_str()  << "): feasible control torque convex distances: \n" << robot_model->getFeasibleControlTDists().transpose() << std::endl;
                    std::cout << "constraint (" << constraint_name_.c_str()  << "): fc_t_dists_jacobian: \n" << fc_t_dists_jacobian << std::endl;
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
                  << "min fc_rp_min_: " << fc_rp_min_
                  << "; fc_rp_min_rp_position_margin_: " << fc_rp_min_rp_position_margin_  << std::endl;
        if(check_fc_t_)
          {
            std::cout << constraint_name_
                      << "min fc_t_min_: " << fc_t_min_
                      << "; fc_t_min_rp_position_margin_: " << fc_t_min_rp_position_margin_  << std::endl;
          }

        if(old_method_)
          {
            std::cout << constraint_name_
                      << "min wremch matrix determinant: " << wrench_mat_det_min_ << "\n"
                      << "min control margin: " << rp_position_margin_min_
                      << "; rp_position_margin_min_rp_" << rp_position_margin_min_rp_ << std::endl;
          }
      }

      bool directConstraint(){return false;}

    protected:

      bool check_fc_t_;
      double fc_t_min_thre_;
      double fc_t_dist_decrease_vel_thre_;
      double fc_rp_min_thre_;
      double fc_rp_dist_decrease_vel_thre_;
      double fc_rp_dist_constraint_range_;
      double fc_rp_dist_forbidden_range_;

      double fc_t_min_;
      double fc_rp_min_;
      double fc_t_min_rp_position_margin_;
      double fc_rp_min_rp_position_margin_;

      bool old_method_;
      double rp_position_margin_thre_;
      double wrench_mat_det_thre_;

      double rp_position_margin_decrease_vel_thre_;
      double rp_position_margin_constraint_range_;
      double rp_position_margin_forbidden_range_;

      double wrench_mat_det_min_;
      double rp_position_margin_min_;
      double rp_position_margin_min_rp_;

    public:

      void numericalUpdate(boost::shared_ptr<aerial_robot_model::transformable::RobotModel> robot_model, Eigen::MatrixXd& A, Eigen::VectorXd& lb, Eigen::VectorXd& ub)
      {
          const auto& joint_indices = robot_model->getLinkJointIndices();
          const auto hydrus_robot_model = boost::dynamic_pointer_cast<HydrusRobotModel>(robot_model);
          auto curr_root_att = planner_->getTargetRootPose<KDL::Frame>().M;
          auto curr_joint_vector =  planner_->getTargetJointVector<KDL::JntArray>();

          /* 1. stability margin */
          double nominal_rp_position_margin = hydrus_robot_model->getRollPitchPositionMargin();
          /* fill lb */
          lb(0) = damplingBound(nominal_rp_position_margin - rp_position_margin_thre_, rp_position_margin_decrease_vel_thre_, rp_position_margin_constraint_range_, rp_position_margin_forbidden_range_);

          /* 2. singularity */
          double nominal_wrench_mat_det = hydrus_robot_model->getWrenchMatDeterminant();
          /* fill ub */
          lb(1) =  wrench_mat_det_thre_ - nominal_wrench_mat_det;

          /* update the result */
          double fc_t_min = hydrus_robot_model->getFeasibleControlTMin();
          double fc_rp_min = hydrus_robot_model->getFeasibleControlRollPitchMin();

          if(wrench_mat_det_min_ > nominal_wrench_mat_det) wrench_mat_det_min_ = nominal_wrench_mat_det;
          if(rp_position_margin_min_ > nominal_rp_position_margin)
            {
              rp_position_margin_min_ = nominal_rp_position_margin;
              rp_position_margin_min_rp_ = fc_rp_min;
            }

          if(fc_t_min < fc_t_min_)
            {
              fc_t_min_ = fc_t_min;
              fc_t_min_rp_position_margin_ = nominal_rp_position_margin;
            }
          if(fc_rp_min < fc_rp_min_)
            {
              fc_rp_min_ = fc_rp_min;
              fc_rp_min_rp_position_margin_ = nominal_rp_position_margin;
            }


          // numerical solution for control margin and wrench mat determinant
          double delta_angle = 0.0001; // [rad]
          auto perturbate = [&](int col, KDL::Rotation root_att, KDL::JntArray joint_vector)
            {
              hydrus_robot_model->setCogDesireOrientation(root_att);
              hydrus_robot_model->updateRobotModel(joint_vector);
              hydrus_robot_model->rollPitchPositionMarginCheck();
              hydrus_robot_model->wrenchMatrixDeterminantCheck();

              A(0, col) = (hydrus_robot_model->getRollPitchPositionMargin() - nominal_rp_position_margin) /delta_angle;  // control margin
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

