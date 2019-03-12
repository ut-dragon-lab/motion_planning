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

/* specialized for DRAGON model */

#include <differential_kinematics/constraint/base_plugin.h>

/* dragon model */
#include <dragon/transform_control.h>


namespace differential_kinematics
{
  namespace constraint
  {
    class Overlap :public Base
    {
    public:
      Overlap(): result_overlap_min_(1e6)
      {}
      ~Overlap(){}

      void virtual initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                              boost::shared_ptr<differential_kinematics::Planner> planner,
                              std::string constraint_name,
                              bool orientation, bool full_body)
      {
        Base::initialize(nh, nhp, planner, constraint_name, orientation, full_body);

        nc_ = 1; // temporary, using one dimension

        nhp_.param ("overlap_change_vel_thre", overlap_change_vel_thre_, 0.05);
        if(verbose_) std::cout << "overlap_change_vel_thre: " << std::setprecision(3) << overlap_change_vel_thre_ << std::endl;
        nhp_.param ("overlap_constraint_range", overlap_constraint_range_, 0.05); // [m]
        if(verbose_) std::cout << "overlap_constraint_range: " << std::setprecision(3) << overlap_constraint_range_ << std::endl;
        nhp_.param ("overlap_forbidden_range", overlap_forbidden_range_, 0.01); // [m]
        if(verbose_) std::cout << "overlap_forbidden_range: " << std::setprecision(3) << overlap_forbidden_range_ << std::endl;

      }

      bool getConstraint(Eigen::MatrixXd& A, Eigen::VectorXd& lb, Eigen::VectorXd& ub, bool debug = false)
      {
        /* TODO: not general */
        auto dragon_model_ptr = boost::dynamic_pointer_cast<DragonRobotModel>(planner_->getRobotModelPtr());

        auto robotModelUpdate = [this, &dragon_model_ptr](KDL::Rotation root_att, KDL::JntArray actuator_vector)
          {
            dragon_model_ptr->setCogDesireOrientation(root_att);
            dragon_model_ptr->updateRobotModel(actuator_vector);
            return dragon_model_ptr->getEdfsOriginFromCog<Eigen::Vector3d>();
          };

        auto distCalc = [this, &dragon_model_ptr](std::vector<Eigen::Vector3d> edfs_origin_from_cog,
                           int link_i, int link_j)
          {
            Eigen::Vector3d diff3d = edfs_origin_from_cog[link_i] - edfs_origin_from_cog[link_j]; //dual
            double projected_dist2d = sqrt(diff3d(0) * diff3d(0) + diff3d(1) * diff3d(1));
            /* Note: approximated, the true one should be (edf_radius_ / cos(tilt) + diff(2) * tan(tilt) */
            double dist2d_thre = 2 * dragon_model_ptr->getEdfRadius()  + fabs(diff3d(2)) * tan(dragon_model_ptr->getEdfMaxTilt());

            return projected_dist2d - dist2d_thre;
          };

        A = Eigen::MatrixXd::Zero(nc_, planner_->getRobotModelPtr()->getLinkJointIndex().size() + 6);
        lb = Eigen::VectorXd::Constant(nc_, -overlap_change_vel_thre_);
        ub = Eigen::VectorXd::Constant(nc_, 1e6);

        KDL::Rotation curr_root_att;
        tf::quaternionTFToKDL(planner_->getTargetRootPose().getRotation(), curr_root_att);
        auto curr_actuator_vector =  planner_->getTargetActuatorVector<KDL::JntArray>();
        auto edfs_origin_from_cog = dragon_model_ptr->getEdfsOriginFromCog<Eigen::Vector3d>();

        /* Note: we only choose the minimum diff, because it is not feasible for N-multilink */
        double min_dist = 1e6;
        int overlap_link1, overlap_link2;
        for(int i = 0; i < rotor_num_ * 2; i++)
          {
            for(int j =  i + 1; j < rotor_num_ * 2; j++)
              {
                /* special for dual rotor */
                if(i / 2 == j / 2) continue;

                auto dist = distCalc(edfs_origin_from_cog, i, j);

                if(dist < 0)
                  {
                    if(verbose_)ROS_ERROR("constraint: %s, overlap!: %d and %d, dist: %f", constraint_name_.c_str(), i + 1, j + 1, dist);

                    /* revert the robot model with current state */
                    robotModelUpdate(curr_root_att, curr_actuator_vector);

                    return false;
                  }

                if(dist < min_dist)
                  {
                    min_dist = dist;
                    overlap_link1 = i;
                    overlap_link2 = j;
                  }
              }
          }

        if(debug)ROS_INFO("constraint: %s, overlap min dist %f between %d and %d", constraint_name_.c_str(), min_dist, overlap_link1 + 1, overlap_link2 + 1);

        /* update matrix A */
        double delta_angle = 0.001; // [rad]
        /* joint */
        for(int index = 0; index < planner_->getRobotModelPtr()->getLinkJointIndex().size(); index++)
          {
            KDL::JntArray perturbation_actuator_vector = curr_actuator_vector;
            perturbation_actuator_vector(planner_->getRobotModelPtr()->getLinkJointIndex().at(index)) += delta_angle;

            edfs_origin_from_cog = robotModelUpdate(curr_root_att,
                                                    perturbation_actuator_vector);
            auto dist = distCalc(edfs_origin_from_cog, overlap_link1, overlap_link2);
            A(0, 6 + index) = (dist - min_dist) / delta_angle;
          }

        if(full_body_)
          {
            /* roll */
            edfs_origin_from_cog = robotModelUpdate(curr_root_att * KDL::Rotation::RPY(delta_angle, 0, 0), curr_actuator_vector);
            auto dist = distCalc(edfs_origin_from_cog, overlap_link1, overlap_link2);
            A(0, 3) = (dist - min_dist) / delta_angle;

            /* pitch */
            edfs_origin_from_cog = robotModelUpdate(curr_root_att * KDL::Rotation::RPY(0, delta_angle, 0), curr_actuator_vector);
            dist = distCalc(edfs_origin_from_cog, overlap_link1, overlap_link2);
            A(0, 4) = (dist - min_dist) / delta_angle;

            /* yaw */
            edfs_origin_from_cog = robotModelUpdate(curr_root_att * KDL::Rotation::RPY(0, 0, delta_angle), curr_actuator_vector);
            dist = distCalc(edfs_origin_from_cog, overlap_link1, overlap_link2);
            A(0, 5) = (dist - min_dist) / delta_angle;
          }
        /* set lb */
        if(min_dist < overlap_constraint_range_)
            lb(0) *= ((min_dist - overlap_forbidden_range_) / (overlap_constraint_range_ - overlap_forbidden_range_));

        //lb(0) = -1e6;
        if(debug)
          {
            std::cout << "constraint (" << constraint_name_ << "): matrix A: \n" << A <<  "\n lb: \n" << lb.transpose() << "\n, ub: \n" << ub.transpose() << std::endl;
          }

        /* update reuslt */
        if(result_overlap_min_ > min_dist)
          {
            result_overlap_min_ = min_dist;
            result_overlap_link1_ = overlap_link1;
            result_overlap_link2_ = overlap_link2;
          }

        /* revert the robot model with current state */
        robotModelUpdate(curr_root_att, curr_actuator_vector);

        return true;
      }

      void result()
      {
        std::cout << constraint_name_ << "\n"
                  << "result overlap min: " << result_overlap_min_
                  << " at rotor" << result_overlap_link1_
                  << " at rotor" << result_overlap_link2_
                  << std::endl;
      }

      bool directConstraint(){return false;}

    protected:
      double overlap_change_vel_thre_;
      double overlap_constraint_range_;
      double overlap_forbidden_range_;

      double result_overlap_min_;
      int result_overlap_link1_;
      int result_overlap_link2_;
    };
  };
};

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(differential_kinematics::constraint::Overlap, differential_kinematics::constraint::Base);

