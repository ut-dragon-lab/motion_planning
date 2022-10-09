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
#include <dragon/model/hydrus_like_robot_model.h>


namespace differential_kinematics
{
  namespace constraint
  {
    class Overlap :public Base
    {
    public:
      Overlap(): overlap_min_(1e6)
      {}
      ~Overlap(){}

      void virtual initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                              boost::shared_ptr<differential_kinematics::Planner> planner,
                              std::string constraint_name,
                              bool orientation, bool full_body)
      {
        Base::initialize(nh, nhp, planner, constraint_name, orientation, full_body);

        nc_ = 1; // using one dimension

        getParam<double>("overlap_change_vel_thre", overlap_change_vel_thre_, 0.05);
        getParam<double>("overlap_constraint_range", overlap_constraint_range_, 0.05); // [m]
        getParam<double>("overlap_forbidden_range", overlap_forbidden_range_, 0.01); // [m]
      }

      bool getConstraint(Eigen::MatrixXd& A, Eigen::VectorXd& lb, Eigen::VectorXd& ub, bool debug = false)
      {
        auto robot_model = boost::dynamic_pointer_cast<Dragon::HydrusLikeRobotModel>(planner_->getRobotModelPtr());
        const auto joint_positions = planner_->getTargetJointVector<KDL::JntArray>();

        double rotor_diameter = 2 * robot_model->getEdfRadius();
        double min_dist = robot_model->getClosestRotorDist() - rotor_diameter;
        int rotor_i = robot_model->getClosestRotorIndices().at(0);
        int rotor_j = robot_model->getClosestRotorIndices().at(1);
        const std::vector<Eigen::Vector3d> rotor_pos = robot_model->getEdfsOriginFromCog<Eigen::Vector3d>();
        const auto& rotor_names = robot_model->getEdfNames();
        double tan_max_tilt = tan(robot_model->getEdfMaxTilt());


        Eigen::MatrixXd jacobian = robot_model->getRotorOverlapJacobian();
        A = jacobian;
        lb = Eigen::VectorXd::Constant(nc_, -overlap_change_vel_thre_);
        ub = Eigen::VectorXd::Constant(nc_, 1e6);

        lb(0) = damplingBound(min_dist, -overlap_change_vel_thre_, overlap_constraint_range_, overlap_forbidden_range_);

        if(debug)
          {
            ROS_INFO("constraint: %s, overlap min dist %f between %d and %d", constraint_name_.c_str(), min_dist, rotor_i, rotor_j);
            std::cout << "constraint (" << constraint_name_ << "): matrix A: \n" << A <<  "\n lb: \n" << lb.transpose() << "\n, ub: \n" << ub.transpose() << std::endl;
          }

        /* update reuslt */
        if(overlap_min_ > min_dist)
          {
            overlap_min_ = min_dist;
            rotor_i_ = rotor_i;
            rotor_j_ = rotor_j;
          }

        return true;
      }

      void result()
      {
        std::cout << constraint_name_ << "\n"
                  << "result overlap min: " << overlap_min_
                  << " at rotor" << rotor_i_
                  << " at rotor" << rotor_j_
                  << std::endl;
      }

      bool directConstraint(){return false;}

    protected:
      double overlap_change_vel_thre_;
      double overlap_constraint_range_;
      double overlap_forbidden_range_;

      double overlap_min_;
      int rotor_i_;
      int rotor_j_;
    };
  };
};

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(differential_kinematics::constraint::Overlap, differential_kinematics::constraint::Base);

