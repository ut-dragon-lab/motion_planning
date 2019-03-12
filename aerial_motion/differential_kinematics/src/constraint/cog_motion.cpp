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

        nhp_.param("cog_omega_limit_flag", cog_omega_limit_flag_, false);
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

        makeJointSegmentMap();
        /*
          for(const auto& joint_itr: joint_segment_map_)
          {
          for(const auto& seg_itr: joint_itr.second)
          std::cout << joint_itr.first << ", id: " << actuator_map_.at(joint_itr.first) << ", " << seg_itr << std::endl;
          }
        */
      }


      bool getConstraint(Eigen::MatrixXd& A, Eigen::VectorXd& lb, Eigen::VectorXd& ub, bool debug = false)
      {
        //debug = true;

        lb = Eigen::VectorXd::Constant(nc_, 0);
        ub = Eigen::VectorXd::Constant(nc_, 0);
        lb.head(3) = - Eigen::Map<Eigen::Vector3d>(velocity_limit_.data(), 3);
        ub.head(3) = Eigen::Map<Eigen::Vector3d>(velocity_limit_.data(), 3);

        /* approximate to a rigid body has a inertial same with the robot model */
        lb.tail(3) = - planner_->getRobotModelPtr()->getInertia<Eigen::Matrix3d>() * Eigen::Map<Eigen::Vector3d>(angular_limit_.data(), 3);
        ub.tail(3) = planner_->getRobotModelPtr()->getInertia<Eigen::Matrix3d>() * Eigen::Map<Eigen::Vector3d>(angular_limit_.data(), 3);

        //std::cout << " total inertia: \n" << planner_->getRobotModelPtr()->getInertia<Eigen::Matrix3d>() << std::endl;

        // get the approximated jacobian, becuase we do not consider the change of gimbal angles by joint angles.
        if(!calcCOGJacobian(A, debug)) return false;

#if 0 // debug by linear approximation, but we cannot calculate angular momentum
        KDL::Rotation curr_root_att;
        tf::quaternionTFToKDL(planner_->getTargetRootPose().getRotation(), curr_root_att);
        auto curr_actuator_vector =  planner_->getTargetActuatorVector<KDL::JntArray>();
        Eigen::MatrixXd test_jacobian = Eigen::MatrixXd::Zero(3, planner_->getRobotModelPtr()->getLinkJointIndex().size() + 6);

        auto robotModelUpdate = [this](KDL::Rotation root_att, KDL::JntArray actuator_vector)
          {
            planner_->getRobotModelPtr()->setCogDesireOrientation(root_att);
            planner_->getRobotModelPtr()->updateRobotModel(actuator_vector);
            planner_->getRobotModelPtr()->stabilityMarginCheck();
            planner_->getRobotModelPtr()->modelling();
          };

        /* CoG velocity */
        Eigen::Vector3d nominal_cog = planner_->getRobotModelPtr()->getCog<Eigen::Affine3d>().translation();

        /* joint */
        double delta_angle = 0.001; // [rad]
        for(int index = 0; index < planner_->getRobotModelPtr()->getLinkJointIndex().size(); index++)
          {
            KDL::JntArray perturbation_actuator_vector = curr_actuator_vector;
            perturbation_actuator_vector(planner_->getRobotModelPtr()->getLinkJointIndex().at(index)) += delta_angle;
            robotModelUpdate(curr_root_att, perturbation_actuator_vector);

            test_jacobian.block(0, 6 + index, 3, 1) =  aerial_robot_model::kdlToEigen(curr_root_att) * (planner_->getRobotModelPtr()->getCog<Eigen::Affine3d>().translation() - nominal_cog) /delta_angle;
          }

        /* root */
        /* linear velocity */
        test_jacobian.block(0, 0, 3, 3) =  aerial_robot_model::kdlToEigen(curr_root_att);
        /* roll */
        robotModelUpdate(curr_root_att * KDL::Rotation::RPY(delta_angle, 0, 0), curr_actuator_vector);
        test_jacobian.block(0, 3, 3, 1) = (aerial_robot_model::kdlToEigen(curr_root_att * KDL::Rotation::RPY(delta_angle, 0, 0)) * planner_->getRobotModelPtr()->getCog<Eigen::Affine3d>().translation() -  aerial_robot_model::kdlToEigen(curr_root_att) * nominal_cog) / delta_angle;

        /*  pitch */
        robotModelUpdate(curr_root_att * KDL::Rotation::RPY(0, delta_angle, 0), curr_actuator_vector);
        test_jacobian.block(0, 4, 3, 1) = (aerial_robot_model::kdlToEigen(curr_root_att * KDL::Rotation::RPY(0, delta_angle, 0)) * planner_->getRobotModelPtr()->getCog<Eigen::Affine3d>().translation() -  aerial_robot_model::kdlToEigen(curr_root_att) * nominal_cog) / delta_angle;

        /* yaw */
        robotModelUpdate(curr_root_att * KDL::Rotation::RPY(0, 0, delta_angle), curr_actuator_vector);
        test_jacobian.block(0, 5, 3, 1) = (aerial_robot_model::kdlToEigen(curr_root_att * KDL::Rotation::RPY(0, 0, delta_angle)) * planner_->getRobotModelPtr()->getCog<Eigen::Affine3d>().translation() -  aerial_robot_model::kdlToEigen(curr_root_att) * nominal_cog) / delta_angle;


        if(debug)
          std::cout << "constraint (" << constraint_name_.c_str()  << "): matrix test: \n" << test_jacobian << std::endl;
        /* 0. revert the robot model with current state */
        robotModelUpdate(curr_root_att, curr_actuator_vector);
#endif

        if(debug)
          {
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
      bool cog_omega_limit_flag_;
      Eigen::MatrixXd cog_jacobian_; //cog velocty+angular jacobian
      std::map<std::string, int> joint_hierachy_;
      std::map<std::string, std::vector<std::string> > joint_segment_map_;

      void makeJointSegmentMap()
      {
        joint_segment_map_.clear();
        const auto actuator_map = planner_->getRobotModelPtr()->getActuatorMap();

        for (const auto actuator : actuator_map) {
          std::vector<std::string> empty_vec;
          joint_segment_map_[actuator.first] = empty_vec;
        }

        std::vector<std::string> current_joints;
        jointSegmentSetupRecursive(planner_->getRobotModelPtr()->getTree().getRootSegment()->second, current_joints);
      }

      void jointSegmentSetupRecursive(const KDL::TreeElement& tree_element, std::vector<std::string> current_joints)
      {
        const auto inertia_map = planner_->getRobotModelPtr()->getInertiaMap();
        const KDL::Segment current_seg = GetTreeElementSegment(tree_element);
        bool add_joint_flag = false;

        // if this segment has a real joint except rotor
        if (current_seg.getJoint().getType() != KDL::Joint::None && current_seg.getJoint().getName().find("rotor") == std::string::npos) {
          std::string focused_joint = current_seg.getJoint().getName();
          current_joints.push_back(focused_joint);
          bool add_joint_flag = true;
        }

        // if this segment is a real segment (= not having fixed joint)
        if (inertia_map.find(current_seg.getName()) != inertia_map.end() || current_seg.getName().find("thrust") != std::string::npos) {
          for (const auto& cj : current_joints) {
            joint_segment_map_.at(cj).push_back(current_seg.getName());
          }
        }

        // recursive process
        for (const auto& elem: GetTreeElementChildren(tree_element)) {
          jointSegmentSetupRecursive(elem->second, current_joints);
        }

        if (add_joint_flag) {
          current_joints.pop_back();
        }
        return;
      }

      bool calcCOGJacobian(Eigen::MatrixXd& jacobian, bool debug)
      {
        double mass_all = planner_->getRobotModelPtr()->getMass();

        jacobian = Eigen::MatrixXd::Zero(nc_, planner_->getRobotModelPtr()->getLinkJointIndex().size() + 6);
        /*
           Note: the jacobian about the cog velocity (linear momentum) and angular momentum.

           Please refer to:
           ============================================================================
           S. Kajita et al., "Resolved momentum control: humanoid motion planning based on the linear and angular momentum," Proceedings 2003 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS 2003) (Cat. No.03CH37453), Las Vegas, NV, USA, 2003, pp. 1644-1650 vol.2.
           ============================================================================

           1. the cog velocity is w.r.t in the frame which has same origin with the root link frame (e.g. {link_1}), but the orientation is coincided with world frame.
           2. the angular momentum is w.r.t in cog frame with desired_orientation_frame

         */

        KDL::Rotation curr_root_att;
        tf::quaternionTFToKDL(planner_->getTargetRootPose().getRotation(), curr_root_att);
        const auto& seg_frames =  planner_->getRobotModelPtr()->getSegmentsTf();
        const auto& segment_map = planner_->getRobotModelPtr()->getTree().getSegments();
        const auto& inertia_map = planner_->getRobotModelPtr()->getInertiaMap();
        const auto& actuator_map = planner_->getRobotModelPtr()->getActuatorMap();

        KDL::Vector total_cog = planner_->getRobotModelPtr()->getCog<KDL::Frame>().p;
        Eigen::MatrixXd cog_velocity_jacobian = Eigen::MatrixXd::Zero(3, planner_->getTargetActuatorVector<KDL::JntArray>().rows());
        Eigen::MatrixXd cog_angular_jacobian = Eigen::MatrixXd::Zero(3, planner_->getTargetActuatorVector<KDL::JntArray>().rows());

        for (const auto& joint_segment : joint_segment_map_)
          {
            /* get first child segment */

            std::string joint_child_segment_name = joint_segment.second.at(0);
            KDL::Segment joint_child_segment = GetTreeElementSegment(segment_map.at(joint_child_segment_name));
            KDL::Vector a = seg_frames.at(joint_child_segment_name).M * joint_child_segment.getJoint().JointAxis();
            KDL::Vector r = seg_frames.at(joint_child_segment_name).p;
            KDL::RigidBodyInertia inertia = KDL::RigidBodyInertia::Zero();

            /* recursively get the local sum inertia */
            for (const auto& seg : joint_segment.second) {
              //ROS_INFO("   segment: %s", seg.c_str());
              if (seg.find("thrust") == std::string::npos) {
                KDL::Frame f = seg_frames.at(seg);
                inertia = inertia + f * inertia_map.at(seg);
              }
            }

            auto col_index = actuator_map.at(joint_segment.first);
            KDL::Vector c = inertia.getCOG();
            double m = inertia.getMass();
            KDL::Vector momentum_jacobian_col = a * (c - r) * m;
            KDL::Vector angular_jacobian_col = (c - total_cog) * momentum_jacobian_col + inertia.getRotationalInertia() * a;
            cog_velocity_jacobian.col(col_index) = aerial_robot_model::kdlToEigen(curr_root_att * momentum_jacobian_col  / mass_all);
            cog_angular_jacobian.col(col_index) = aerial_robot_model::kdlToEigen(curr_root_att * angular_jacobian_col); // R{root_link} => R{cog} = R{world}
          }

        /* extract the jacobian directly connected with link joint */
        for(int i = 0; i < planner_->getRobotModelPtr()->getLinkJointIndex().size(); i++)
          {
            jacobian.block(0, 6 + i, 3, 1) = cog_velocity_jacobian.block(0, planner_->getRobotModelPtr()->getLinkJointIndex().at(i), 3, 1);
            jacobian.block(3, 6 + i, 3, 1) = cog_angular_jacobian.block(0, planner_->getRobotModelPtr()->getLinkJointIndex().at(i), 3, 1);
          }

        if(full_body_)
          {
            /* root link */
            /* - linear velocity */
            jacobian.block(0, 0, 3, 3) = aerial_robot_model::kdlToEigen(curr_root_att);
            jacobian.block(0, 3, 3, 1) = aerial_robot_model::kdlToEigen(curr_root_att) * (Eigen::Vector3d(1, 0, 0)).cross(aerial_robot_model::kdlToEigen(total_cog));
            jacobian.block(0, 4, 3, 1) = aerial_robot_model::kdlToEigen(curr_root_att) * (Eigen::Vector3d(0, 1, 0)).cross(aerial_robot_model::kdlToEigen(total_cog));
            jacobian.block(0, 5, 3, 1) = aerial_robot_model::kdlToEigen(curr_root_att) * (Eigen::Vector3d(0, 0, 1)).cross(aerial_robot_model::kdlToEigen(total_cog));

            /* - angular momentum */
            jacobian.block(3, 3, 3, 3) = planner_->getRobotModelPtr()->getInertia<Eigen::Matrix3d>();
          }
        if(debug) std::cout << "jacobian with only joint: \n " <<  jacobian << std::endl; //debug

        /* calculate the jacobian from link joint to gimbal angles at each link */
        if(planner_->getGimbalModuleFlag())
          {
            KDL::TreeJntToJacSolver jac_solver(planner_->getRobotModelPtr()->getTree());
            KDL::Jacobian jac_root_link(planner_->getRobotModelPtr()->getTree().getNrOfJoints());

            Eigen::MatrixXd gimbal_joint_jacobian = Eigen::MatrixXd::Zero(3, planner_->getRobotModelPtr()->getLinkJointIndex().size()); // R^{3 x N_j}

            for(int i = 0; i < rotor_num_; i++)
              {
                /* calculate the jacobian from joints to rotation of {thrust_i} w.r.t {world} */
                if(jac_solver.JntToJac(planner_->getTargetActuatorVector<KDL::JntArray>(),
                                       jac_root_link, std::string("thrust") + std::to_string(i + 1)) == KDL::SolverI::E_NOERROR)
                  {
                    //std::cout << "raw whole jacobian: \n" << jac_root_link.data << std::endl;

                    /* extract the rotation only jacobian from link joint to the thrust link */
                    Eigen::MatrixXd thrust_joint_rot_jacobian_world = Eigen::MatrixXd::Zero(3, planner_->getRobotModelPtr()->getLinkJointIndex().size()); // R^{3 x N_j}
                    for(int index = 0; index < planner_->getRobotModelPtr()->getLinkJointIndex().size(); index++)
                      {
                        thrust_joint_rot_jacobian_world.block(0, index, 3, 1)  =  aerial_robot_model::kdlToEigen(curr_root_att) * jac_root_link.data.block(3, planner_->getRobotModelPtr()->getLinkJointIndex().at(index), 3, 1);
                      }

                    //std::cout << "thrust_joint_rot_jacobian_world: \n" << thrust_joint_rot_jacobian_world << std::endl;
                    Eigen::MatrixXd thrust_gimbal_rot_jacobian_world = Eigen::MatrixXd::Zero(3, 2); // R^{3 x 2}
                    /* TODO: hard-coding about the name of gimbal link */
                    auto gimbal1_index = actuator_map.at(std::string("gimbal") + std::to_string(i + 1) + std::string("_roll"));
                    auto gimbal2_index = actuator_map.at(std::string("gimbal") + std::to_string(i + 1) + std::string("_pitch"));
                    thrust_gimbal_rot_jacobian_world.block(0, 0, 3, 1) = aerial_robot_model::kdlToEigen(curr_root_att) * jac_root_link.data.block(3, gimbal1_index, 3, 1);
                    thrust_gimbal_rot_jacobian_world.block(0, 1, 3, 1) = aerial_robot_model::kdlToEigen(curr_root_att) * jac_root_link.data.block(3, gimbal2_index, 3, 1);

                    //std::cout << "thrust_gimbal_rot_jacobian_world: \n" << thrust_gimbal_rot_jacobian_world  << std::endl;

                    Eigen::MatrixXd gimbal_joint_jacobian = - (Eigen::MatrixXd::Identity(2,3) * thrust_gimbal_rot_jacobian_world).inverse() * Eigen::MatrixXd::Identity(2,3) * thrust_joint_rot_jacobian_world; // R^{2 x N_j}

                    /* combine for joint: J = J_joint + J_gimbal * J_gimbal_joint */
                    jacobian.block(0, 6, 3, gimbal_joint_jacobian.cols()) += (cog_velocity_jacobian.block(0, gimbal1_index, 3, 1) * gimbal_joint_jacobian.row(0) + cog_velocity_jacobian.block(0, gimbal2_index, 3, 1) * gimbal_joint_jacobian.row(1));

                    jacobian.block(3, 6, 3, gimbal_joint_jacobian.cols()) += (cog_angular_jacobian.block(0, gimbal1_index, 3, 1) * gimbal_joint_jacobian.row(0) + cog_angular_jacobian.block(0, gimbal2_index, 3, 1) * gimbal_joint_jacobian.row(1));

                    if(full_body_)
                      {
                        /* combine for root link */

                        gimbal_joint_jacobian = - (Eigen::MatrixXd::Identity(2,3) * thrust_gimbal_rot_jacobian_world).inverse() * Eigen::MatrixXd::Identity(2,3) * aerial_robot_model::kdlToEigen(curr_root_att);

                        jacobian.block(0, 3, 3, 3) += (cog_velocity_jacobian.block(0, gimbal1_index, 3, 1) * gimbal_joint_jacobian.row(0) + cog_velocity_jacobian.block(0, gimbal2_index, 3, 1) * gimbal_joint_jacobian.row(1));

                        jacobian.block(3, 3, 3, 3) += (cog_angular_jacobian.block(0, gimbal1_index, 3, 1) * gimbal_joint_jacobian.row(0) + cog_angular_jacobian.block(0, gimbal2_index, 3, 1) * gimbal_joint_jacobian.row(1));
                      }
                  }
                else
                  {
                    ROS_WARN("constraint (%s) can not calculate the jacobian", constraint_name_.c_str());
                    return false;
                  }
              }
          }

        /* special process */
        if(planner_->getMultilinkType() == motion_type::SE2)
          jacobian.block(2, 0, 3, jacobian.cols()) = Eigen::MatrixXd::Zero(3, jacobian.cols());

        if(debug)
          std::cout << "cog jacobian combined: \n" << jacobian << std::endl;

        return true;
      }

    };
  };
};

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(differential_kinematics::constraint::CogMotion, differential_kinematics::constraint::Base);

