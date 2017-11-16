// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, JSK Lab
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

/* TODO: not implement multi side search */

#include <aerial_transportation/grasp_form_searching/search_algorithm/full_search.h>

namespace grasp_form_search
{

  void FullSearch::initialize(ros::NodeHandle nh, ros::NodeHandle nhp, GraspFormSearch* form_searcher)
  {
    SearchAlgorithmBase::initialize(nh, nhp, form_searcher);

    nhp_.param("one_side_flag", one_side_flag_, true);
    std::cout << "[search algorithm] one_side_flag: " << one_side_flag_ << std::endl;
    nhp_.param("res_d", res_d_, 0.001); //[m]
    std::cout << "[search algorithm] res_d: " << res_d_ << std::endl;
    nhp_.param("res_delta", res_delta_, 0.01); //[rad]
    std::cout << "[search algorithm] res_delta: " << res_delta_ << std::endl;

    nhp_.param("delta_valid_range", delta_valid_range_, 0.1); //[rad]
    std::cout << "[search algorithm] delta_valid_range: " << delta_valid_range_ << std::endl;
    nhp_.param("d_valid_range", d_valid_range_, 0.03); //[mm]
    std::cout << "[search algorithm] d_valid_range: " << d_valid_range_ << std::endl;

    nhp_.param("thrust_weight", thrust_weight_, 0.8);
    std::cout << "[search algorithm] thrust_weight: " << thrust_weight_ << std::endl;

    nhp_.param("torque_weight", torque_weight_, 0.4);
    std::cout << "[search algorithm] torque_weight: " << torque_weight_ << std::endl;

  }

  void FullSearch::reset()
  {
    SearchAlgorithmBase::reset();
    search_map_.clear();
  }

  bool FullSearch::getSolution()
  {
    int contact_num = form_searcher_->getContactNum();

    std::vector<double> v_theta;
    std::vector<double> v_delta;
    std::vector<double> v_contact_d;
    std::vector<Vector3d> v_contact_p;
    std::vector<Quaterniond> v_contact_rot;
    std::vector<Vector3d> v_joint_p;

    VectorXd v_hover_thrust;
    VectorXd v_min_f_fc = VectorXd::Zero(contact_num * 3);
    VectorXd v_tau = VectorXd::Zero(contact_num -1 );

    std::vector<double> v_best_theta;
    std::vector<double> v_best_delta;
    std::vector<double> v_best_contact_d;
    std::vector<Vector3d> v_best_contact_p;
    std::vector<Quaterniond> v_best_contact_rot;
    std::vector<Vector3d> v_best_joint_p;
    VectorXd v_best_hover_thrust;
    VectorXd v_best_min_f_fc = VectorXd::Zero(contact_num * 3);
    VectorXd v_best_tau = VectorXd::Zero(contact_num -1 );

    /* valid range */
    std::vector<double> v_valid_lower_bound_theta;
    std::vector<double> v_valid_upper_bound_theta;
    std::vector<double> v_valid_lower_bound_delta;
    std::vector<double> v_valid_upper_bound_delta;
    std::vector<double> v_valid_lower_bound_contact_d;
    std::vector<double> v_valid_upper_bound_contact_d;

    std::ofstream log_ofs; /* for record to a file */
    if(search_file_log_flag_) log_ofs.open( "full_searching_log.txt" );

    double link_length = form_searcher_->getLinkLength();
    double duct_radius = form_searcher_->getDuctRadius();
    double object_radius = form_searcher_->getObjectRadius();

    /* find better grasp form region */
    double min_cost_sum = 1e6;

    switch(form_searcher_->getObjectType())
      {
      case aerial_transportation::ObjectConfigure::Request::CONVEX_POLYGONAL_COLUMN:
        {
          int best_start_side = 0;

          /* full search for each side to serve as the first side*/
          for(int i = 0; i < form_searcher_->getSideNum(); i++)
            {
              double side_length = form_searcher_->getObjectInfo()[i]->len_;
              // double l_delta = - atan2(duct_radius, d);
              // double u_delta = atan2(duct_radius, v_side_length_[0] -d );
              double l_delta = - M_PI / 2;
              double u_delta = M_PI / 2;

              search_map_.push_back(MatrixXd::Constant(side_length / res_d_, (u_delta - l_delta) / res_delta_, 1e6));

              int row;
              int col;
              double d;
              double delta;
              for(d = 0, row = 0; d < side_length; d += res_d_, row++)
                {
                  for(delta = l_delta, col = 0; delta < u_delta; delta += res_delta_, col++)
                    {

                      bool kinematics_validity = true;
                      bool statics_validity = true;
                      /* 1. calculate the joint angles */
                      if(!form_searcher_->envelopingCalc(d, delta, v_theta, v_delta, v_contact_d, v_contact_p, v_contact_rot, v_joint_p, i))
                        kinematics_validity = false;
                      /* 2. calculate the joint angles */
                      if(kinematics_validity)
                        {
                          if(!form_searcher_->graspForceClosure(v_contact_p, v_contact_rot, v_joint_p, v_tau, v_min_f_fc))
                            statics_validity = false;


                          tf::Transform tf_object_origin_to_uav_root;
                          tf::Vector3 uav_root_origin;
                          tf::vectorEigenToTF(v_joint_p[0], uav_root_origin);
                          tf_object_origin_to_uav_root.setOrigin(uav_root_origin);
                          tf::Quaternion uav_root_q;
                          tf::quaternionEigenToTF(form_searcher_->getObjectInfo()[i]->contact_rot_ * AngleAxisd(delta, Vector3d::UnitZ()), uav_root_q);
                          tf_object_origin_to_uav_root.setRotation(uav_root_q);
                          if(!form_searcher_->hoveringStatics(v_theta, tf_object_origin_to_uav_root.inverse(), v_hover_thrust))
                            statics_validity = false;
                        }
                      else
                        statics_validity = false;

                      /* 2.5 show the result of the validity */
                      if(kinematics_validity && !statics_validity)
                        ROS_ERROR("[full search]: d:%f, delta:%f, l_delta:%f, u_delta:%f, statics invalid", d, delta, l_delta, u_delta);
                      if(kinematics_validity && statics_validity)
                        {
                          ROS_INFO("[full search]: d:%f, delta:%f, l_delta:%f, u_delta:%f, valid state", d, delta, l_delta, u_delta);
                        }

                      /* 3. update  */
                      if(statics_validity)
                        {
                          /* weighted sum */
#if 1
                          search_map_[i](row, col) = thrust_weight_ * v_hover_thrust.norm() + torque_weight_ * v_tau.maxCoeff();
#else
                          double diff_min_ave_thrust = fabs(v_hover_thrust.minCoeff() - average_thrust);
                          double diff_max_ave_thrust = fabs(v_hover_thrust.maxCoeff() - average_thrust);
                          if(diff_max_ave_thrust > diff_min_ave_thrust)
                            search_map_[i](row, col) = thrust_weight_ * diff_max_ave_thrust + torque_weight_ * v_tau.maxCoeff();
                          else
                            search_map_[i](row, col) = thrust_weight_ * diff_min_ave_thrust + torque_weight_ * v_tau.maxCoeff();

#endif
                        }

                      if(search_file_log_flag_)
                        {
                          if(kinematics_validity)
                            {
                              log_ofs << i + 1 << "\t" << d  << "\t" << delta << "\t" << search_map_[i](row, col);
                              if(statics_validity)
                                log_ofs << "\t" << v_tau.maxCoeff() << "\t"  << v_hover_thrust.maxCoeff() << "\t"  << v_hover_thrust.minCoeff()  << std::endl;
                              else
                                log_ofs << "\t" << -1 << "\t"  << -1 << "\t"  << -1  << std::endl;
                            }
                        }
                    }
                }

              for(row = 0; row < search_map_[i].rows(); row++)
                {
                  for(col = 0; col < search_map_[i].cols(); col++)
                    {
                      int d_valid_range  = d_valid_range_ / res_d_;
                      int delta_valid_range  = delta_valid_range_ / res_delta_;

                      if(row + d_valid_range >= search_map_[i].rows() ||
                         col + delta_valid_range > search_map_[i].cols())
                        continue;

                      double cost_sum = search_map_[i].block(row, col, d_valid_range, delta_valid_range).sum();
                      if(min_cost_sum > cost_sum)
                        {
                          min_cost_sum = cost_sum;
                          if(search_verbose_) ROS_INFO("update min cost sum: %f, at side %d", min_cost_sum, i);
                          best_start_side = i;

                          double center_d = row * res_d_ + d_valid_range_ / 2;
                          double center_delta = col * res_delta_ + l_delta + delta_valid_range_ / 2;

                          if(!form_searcher_->envelopingCalc(center_d, center_delta, v_best_theta, v_best_delta, v_best_contact_d, v_best_contact_p, v_best_contact_rot, v_best_joint_p, i))
                            ROS_FATAL("[full search]: can not get the valid kinematics from the best confiugration");
                          /* check force-closure */
                          if(!form_searcher_->graspForceClosure(v_best_contact_p, v_best_contact_rot, v_best_joint_p, v_best_tau, v_best_min_f_fc))
                            ROS_FATAL("[full search]: can not get the valid force-closure from the best confiugration");
                          /* check hovering  */
                          tf::Transform tf_object_origin_to_uav_root;
                          tf::Vector3 uav_root_origin;
                          tf::vectorEigenToTF(v_best_joint_p[0], uav_root_origin);
                          tf_object_origin_to_uav_root.setOrigin(uav_root_origin);
                          tf::Quaternion uav_root_q;
                          tf::quaternionEigenToTF(form_searcher_->getObjectInfo()[i]->contact_rot_ * AngleAxisd(v_best_delta[9], Vector3d::UnitZ()), uav_root_q);
                          tf_object_origin_to_uav_root.setRotation(uav_root_q);
                          if(!form_searcher_->hoveringStatics(v_best_theta, tf_object_origin_to_uav_root.inverse(), v_best_hover_thrust))
                            ROS_FATAL("[full search]: can not get the valid hovering state from the best confiugration");

                            /* set the valid range of the contact_d and delta */
                          double valid_lower_d = row * res_d_ + d_valid_range_;
                          double valid_lower_delta = col * res_delta_ + l_delta + delta_valid_range_;

                          if(!form_searcher_->envelopingCalc(valid_lower_d, valid_lower_delta, v_valid_lower_bound_theta, v_valid_lower_bound_delta, v_valid_lower_bound_contact_d, v_contact_p, v_contact_rot, v_joint_p, i))
                            ROS_FATAL("[full search]: can not get the valid kinematics from the valid lower bound configuration");

                          double valid_upper_d = row * res_d_;
                          double valid_upper_delta = col * res_delta_ + l_delta;

                          if(!form_searcher_->envelopingCalc(valid_upper_d, valid_upper_delta, v_valid_upper_bound_theta, v_valid_upper_bound_delta, v_valid_upper_bound_contact_d, v_contact_p, v_contact_rot, v_joint_p, i))
                            ROS_FATAL("[full search]: can not get the valid kinematics from the valid upper bound configuration");
                        }
                    }
                }
              if(one_side_flag_) break;
            }

          /* set the final result */
          form_searcher_->setBestStartSide(best_start_side);
          form_searcher_->setBestForceClosureResult(v_best_theta, v_valid_lower_bound_theta, v_valid_upper_bound_theta, v_best_delta, v_valid_lower_bound_delta, v_valid_upper_bound_delta, v_best_contact_p, v_best_contact_rot, v_best_joint_p, v_best_min_f_fc, v_best_tau, v_hover_thrust);
          form_searcher_->setBestContactDVector(v_best_contact_d, v_valid_lower_bound_contact_d, v_valid_upper_bound_contact_d);
          break;
        }
      case aerial_transportation::ObjectConfigure::Request::CYLINDER:
        {
          double u_delta = M_PI / 2  - acos((link_length / 2) / (duct_radius + object_radius));
          double l_delta = - u_delta;

          search_map_.push_back(MatrixXd::Constant( (u_delta - l_delta) / res_delta_, 1, 1e6));

          int index;
          double delta;
          for(delta = l_delta, index = 0 ; delta <= u_delta; delta += res_delta_, index++)
            {
              bool kinematics_validity = true;
              bool statics_validity = true;

              /* 1. calculate the joint angles */
              if(!form_searcher_->envelopingCalc(0, delta, v_theta, v_delta, v_contact_d, v_contact_p, v_contact_rot, v_joint_p, 0))
                kinematics_validity = false;

              /* 2. calculate the joint angles */
              if(kinematics_validity)
                {
                  if(!form_searcher_->graspForceClosure(v_contact_p, v_contact_rot, v_joint_p, v_tau, v_min_f_fc))
                    statics_validity = false;

                  tf::Transform tf_object_origin_to_uav_root;
                  tf::Vector3 uav_root_origin;
                  tf::vectorEigenToTF(v_joint_p[0], uav_root_origin);
                  tf_object_origin_to_uav_root.setOrigin(uav_root_origin);
                  tf::Quaternion uav_root_q;
                  tf::quaternionEigenToTF(Quaterniond(AngleAxisd(delta, Vector3d::UnitZ())), uav_root_q);
                  tf_object_origin_to_uav_root.setRotation(uav_root_q);
                  if(!form_searcher_->hoveringStatics(v_theta, tf_object_origin_to_uav_root.inverse(), v_hover_thrust))
                    statics_validity = false;
                }
              else
                statics_validity = false;

              /* 2.5 show the result of the validity */
              if(!kinematics_validity)
                ROS_WARN("[full search] index: %d, delta:%f, l_delta:%f, u_delta:%f, kinematics invalid", index, delta, l_delta, u_delta);
              if(kinematics_validity && !statics_validity)
                ROS_ERROR("[full search] index: %d, delta:%f, l_delta:%f, u_delta:%f, statics invalid", index, delta, l_delta, u_delta);
              if(kinematics_validity && statics_validity)
                ROS_INFO("[full search] index: %d, delta:%f, l_delta:%f, u_delta:%f, valid state", index, delta, l_delta, u_delta);

              /* 3. update  */
              if(statics_validity)
                {
#if 1
                  search_map_[0](index, 0) = thrust_weight_ * v_hover_thrust.norm() + torque_weight_ * v_tau.maxCoeff();
#else
                  /* weighted sum */
                  double diff_min_ave_thrust = fabs(v_hover_thrust.minCoeff() - average_thrust);
                  double diff_max_ave_thrust = fabs(v_hover_thrust.maxCoeff() - average_thrust);
                  if(diff_max_ave_thrust > diff_min_ave_thrust)
                    search_map_[0](index, 0) = thrust_weight_ * diff_max_ave_thrust + torque_weight_ * v_tau.maxCoeff();
                  else
                    search_map_[0](index, 0) = thrust_weight_ * diff_min_ave_thrust + torque_weight_ * v_tau.maxCoeff();
#endif
                }

              if(search_file_log_flag_)
                {
                  if(kinematics_validity)
                    {
                      log_ofs << delta << "\t" << search_map_[0](index, 0);
                      if(statics_validity)
                        log_ofs << "\t" << v_tau.maxCoeff() << "\t"  << v_hover_thrust.maxCoeff() << "\t"  << v_hover_thrust.minCoeff()  << std::endl;
                    }
                }
            }

          for(int row = 0; row < search_map_[0].rows(); row++)
            {
              int delta_valid_range  = delta_valid_range_ / res_delta_;

              if(row + delta_valid_range >= search_map_[0].rows()) break;

              double cost_sum = search_map_[0].block(row, 0, delta_valid_range, 1).sum();

              //ROS_INFO("start patch search: cost_sum: %f, min_cost: %f", cost_sum, min_cost_sum);

              if(min_cost_sum > cost_sum)
                {

                  min_cost_sum = cost_sum;
                  if(search_verbose_) ROS_INFO("update min cost sum: %f", min_cost_sum);
                  double center_delta = row * res_delta_ + l_delta + delta_valid_range_ / 2;

                  if(!form_searcher_->envelopingCalc(0, center_delta, v_best_theta, v_best_delta, v_best_contact_d, v_best_contact_p, v_best_contact_rot, v_best_joint_p, 0))
                    ROS_FATAL("[full search]: can not get the valid kinematics from the best confiugration");
                  /* check force-closure */
                  if(!form_searcher_->graspForceClosure(v_best_contact_p, v_best_contact_rot, v_best_joint_p, v_best_tau, v_best_min_f_fc))
                    ROS_FATAL("[full search]: can not get the valid force-closure from the best confiugration");

                  /* check hovering state */
                  tf::Transform tf_object_origin_to_uav_root;
                  tf::Vector3 uav_root_origin;
                  tf::vectorEigenToTF(v_best_joint_p[0], uav_root_origin);
                  tf_object_origin_to_uav_root.setOrigin(uav_root_origin);
                  tf::Quaternion uav_root_q;
                  tf::quaternionEigenToTF(Quaterniond(AngleAxisd(v_best_delta[0], Vector3d::UnitZ())), uav_root_q);
                  tf_object_origin_to_uav_root.setRotation(uav_root_q);
                  if(!form_searcher_->hoveringStatics(v_best_theta, tf_object_origin_to_uav_root.inverse(), v_best_hover_thrust))
                    ROS_FATAL("[full search]: can not get the valid hovering state from the best confiugration");

                  /* set the valid range of the contact_d and delta */
                  double valid_lower_delta = row * res_delta_ + l_delta;
                  if(!form_searcher_->envelopingCalc(0, valid_lower_delta, v_valid_lower_bound_theta, v_valid_lower_bound_delta, v_valid_lower_bound_contact_d, v_contact_p, v_contact_rot, v_joint_p, 0))
                    ROS_FATAL("[full search]: can not get the valid kinematics from the valid lower bound configuration");

                  double valid_upper_delta = row * res_delta_ + l_delta + delta_valid_range_;
                  if(!form_searcher_->envelopingCalc(0, valid_upper_delta, v_valid_upper_bound_theta, v_valid_upper_bound_delta, v_valid_upper_bound_contact_d, v_contact_p, v_contact_rot, v_joint_p, 0))
                    ROS_FATAL("[full search]: can not get the valid kinematics from the valid upper bound configuration");
                }
            }

          /* set the final result */
          form_searcher_->setBestForceClosureResult(v_best_theta, v_valid_lower_bound_theta, v_valid_upper_bound_theta, v_best_delta, v_valid_lower_bound_delta, v_valid_upper_bound_delta, v_best_contact_p, v_best_contact_rot, v_best_joint_p, v_best_min_f_fc, v_best_tau, v_best_hover_thrust);
          break;
        }
      default:
        {
          break;
        }
      }
    return true;
  }
};

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(grasp_form_search::FullSearch, grasp_form_search::SearchAlgorithmBase);
