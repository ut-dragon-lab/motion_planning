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

#include <aerial_transportation/grasp_form_searching/grasp_form_search.h>

#include <qpOASES.hpp>
#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>

/* file */
#include <iostream>
#include <sstream>
#include <fstream>

using namespace Eigen;
USING_NAMESPACE_QPOASES

namespace grasp_form_search
{
  namespace
  {
    bool statics_verbose_;

    double fric_z_mu_; /* the rate of vertical firction */
    double fric_x_mu_; /* the rate of horizontal firction */

    /* the QP solver variables */
    /* TODO: should not use sqp, use qp */
    boost::shared_ptr<SQProblem> fc_solver_;
    bool qp_init_flag_;
    MatrixXd H_;
    MatrixXd A_;
    MatrixXd g_;
    VectorXd lb_;
    VectorXd ub_;
    VectorXd lA_;
    VectorXd uA_;
    int n_wsr_;
    Options qp_options_;
  }

  void GraspFormSearch::staticsInit()
  {
    nhp_.param("statics_verbose", statics_verbose_, false);
    if(verbose_) std::cout << "[statics] verbose: " << statics_verbose_ << std::endl;
    nhp_.param("fric_x_mu", fric_x_mu_, 0.1);
    if(verbose_) std::cout << "[statics] fric_x_mu: " << fric_x_mu_ << std::endl;
    nhp_.param("fric_z_mu", fric_z_mu_, 0.1);
    if(verbose_) std::cout << "[statics] fric_z_mu: " << fric_z_mu_ << std::endl;

    /* qp problem */
    nhp_.param("n_wsr", n_wsr_, 10);
    if(verbose_) std::cout << "[statics] QP, n_wsr: " << n_wsr_ << std::endl;

    fc_solver_ = boost::shared_ptr<SQProblem>(new SQProblem(contact_num_ * 3, contact_num_ * 4 + 6));
    qp_init_flag_ = false;

    /* Hessian */
    H_ = MatrixXd::Zero(contact_num_ * 3, contact_num_ * 3);
    Matrix3d h;
    h << 1, 0, 0, 0, 1, 0, 0, 0, 0;
    for(int i = 0; i < contact_num_; i++)
      H_.block(3 * i, 3 * i, 3, 3) = h;

    /* linear constraints */
    A_ = MatrixXd::Zero(6 + contact_num_ * 4, contact_num_ * 3);
    MatrixXd G_fric = MatrixXd::Zero(4 * contact_num_, 3 * contact_num_);
    MatrixXd g_fric(4,3);
    g_fric << fric_x_mu_, -1, 0, fric_x_mu_, 1, 0, fric_z_mu_, 0, -1, fric_z_mu_, 0, 1;
    for(int i = 0; i < contact_num_; i++)
      { /* G_{fric} = diag(G_{fric \ 1}, \cdots, G_{fric \ K}), G_{fric}f_{FC} \req 0 */
        G_fric.block(4 * i, 3 * i, 4, 3) = g_fric;
      }
    A_.block(6, 0, 4 * contact_num_, 3 * contact_num_) = G_fric;
    if(statics_verbose_)
      {
        std::cout << "[statics] A_ : \n " << A_ << std::endl;
        std::cout << "[statics] G_fric : \n " << G_fric << std::endl;
      }

    /* gradient */
    g_ = MatrixXd::Zero(1, contact_num_ * 3);

    /* lower & upper bound for linear constraints */
    lA_ = VectorXd::Zero(6 + contact_num_ * 4);
    uA_ = VectorXd::Constant(6 + contact_num_ * 4, INFTY);
    VectorXd Fe(6);
    Fe << 0, 0, -object_inertia_.m * 9.797, 0, 0, 0; //external force
    lA_.block(0, 0, 6, 1) = - Fe;
    uA_.block(0, 0, 6, 1) = - Fe;
    if(statics_verbose_)
      {
        std::cout << "[statics] lA_ T: " << lA_.transpose() << std::endl;
        std::cout << "[statics] uA_ T: " << uA_.transpose() << std::endl;
      }

    /* lower & upper bound for variables */
    lb_ = VectorXd::Constant(contact_num_ * 3, -INFTY);
    for(int i = 0; i < contact_num_; i++) lb_(i * 3) = 0;
    ub_ = VectorXd::Constant(contact_num_ * 3, INFTY);
    if(statics_verbose_)
      {
        std::cout << "[statics] lb_ T: " << lb_.transpose() << std::endl;
        std::cout << "[statics] ub_ T: " << ub_.transpose() << std::endl;
      }

    qp_options_.enableEqualities = BT_TRUE;
    qp_options_.printLevel = PL_LOW;
    fc_solver_->setOptions(qp_options_);
  }

  bool GraspFormSearch::graspForceClosure(std::vector<Vector3d> v_contact_p, std::vector<Quaterniond> v_contact_rot, std::vector<Vector3d> v_joint_p, VectorXd& tau, VectorXd& min_f_fc)
  {
    assert(v_contact_p.size() == v_joint_p.size());
    /* init */
    tau = VectorXd::Zero(v_contact_p.size()  - 1);

    auto skew = [](Eigen::Vector3d vec) -> Matrix3d
    {
      Matrix3d skew;
      skew << 0.0, -vec(2), vec(1),
      vec(2), 0.0, -vec(0),
      -vec(1), vec(0), 0.0;
      return skew;
    };

    /* 1. calculate the force in each contact point*/
    /* this is based on the force closure and the minimize the internal forces */
    /* force closure: G_c\bm{f_{FC}} + \F_e = 0, \bm{f_{FC}} = [\bm{f_{ci}}, \cdots, \bm{f_{cin}]^{t}  */
    /* constraints: \bm{f_{ci}} \in FC,  FC = { [f_x, f_y, f_z]^T \mid  f^2_y + f^2_z \leq \mu^2 f^2_x, f_x \req 0} */
    /* linear constraints: FC = { [f_x, f_y, f_z]^T \mid f_x \req 0,  -\mu f_x \leq f_y \leq \mu f_x, 0 \leq f_z \leq \mu f_x} */

    /* force closure constraints */
    MatrixXd G_c = MatrixXd::Zero(6, 3 * contact_num_);

    Vector3d object_com;
    tf::vectorMsgToEigen(object_inertia_.com, object_com);

    for(int i = 0; i < contact_num_; i++)
      {
        /* G_c = [R_c [P_oc x]R_c] */
        Quaternion<double> R_ci = v_contact_rot[i] * AngleAxisd(M_PI / 2, Vector3d::UnitZ()); //absolute angle from first link

        MatrixXd G_ci(6, 3);
        G_ci.block(0, 0, 3, 3) = R_ci.matrix();
        G_ci.block(3, 0, 3, 3) = skew(v_contact_p[i] - object_com) * R_ci.matrix();

        //TODO remove!!!
        for(int j = 0; j < 6; j++)
          {
            for(int k = 0; k < 3; k++)
              {
                if(fabs(G_ci(j,k)) < 1e-6) G_ci(j,k) = 0;
              }
          }

        G_c.block(0, 3 * i, 6, 3) = G_ci;
      }
    A_.block(0,0, 6, 3 * contact_num_) = G_c;
    if(statics_verbose_)
      {
        std::cout << "[statics]  G_c : \n " << G_c << std::endl;
        std::cout << "[statics]  A_ : \n " << A_  << std::endl;
      }
    MatrixXd At = A_.transpose(); //eigen is col-major order array in default

    /* approximated friction constraints */
    /* constant */

    /* calulate the minimum force-closure based QP solver */
    double start_tm = ros::Time::now().toSec();
    n_wsr_ = 100; //why can not set from rosparam?
    int solver_result = 0;

    if(!qp_init_flag_)
      { /* first time */
        qp_init_flag_ = true;

        solver_result = fc_solver_->init(H_.data(), g_.data(), At.data(),
                                         lb_.data(), ub_.data(), lA_.data(), uA_.data(), n_wsr_);
        // std::cout << "  H_ : \n " << H_ << std::endl;
        // std::cout << "  g_ : \n " << g_ << std::endl;
        // std::cout << "  A_ : \n " << A_ << std::endl;
        // std::cout << "  lb_ : \n " << lb_ << std::endl;
        // std::cout << "  ub_ : \n " << ub_ << std::endl;
        // std::cout << "  lA_ : \n " << lA_ << std::endl;
        // std::cout << "  uA_ : \n " << uA_ << std::endl;
      }
    else
      solver_result = fc_solver_->hotstart(H_.data(), g_.data(), At.data(),
                                           lb_.data(), ub_.data(), lA_.data(), uA_.data(), n_wsr_);

    fc_solver_->getPrimalSolution(min_f_fc.data());
    //fc_solver_->printProperties();

    //ROS_INFO("the solver_result is %d", solver_result);
    if(solver_result != 0)
      {
        if(statics_verbose_)
          ROS_ERROR("[statics] QP: the solver_result is %d", solver_result);
        return false;
      }
    else
      {
        if(statics_verbose_)
          std::cout << "[statics] QP: solution time is " << ros::Time::now().toSec() - start_tm << std::endl;
      }

    if(statics_verbose_)
      std::cout << "[statics] QP: min f fc: [" << min_f_fc.transpose() << "]"  << std::endl;

    /* 2. calculate the torque of each joint
       /* the statics of link, based on the equation of 3.3.55 at page 119 of "robot motion" */
    /* eq: \bm{\tau} = - \sum{K}{i = 1}J_{i}^{T}(\bm{\theta})\bm{Q}_{i} */
    for(int i = 0; i < contact_num_; i++)
      { // i : the force point
        MatrixXd J_b_i = MatrixXd::Zero(6, contact_num_ - 1);
        for(int j = 0; j < contact_num_ - 1; j++)
          {// j : the joint
            VectorXd  J_b_ij = VectorXd::Zero(6);
            if(i - j > 0)
              {
                Vector3d e_z(0,0,1);
                J_b_ij.block(0, 0, 3, 1) = e_z.cross(v_contact_p[i] - v_joint_p[j + 1]);
                J_b_ij.block(3, 0, 3, 1) = e_z;
                J_b_i.block(0, j, 6 , 1) = J_b_ij;
              }
          }
        if(statics_verbose_) std::cout << "[statics] J_b_" << i << " : \n " << J_b_i << std::endl;
        //tau += J_b_i.transpose() * G_c.block(0, 3 * i, 6, 3) * min_f_fc.block(3 * i, 0, 3, 1);
        VectorXd tau_tmp = tau;
        tau = tau_tmp + J_b_i.transpose() * G_c.block(0, 3 * i, 6, 3) * min_f_fc.block(3 * i, 0, 3, 1);
      }
    if(statics_verbose_)
      std::cout << "[statics] QP:  tau: [" << tau.transpose() << "]" << std::endl;
  }

  bool GraspFormSearch::hoveringStatics(std::vector<double>& v_theta, tf::Transform tf_uav_root_to_object_origin, VectorXd& hover_thrust)
  {
    assert(v_theta.size() == contact_num_ - 1);

    std::string object_name("object");
    std::string parent_link_name("link1");
    geometry_msgs::Transform uav_root_to_object_origin_msg;
    tf::transformTFToMsg(tf_uav_root_to_object_origin, uav_root_to_object_origin_msg);

    /* clear */
    uav_kinematics_->addExtraModule(hydrus::AddExtraModule::Request::REMOVE, object_name, parent_link_name, uav_root_to_object_origin_msg, object_inertia_);
    /* add */
    uav_kinematics_->addExtraModule(hydrus::AddExtraModule::Request::ADD, object_name, parent_link_name, uav_root_to_object_origin_msg, object_inertia_);

    sensor_msgs::JointState joint_states;
    int joint_num = uav_kinematics_->getRotorNum() - 1;
    joint_states.position.resize(joint_num, 0);
    joint_states.name.resize(joint_num);
    for(int i = 0; i < joint_num; i++)
      {
        std::stringstream joint_no;
        joint_no << i + 1;
        if(i < contact_num_ - 1) joint_states.position[i] = v_theta[i];
        joint_states.name[i] = std::string("joint") + joint_no.str();
      }

    uav_kinematics_->kinematics(joint_states);
    bool status = uav_kinematics_->modelling(statics_verbose_);
    hover_thrust  = uav_kinematics_->getStableState();

    if(!status)
      {
        if(statics_verbose_)
          ROS_WARN("[statics] hovering check: can not stable with four axis mode");
        return false;
      }

    if(statics_verbose_)
      ROS_WARN("[statics] hovering check: the max/min element of hovering thrust is [%f, %f]", hover_thrust.maxCoeff(), hover_thrust.minCoeff());

    return true;
  }
};
