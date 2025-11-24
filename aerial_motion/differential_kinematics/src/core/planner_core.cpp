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

#include <differential_kinematics/planner_core.h>

/* cost & constraint */
#include <differential_kinematics/cost/base_plugin.h>
#include <differential_kinematics/constraint/base_plugin.h>

/* TODO: this is an workaround */
#include <dragon/model/hydrus_like_robot_model.h> // TODO: change to full vectoring robot model


namespace differential_kinematics
{
  /* pre-definition */
  using CostContainer = std::vector<boost::shared_ptr<cost::Base> >;
  using ConstraintContainer = std::vector<boost::shared_ptr<constraint::Base> >;

  Planner::Planner(ros::NodeHandle nh, ros::NodeHandle nhp, boost::shared_ptr<aerial_robot_model::transformable::RobotModel> robot_model_ptr): nh_(nh), nhp_(nhp), robot_model_ptr_(robot_model_ptr), multilink_type_(motion_type::SE2), gimbal_module_flag_(false), motion_func_vector_(), solved_(false)
  {
    target_joint_vector_.resize(robot_model_ptr_->getTree().getNrOfJoints());
    for(auto tree_itr : robot_model_ptr_->getTree().getSegments())
      {
        std::string joint_name = tree_itr.second.segment.getJoint().getName();

        /* TODO: workaround to detect whether this is a SE2 model or a SE3 model */
        if(joint_name.find("joint") == 0 &&
           tree_itr.second.segment.getJoint().getType() != KDL::Joint::JointType::None)
          {
            if(tree_itr.second.segment.getJoint().JointAxis().z() == 0)
              {
                multilink_type_ = motion_type::SE3;
              }
          }

        /* TODO: workaround to detect whether this is a model with gimbal moduel (e.g. dragon) */
        if(joint_name.find("gimbal") == 0 &&
           (joint_name.find("roll") != std::string::npos ||
            joint_name.find("pitch") != std::string::npos) &&
           tree_itr.second.segment.getJoint().getType() != KDL::Joint::JointType::None)
          {
            gimbal_module_flag_ = true;
          }
      }
    target_joint_vector_.resize(robot_model_ptr_->getTree().getNrOfJoints());
    nhp_.param ("differential_kinematics_count", differential_kinematics_count_, 100);

    /* publisher for joint state */
    std::string joint_state_pub_name;
    nhp_.param("joint_state_pub_name", joint_state_pub_name, std::string("joint_states"));
    joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>(joint_state_pub_name, 1);
    nhp_.param("tf_prefix", tf_prefix_, std::string(""));

    /* motion timer */
    double rate;
    nhp_.param("motion_func_rate", rate, 10.0);
    if(rate > 0)
      motion_timer_ = nhp_.createTimer(ros::Duration(1.0 / rate), &Planner::motionFunc, this);
  }

  template<> const KDL::Frame Planner::getTargetRootPose() const
  {
    return target_root_pose_;
  }

  template<> const tf::Transform Planner::getTargetRootPose() const
  {
    tf::Transform pose;
    tf::transformKDLToTF(target_root_pose_, pose);
    return pose;
  }

  template<> const sensor_msgs::JointState Planner::getTargetJointVector() const
  {
    return robot_model_ptr_->kdlJointToMsg(target_joint_vector_);
  }

  template<> const KDL::JntArray Planner::getTargetJointVector() const
  {
    return target_joint_vector_;
  }

  template<> void Planner::setTargetRootPose(const KDL::Frame target_root_pose)
  {
    target_root_pose_ = target_root_pose;
  }

  template<> void Planner::setTargetRootPose(const tf::Transform target_root_pose)
  {
    tf::transformTFToKDL(target_root_pose, target_root_pose_);
  }

  template<> void Planner::setTargetJointVector(const KDL::JntArray target_joint_vector)
  {
    target_joint_vector_ = target_joint_vector;
  }

  template<> void Planner::setTargetJointVector(const sensor_msgs::JointState target_joint_vector)
  {
    target_joint_vector_ = robot_model_ptr_->jointMsgToKdl(target_joint_vector);
  }

  bool Planner::solver(CostContainer cost_container, ConstraintContainer constraint_container, bool debug)
  {
    auto modelUpdate = [this]()
      {
        KDL::Rotation root_att;
        robot_model_ptr_->setCogDesireOrientation(target_root_pose_.M);
        robot_model_ptr_->updateRobotModel(target_joint_vector_);
        robot_model_ptr_->updateJacobians();

        if(!robot_model_ptr_->stabilityCheck()) ROS_ERROR("[differential kinematics]: invalid stability");

        /* update each cost or constraint (e.g. changable ik), if necessary */
        for(auto func_itr = update_func_vector_.begin(); func_itr != update_func_vector_.end(); func_itr++)
          {
          if(!(*func_itr)())
            {
              ROS_ERROR("update function fail");
              //solved_ = true; //debug
              return false;
            }
          }

        /* workaround: special process for model which has gimbal module (e.g. dragon) */
        if(gimbal_module_flag_)
          {
            auto dragon_model_ptr = boost::dynamic_pointer_cast<Dragon::HydrusLikeRobotModel>(robot_model_ptr_);
            assert(target_joint_vector_.rows() == dragon_model_ptr->getGimbalProcessedJoint<KDL::JntArray>().rows());
            target_joint_vector_ = dragon_model_ptr->getGimbalProcessedJoint<KDL::JntArray>();
          }

        /* considering the non-joint modules such as gimbal are updated after forward-kinemtics */
        /* the correct target_joint vector should be added here */
        target_root_pose_sequence_.push_back(target_root_pose_);
        target_joint_vector_sequence_.push_back(target_joint_vector_);

        return true;
      };

    robot_model_ptr_->setBaselinkName(std::string("link1"));

    target_joint_vector_sequence_.resize(0);
    target_root_pose_sequence_.resize(0);
    sequence_ = 0;
    double start_time = ros::Time::now().toSec();
    int total_nc = 0;
    for(auto itr: constraint_container) if(!itr->directConstraint()) total_nc += itr->getNc(); //for qpOases
    if(total_nc == 0)
      {
        ROS_ERROR("no propoer constraint for QP (qp oases), because the constraints number is zero (excluding the state range)");
        return false;
      }

    /* QP */
    /*
      1. QP cost function in qp Oases is : 1/2 * x^T H x + g^T x
      - so the hessian Matrix H shoube be multiplied by 2, or gradian g shoud be half.
      2. n_wrs is a total number in whole sequnece process, the value decrease every loop.
      3. the matrix in Eigen library is colunm-major, so use transpose to get row-major data.
      Plus, (A.transpose()).data is still column-mojar, please assign to a new matrix
    */
    boost::shared_ptr<SQProblem> qp_solver(new SQProblem(6 + robot_model_ptr_->getLinkJointIndices().size(), total_nc));
    bool qp_init_flag = true;

    int n_wsr = 100;
    Options qp_options;
    qp_options.enableEqualities = BT_TRUE;
    qp_options.printLevel = PL_LOW;
    qp_solver->setOptions(qp_options);

    /* inverse kinematics loop */
    for(int l = 0; l < differential_kinematics_count_; l++)
      {
        if(debug) ROS_WARN("loop: %d", l);
        Eigen::MatrixXd qp_H = Eigen::MatrixXd::Zero(qp_solver->getNV(), qp_solver->getNV());
        Eigen::MatrixXd qp_g = Eigen::VectorXd::Zero(qp_solver->getNV());
        Eigen::VectorXd qp_lb = Eigen::VectorXd::Constant(qp_solver->getNV(), -INFTY);
        Eigen::VectorXd qp_ub = Eigen::VectorXd::Constant(qp_solver->getNV(),  INFTY);

        Eigen::MatrixXd qp_A = Eigen::MatrixXd::Zero(qp_solver->getNC(), qp_solver->getNV());
        Eigen::VectorXd qp_lA = Eigen::VectorXd::Constant(qp_solver->getNC(), -INFTY);
        Eigen::VectorXd qp_uA = Eigen::VectorXd::Constant(qp_solver->getNC(), INFTY);

        // if(debug) std::cout << "the init qp lb is: \n" << qp_lb << std::endl;
        // if(debug) std::cout << "the init qp ub is: \n" << qp_ub << std::endl;
        if(l == 0) modelUpdate(); // store the init state
        /* step2: check convergence & update Hessian and Gradient */
        bool convergence = true;
        for(auto itr = cost_container.begin(); itr != cost_container.end(); itr++)
          {
            bool single_convergence;
            Eigen::MatrixXd single_H;
            Eigen::VectorXd single_g;
            /* hesian and gradient should be linear sum */
            (*itr)->getHessianGradient(single_convergence, single_H, single_g, debug);
            qp_H += single_H;
            qp_g += single_g;
            convergence &= single_convergence;
          }

        if(convergence)
          {
            ROS_INFO("convergence to target state with %d times with %f[sec]", l, ros::Time::now().toSec() - start_time);
            for(auto itr: constraint_container) itr->result();
            solved_ = true;
            return true;
          }

        /* step3: update Constaint Matrix and Bounds */
        size_t offset = 0;
        for(auto itr = constraint_container.begin(); itr != constraint_container.end(); itr++)
          {
            Eigen::MatrixXd single_A;
            Eigen::VectorXd single_lb;
            Eigen::VectorXd single_ub;

            if(!(*itr)->getConstraint(single_A, single_lb, single_ub, debug))
              {
                ROS_ERROR("constraint: %s is invalid", (*itr)->getConstraintName().c_str());
                return false;
              }

            /* for qpoasese */
            if((*itr)->directConstraint()) /* without constraint matrix */
              {
                if(qp_solver->getNV() != (*itr)->getNc())
                  {
                    ROS_ERROR("the constiant number from %s is %d, which is not equal with the state number %d",  (*itr)->getConstraintName().c_str(), (*itr)->getNc(), qp_solver->getNV());
                    return false;
                  }

                for(size_t i = 0; i < qp_solver->getNV(); i++)
                  {
                    if(single_lb(i) > qp_lb(i)) qp_lb(i) = single_lb(i);
                    if(single_ub(i) < qp_ub(i)) qp_ub(i) = single_ub(i);
                  }
              }
            else /* with constraint matrix */
              {
                qp_lA.segment(offset, (*itr)->getNc()) = single_lb;
                qp_uA.segment(offset, (*itr)->getNc()) = single_ub;
                qp_A.block(offset, 0, single_A.rows(), single_A.cols()) = single_A;
                offset += (*itr)->getNc();
              }
          }
        if(debug)
          {
            std::cout << "qp H \n" << qp_H << std::endl;
            std::cout << "qp g \n" << qp_g.transpose() << std::endl;
            std::cout << "qp A \n" << qp_A << std::endl;
            std::cout << "qp lA \n" << qp_lA.transpose() << std::endl;
            std::cout << "qp uA \n" << qp_uA.transpose() << std::endl;
          }

        /* step4: calculate the QP using qp-oases  */
        int solver_result;
        n_wsr = 100; /* this value have to be updated every time, otherwise it will decrease every loop */
        Eigen::MatrixXd qp_At = qp_A.transpose();
        if(qp_init_flag)
          { /* first time */
            qp_init_flag = false;

            solver_result = qp_solver->init(qp_H.data(), qp_g.data(),
                                            qp_At.data(), /* CAUTION: eigen is col-major order array in default! */
                                            qp_lb.data(), qp_ub.data(),
                                            qp_lA.data(), qp_uA.data(), n_wsr);
          }
        else
          {
            solver_result = qp_solver->hotstart(qp_H.data(), qp_g.data(),
                                                qp_At.data(),
                                                qp_lb.data(), qp_ub.data(),
                                                qp_lA.data(), qp_uA.data(), n_wsr);
          }

        if(solver_result != 0)
          {
            ROS_ERROR("can not solve QP the solver_result is %d", solver_result);
            solved_ = true; //debug
            return false;
          }

        Eigen::VectorXd delta_state_vector = Eigen::VectorXd::Zero(qp_solver->getNV());
        qp_solver->getPrimalSolution(delta_state_vector.data());
        if(debug)
          {
            std::cout << "delta state vector: \n" << delta_state_vector.transpose() << std::endl;
            std::cout << "qp_A * delta state vector: \n" << (qp_A *  delta_state_vector).transpose() << std::endl;
          }

        /* step5: update the state (root link & joint state) */
        /* root link incremental transformation: */
        KDL::Vector delta_pos, delta_rot;
        tf::vectorEigenToKDL(delta_state_vector.head(3), delta_pos);
        tf::vectorEigenToKDL(delta_state_vector.segment(3, 3), delta_rot);

        if(delta_rot.Norm() == 0)
          target_root_pose_ = target_root_pose_ *  KDL::Frame(KDL::Rotation::Identity(), delta_pos);
        else
          target_root_pose_ = target_root_pose_ *  KDL::Frame(KDL::Rotation::Rot(delta_rot, delta_rot.Norm()), delta_pos);

        /* udpate the joint angles */
        for(size_t i = 0; i < robot_model_ptr_->getLinkJointIndices().size(); i++) target_joint_vector_(robot_model_ptr_->getLinkJointIndices().at(i)) += delta_state_vector(i + 6);

        /* step6: update the kinematics by forward kinemtiacs, along with the modelling with current kinematics  */
        modelUpdate();

        if(debug)
          {
            ROS_WARN("finish loop %d", l);
            auto target_joint_state = getTargetJointVector<sensor_msgs::JointState>();
            std::cout << "target joint vector: ";
            for(int i = 0 ; i < target_joint_state.name.size(); i++)
              std::cout << "[" << target_joint_state.name.at(i) << ", " << target_joint_state.position.at(i) << "], " << std::endl;
            std::cout << std::endl;
          }
      }

    ROS_WARN("can not solve the differential kinematics based planning with %d", differential_kinematics_count_);
    solved_ = true; //debug
    return false;
  }

  void Planner::registerMotionFunc(std::function<void(void)> new_func)
  {
    motion_func_vector_.push_back(new_func);
  }

  void Planner::registerUpdateFunc(std::function<bool(void)> new_func)
  {
    update_func_vector_.push_back(new_func);
  }

  void Planner::motionFunc(const ros::TimerEvent & e)
  {
    if(solved_)
      {
        /* publish joint angle and root tf */
        ros::Time now_time = ros::Time::now();
        sensor_msgs::JointState joint_msg = robot_model_ptr_->kdlJointToMsg(target_joint_vector_sequence_.at(sequence_));
        joint_msg.header.stamp = now_time;

        tf::Transform root_pose;
        tf::transformKDLToTF(target_root_pose_sequence_.at(sequence_), root_pose);
        br_.sendTransform(tf::StampedTransform(root_pose, now_time, "world", tf::resolve(tf_prefix_, "root")));
        joint_state_pub_.publish(joint_msg);
        sequence_++;
        if(sequence_ == target_joint_vector_sequence_.size()) sequence_ = 0;

        /* additional function from external module */
        for(auto func_itr = motion_func_vector_.begin(); func_itr != motion_func_vector_.end(); func_itr++)
          (*func_itr)();
      }
  }

};
