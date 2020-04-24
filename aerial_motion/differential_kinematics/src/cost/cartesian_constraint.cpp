#include <differential_kinematics/cost/cartesian_constraint.h>

namespace differential_kinematics
{
  namespace cost
  {
    void CartersianConstraint::initialize(ros::NodeHandle nh,
                                                          ros::NodeHandle nhp,
                                                          boost::shared_ptr<differential_kinematics::Planner> planner,
                                                          std::string cost_name,
                                                          bool orientation, bool full_body)
    {
      Base::initialize(nh, nhp, planner, cost_name, orientation, full_body);

      nhp_.param ("pos_convergence_thre", pos_convergence_thre_, 0.001); //[m]
      if(verbose_) std::cout << "pos_convergence_thre: " << std::setprecision(3) << pos_convergence_thre_ << std::endl;
      nhp_.param ("rot_convergence_thre", rot_convergence_thre_, 0.017); //[rad]
      if(verbose_) std::cout << "rot_convergence_thre: " << std::setprecision(3) << rot_convergence_thre_ << std::endl;
      nhp_.param ("pos_err_max", pos_err_max_, 0.1); //[m]
      if(verbose_) std::cout << "pos_err_max: " << std::setprecision(3) << pos_err_max_ << std::endl;
      nhp_.param ("rot_err_max", rot_err_max_, 0.17); //[rad]
      if(verbose_) std::cout << "rot_err_max: " << std::setprecision(3) << rot_err_max_ << std::endl;
      nhp_.param ("w_pos_err_constraint", w_pos_err_constraint_, 1.0);
      if(verbose_) std::cout << "w_pos_err_constraint: " << std::setprecision(3) << w_pos_err_constraint_ << std::endl;
      nhp_.param ("w_att_err_constraint", w_att_err_constraint_, 1.0);
      if(verbose_) std::cout << "w_att_err_constraint: " << std::setprecision(3) << w_att_err_constraint_ << std::endl;

      W_cartesian_err_constraint_ = Eigen::MatrixXd::Identity(6, 6);
      W_cartesian_err_constraint_.block(0, 0, 3, 3) *=  w_pos_err_constraint_;
      if(orientation_) W_cartesian_err_constraint_.block(3, 3, 3, 3) *= w_att_err_constraint_;
      else W_cartesian_err_constraint_.block(3, 3, 3, 3) = Eigen::MatrixXd::Zero(3, 3);
    }

    bool CartersianConstraint::getHessianGradient(bool& convergence, Eigen::MatrixXd& H, Eigen::VectorXd& g, bool debug)
    {
      //debug = true;
      convergence = false;

      KDL::Frame end_frame;
      KDL::ChainFkSolverPos_recursive fk_solver(chain_);
      KDL::JntArray joint_positions(chain_.getNrOfJoints());

      // TODO: check the validity
      for(int i = 0; i < chain_joint_index_.size(); i++)
        joint_positions(i) = planner_->getTargetJointVector<KDL::JntArray>()(chain_joint_index_.at(i));
      fk_solver.JntToCart(joint_positions, end_frame);

      tf::Transform end_tf;
      tf::transformKDLToTF(end_frame, end_tf);
      if(debug)
        ROS_WARN("end [%f, %f, %f], yaw: %f, target end  [%f, %f, %f], yaw: %f", (planner_->getTargetRootPose() * end_tf).getOrigin().x(), (planner_->getTargetRootPose() * end_tf).getOrigin().y(), (planner_->getTargetRootPose() * end_tf).getOrigin().z(), tf::getYaw((planner_->getTargetRootPose() * end_tf).getRotation()), target_reference_frame_.getOrigin().x(), target_reference_frame_.getOrigin().y(), target_reference_frame_.getOrigin().z(), tf::getYaw(target_reference_frame_.getRotation()));

      /* full axis */
      //tf::Transform tf_err_ee =  (planner_->getTargetRootPose() * end_tf).inverse() * target_reference_frame_;
      //double pos_err = tf_err.getOrigin().length();
      //double rot_err = fabs(tf_err.getRotation().getAngleShortestPath());

      /* process free axis */
      tf::Transform tf_err_ref =  target_reference_frame_.inverse() * planner_->getTargetRootPose() * end_tf; //default
      /* -- translation -- */
      //ROS_ERROR("tf_err_ref: [%f, %f, %f]", tf_err_ref.getOrigin().x(), tf_err_ref.getOrigin().y(), tf_err_ref.getOrigin().z());
      tf_err_ref.getOrigin().setValue(tf_err_ref.getOrigin().x() * free_axis_mask_(MaskAxis::TRAN_X),
                                  tf_err_ref.getOrigin().y() * free_axis_mask_(MaskAxis::TRAN_Y),
                                  tf_err_ref.getOrigin().z() * free_axis_mask_(MaskAxis::TRAN_Z));

      //ROS_ERROR("tf_err_ref after: [%f, %f, %f]", tf_err_ref.getOrigin().x(), tf_err_ref.getOrigin().y(), tf_err_ref.getOrigin().z());
      //double r,p,y;
      //(planner_->getTargetRootPose() * end_tf).getBasis().getRPY(r,p,y);
      //ROS_ERROR("end euler: [%f, %f, %f]", r,p,y);
      //target_reference_frame_.getBasis().getRPY(r,p,y);
      //ROS_ERROR("target ref: [%f, %f, %f]", r,p,y);

      double pos_err = tf_err_ref.getOrigin().length();
      /* -- rotation -- */
      double rot_err = fabs(tf_err_ref.getRotation().getAngleShortestPath()); //default
      assert(tf_err_ref.getRotation().getAngle() >= 0);
      //ROS_ERROR("angle: %f", tf_err_ref.getRotation().getAngle()); //
      if(free_axis_mask_(MaskAxis::ROT_X) == 0)
        {
          tf::Vector3 coincide_axis = tf_err_ref.getBasis().getColumn(0);
          rot_err = atan2(sqrt(std::pow(coincide_axis.y(), 2) + std::pow(coincide_axis.z(), 2)), coincide_axis.x() );
        }
      if(free_axis_mask_(MaskAxis::ROT_Y) == 0)
        {
          tf::Vector3 coincide_axis = tf_err_ref.getBasis().getColumn(1);
          rot_err = atan2(sqrt(std::pow(coincide_axis.x(), 2) + std::pow(coincide_axis.z(), 2)), coincide_axis.y() );
        }
      if(free_axis_mask_(MaskAxis::ROT_Z) == 0)
        {
          tf::Vector3 coincide_axis = tf_err_ref.getBasis().getColumn(2);
          rot_err = atan2(sqrt(std::pow(coincide_axis.x(), 2) + std::pow(coincide_axis.y(), 2)), coincide_axis.z() );
        }

      if(debug)
        ROS_INFO("pos err, rot(angle): %f[m], %f[rad], err pos vec w.r.t ref frame : [%f, %f, %f]", pos_err, rot_err, tf_err_ref.getOrigin().x(), tf_err_ref.getOrigin().y(), tf_err_ref.getOrigin().z());

      /* check convergence */
      if(pos_err < pos_convergence_thre_ ) /* position convergence */
        {
          if(!orientation_)  convergence = true;
          else
            if(rot_err < rot_convergence_thre_) convergence = true;
        }

      /* transform the cartesian err to the root link frame */
      if(pos_err > pos_err_max_) pos_err = pos_err_max_;
      if(rot_err > rot_err_max_) rot_err = rot_err_max_;

      /* set err in root link frame */
      //tf::Vector3 target_pos_err_root_link = end_tf.getBasis() * tf_err.getOrigin().normalize() * pos_err;
      //tf::Vector3 target_rot_err_root_link = end_tf.getBasis() * tf_err.getRotation().getAxis() * rot_err;
      /* set err in ref frame (in terms of orientation) */
      tf::Vector3 target_pos_err_ref = -tf_err_ref.getOrigin().normalize() * pos_err;
      tf::Vector3 target_rot_err_ref;
      if(tf_err_ref.getRotation().getAngle() > M_PI)
        target_rot_err_ref = tf_err_ref.getRotation().getAxis() * rot_err;
      else
        target_rot_err_ref = -tf_err_ref.getRotation().getAxis() * rot_err;

      if(free_axis_mask_(MaskAxis::ROT_X) == 0)
        {
          if((tf_err_ref.getBasis().getColumn(0).cross(tf::Vector3(1, 0, 0))).length() < 1e-6)
            target_rot_err_ref = tf::Vector3(0,0,0);
          else
            target_rot_err_ref = (tf_err_ref.getBasis().getColumn(0).cross(tf::Vector3(1, 0, 0))).normalize() * rot_err;
        }
      if(free_axis_mask_(MaskAxis::ROT_Y) == 0)
        {
          if((tf_err_ref.getBasis().getColumn(1).cross(tf::Vector3(0, 1, 0))).length() < 1e-6)
            target_rot_err_ref = tf::Vector3(0,0,0);
          else
            target_rot_err_ref = (tf_err_ref.getBasis().getColumn(1).cross(tf::Vector3(0, 1, 0))).normalize() * rot_err;
        }
      if(free_axis_mask_(MaskAxis::ROT_Z) == 0)
        {
          if((tf_err_ref.getBasis().getColumn(2).cross(tf::Vector3(0, 0, 1))).length() < 1e-6)
            target_rot_err_ref = tf::Vector3(0,0,0);
          else
            target_rot_err_ref = (tf_err_ref.getBasis().getColumn(2).cross(tf::Vector3(0, 0, 1))).normalize() * rot_err;
        }

      Eigen::VectorXd delta_cartesian = Eigen::VectorXd::Zero(6);
      Eigen::Vector3d temp_vec;
      tf::vectorTFToEigen(target_pos_err_ref, temp_vec);
      delta_cartesian.head(3) = temp_vec;
      tf::vectorTFToEigen(target_rot_err_ref, temp_vec);
      delta_cartesian.tail(3) = temp_vec;

      if(planner_->getMultilinkType() == motion_type::SE2)
        {
          if(!orientation_) delta_cartesian.segment(2, 4) = Eigen::VectorXd::Zero(4);
          else delta_cartesian.segment(2, 3) = Eigen::VectorXd::Zero(3);
        }
      else
        {
          if(!orientation_) delta_cartesian.segment(3, 3) = Eigen::VectorXd::Zero(3);
        }

      if(debug) std::cout << "delta cartesian: \n" << delta_cartesian.transpose() << std::endl;

      Eigen::MatrixXd jacobian;
      if(!calcJointJacobian(jacobian, debug)) return false;

      /*
        QP:
        - cost function : (J dq - dx)^T W1 (J dq - dx) + dq^T W2 dq  :  The least square error term for equality conditions(e.g. IK)  + vel (i.e. joint, root) penality
        - dq^T (J^T W1 J + W2) dq - 2 dx^T W1 J dq + hoge
        - since:-2 dx^T W1 J = -2 (J^T W1^T dx)^T = -2 (J^T W1 dx)^T
      */
      H = jacobian.transpose() * W_cartesian_err_constraint_ * jacobian;
      /* CAUTION: becuase of QP-OASES, the scale "2" is included inside the function */
      g = - delta_cartesian.transpose()  * W_cartesian_err_constraint_ * jacobian;

      /*
      Eigen::Matrix3d root_rot;
      tf::matrixTFToEigen((planner_->getTargetRootPose().inverse() * target_reference_frame_).getBasis(), root_rot);
      Eigen::MatrixXd extend_mat = Eigen::MatrixXd::Identity(6,6);
      extend_mat.block(0,0,3,3) = root_rot;
      extend_mat.block(3,3,3,3) = root_rot;
      H =  (extend_mat * jacobian).transpose() * W_cartesian_err_constraint_ * (extend_mat * jacobian);
      g = - (extend_mat * delta_cartesian).transpose()  * W_cartesian_err_constraint_ * (extend_mat * jacobian);
      */

      if(debug)
        {
          std::cout << "the Weight Matrix of cartesian_err constraint: \n" << W_cartesian_err_constraint_ << std::endl;
          std::cout << "cost name: " << cost_name_ << ", H: \n" << H << std::endl;
          std::cout << "cost name: " << cost_name_ << ", g: \n" << g.transpose() << std::endl;
        }
      return true;
    }

    bool CartersianConstraint::calcJointJacobian(Eigen::MatrixXd& jacobian, bool debug)
    {
      Eigen::MatrixXd jacobian_root_link = Eigen::MatrixXd::Zero(6, planner_->getRobotModelPtr()->getLinkJointIndices().size() + 6);

      /* fill the joint state */
      KDL::JntArray joint_positions(chain_.getNrOfJoints());
      for(int i = 0; i < chain_joint_index_.size(); i++)
        joint_positions(i) = planner_->getTargetJointVector<KDL::JntArray>()(chain_joint_index_.at(i));

      if(chain_.getNrOfJoints() > 0 )
        {
          /* calculate the jacobian */
          KDL::ChainJntToJacSolver jac_solver(chain_);
          KDL::Jacobian jac(chain_.getNrOfJoints());
          if(jac_solver.JntToJac(joint_positions, jac) == KDL::SolverI::E_NOERROR)
            {
              /* joint part */
              jacobian_root_link.block(0, 6, 6, jac.data.cols()) = jac.data;
            }
          else
            {
              ROS_WARN("cost (%s) can not calculate the jacobian", cost_name_.c_str());
              return false;
            }
        }

      if(full_body_)
        {
          /* consider root is attached with a 6Dof free joint */

          /* get end frame position from root */
          KDL::Frame end_frame;
          KDL::ChainFkSolverPos_recursive fk_solver(chain_);
          fk_solver.JntToCart(joint_positions, end_frame);

          if(debug)
            ROS_INFO("end pose: [%f %f %f]", end_frame.p.x(), end_frame.p.y(), end_frame.p.z());

          /* root link */
          jacobian_root_link.block(0, 0, 6, 6) = Eigen::MatrixXd::Identity(6, 6);
          jacobian_root_link.block(0, 3, 3, 1) = (Eigen::Vector3d(1, 0, 0)).cross(Eigen::Vector3d(end_frame.p.data));
          jacobian_root_link.block(0, 4, 3, 1) = (Eigen::Vector3d(0, 1, 0)).cross(Eigen::Vector3d(end_frame.p.data));
          jacobian_root_link.block(0, 5, 3, 1) = (Eigen::Vector3d(0, 0, 1)).cross(Eigen::Vector3d(end_frame.p.data));
        }

      /* change to reference frame */
      Eigen::MatrixXd jacobian_ref = jacobian_root_link; //init set
      Eigen::Matrix3d ref_rot;
      int q_size = planner_->getRobotModelPtr()->getLinkJointIndices().size() + 6;
      tf::matrixTFToEigen((target_reference_frame_.inverse() * planner_->getTargetRootPose()).getBasis(), ref_rot);
      jacobian_ref.block(0, 0, 3, q_size) = ref_rot * jacobian_root_link.block(0, 0, 3, q_size);
      jacobian_ref.block(3, 0, 3, q_size) = ref_rot * jacobian_root_link.block(3, 0, 3, q_size);
      /* applay free axis mask */
      jacobian = (Eigen::Map<Eigen::VectorXd>(free_axis_mask_.data(), free_axis_mask_.size())).asDiagonal() * jacobian_ref;

      //Eigen::MatrixXd temp = (Eigen::Map<Eigen::VectorXd>(free_axis_mask_.data(), free_axis_mask_.size())).asDiagonal();
      //std::cout << " mask eigen: \n" << temp << std::endl;

      /* special */
      if(planner_->getMultilinkType() == motion_type::SE2)
        {
          if(!orientation_)
            jacobian.block(2, 0, 4, jacobian.cols()) = Eigen::MatrixXd::Zero(4, jacobian.cols());
          else
            jacobian.block(2, 0, 3, jacobian.cols()) = Eigen::MatrixXd::Zero(3, jacobian.cols());
        }
      else
        {
          if(!orientation_)
            jacobian.block(3, 0, 3, jacobian.cols()) = Eigen::MatrixXd::Zero(3, jacobian.cols());
        }

      if(debug) std::cout << "jacobian: \n" << jacobian << std::endl;
      return true;
    }
  };
};

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(differential_kinematics::cost::CartersianConstraint, differential_kinematics::cost::Base);
