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

      getParam<double>("pos_convergence_thre", pos_convergence_thre_, 0.001); //[m]
      getParam<double>("rot_convergence_thre", rot_convergence_thre_, 0.017); //[rad]
      getParam<double>("pos_err_max", pos_err_max_, 0.1); //[m]
      getParam<double>("angle_err_max", angle_err_max_, 0.17); //[rad]
      getParam<double>("w_pos_err_constraint", w_pos_err_constraint_, 1.0);
      getParam<double>("w_att_err_constraint", w_att_err_constraint_, 1.0);

      W_cartesian_err_constraint_ = Eigen::MatrixXd::Identity(6, 6);
      W_cartesian_err_constraint_.topLeftCorner(3, 3) *=  w_pos_err_constraint_;
      if(orientation_) W_cartesian_err_constraint_.bottomRightCorner(3, 3) *= w_att_err_constraint_;
      else W_cartesian_err_constraint_.bottomRightCorner(3, 3) = Eigen::MatrixXd::Zero(3, 3);

      // special process for SE2 model
      if(planner_->getMultilinkType() == motion_type::SE2)
        {
          free_axis_mask_(MaskAxis::TRAN_Z) = 0;
        }
    }

    bool CartersianConstraint::getHessianGradient(bool& convergence, Eigen::MatrixXd& H, Eigen::VectorXd& g, bool debug)
    {
      //debug = true;
      convergence = false;
      const auto robot_model = planner_->getRobotModelPtr();
      const auto& seg_frames = robot_model->getSegmentsTf();
      const KDL::Frame root_pose = planner_->getTargetRootPose<KDL::Frame>();

      KDL::Frame end_frame = seg_frames.at(parent_link_) * reference_frame_;

      /* full axis */
      /* process free axis */
      KDL::Frame pose_err =  target_frame_.Inverse() * root_pose * end_frame; //default
      /* -- translation -- */
      pose_err.p.x(pose_err.p.x() * free_axis_mask_(MaskAxis::TRAN_X));
      pose_err.p.y(pose_err.p.y() * free_axis_mask_(MaskAxis::TRAN_Y));
      pose_err.p.z(pose_err.p.z() * free_axis_mask_(MaskAxis::TRAN_Z));
      double pos_err = pose_err.p.Norm();


      /* -- rotation -- */
      tf::Quaternion rot_err;
      tf::quaternionKDLToTF(pose_err.M, rot_err);
      double angle_err = fabs(rot_err.getAngleShortestPath()); //default
      //ROS_ERROR("angle error: %f", angle_err);
      if(free_axis_mask_(MaskAxis::ROT_X) == 0)
        {
          tf::Vector3 coincide_axis = tf::Matrix3x3(rot_err).getColumn(0);
          angle_err = atan2(sqrt(std::pow(coincide_axis.y(), 2) + std::pow(coincide_axis.z(), 2)), coincide_axis.x() );
        }
      if(free_axis_mask_(MaskAxis::ROT_Y) == 0)
        {
          tf::Vector3 coincide_axis = tf::Matrix3x3(rot_err).getColumn(1);
          angle_err = atan2(sqrt(std::pow(coincide_axis.x(), 2) + std::pow(coincide_axis.z(), 2)), coincide_axis.y() );
        }
      if(free_axis_mask_(MaskAxis::ROT_Z) == 0)
        {
          tf::Vector3 coincide_axis = tf::Matrix3x3(rot_err).getColumn(2);
          angle_err = atan2(sqrt(std::pow(coincide_axis.x(), 2) + std::pow(coincide_axis.y(), 2)), coincide_axis.z() );
        }

      if(debug)
        ROS_INFO("pos err, angle err: %f[m], %f[rad], pos vec err w.r.t ref frame : [%f, %f, %f]", pos_err, angle_err, pose_err.p.x(), pose_err.p.y(), pose_err.p.z());

      /* check convergence */
      if(pos_err < pos_convergence_thre_ ) /* position convergence */
        {
          if(!orientation_)  convergence = true;
          else
            if(angle_err < rot_convergence_thre_) convergence = true;
        }

      /* transform the cartesian err to the root link frame */
      if(pos_err > pos_err_max_) pos_err = pos_err_max_;
      if(angle_err > angle_err_max_) angle_err = angle_err_max_;

      /* set err in ref frame */
      KDL::Vector rot_axis;
      tf::vectorTFToKDL(rot_err.getAxis(), rot_axis);
      KDL::Vector target_rot_err;
      if(rot_err.getAngle() > M_PI) target_rot_err = rot_axis * angle_err;
      else target_rot_err = -rot_axis * angle_err;

      if(free_axis_mask_(MaskAxis::ROT_X) == 0)
        {
          KDL::Vector err_vector = pose_err.M.UnitX() * (KDL::Vector(1, 0, 0));
          if(err_vector.Norm() < 1e-6) target_rot_err = KDL::Vector::Zero();
          else target_rot_err = err_vector / err_vector.Norm()  * angle_err;
        }
      if(free_axis_mask_(MaskAxis::ROT_Y) == 0)
        {
          KDL::Vector err_vector = pose_err.M.UnitY() * (KDL::Vector(0, 1, 0));
          if(err_vector.Norm() < 1e-6) target_rot_err = KDL::Vector::Zero();
          else target_rot_err = err_vector / err_vector.Norm() * angle_err;
        }
      if(free_axis_mask_(MaskAxis::ROT_Z) == 0)
        {
          KDL::Vector err_vector = pose_err.M.UnitZ() * (KDL::Vector(0, 0, 1));
          if(err_vector.Norm() < 1e-6) target_rot_err = KDL::Vector::Zero();
          else target_rot_err = err_vector / err_vector.Norm() * angle_err;
        }

      Eigen::VectorXd delta_cartesian = Eigen::VectorXd::Zero(6);
      pose_err.p.Normalize();
      delta_cartesian.head(3) = aerial_robot_model::kdlToEigen(-pose_err.p * pos_err);
      delta_cartesian.tail(3) = aerial_robot_model::kdlToEigen(target_rot_err);

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
      H = jacobian.transpose() * W_cartesian_err_constraint_ * jacobian;
      /* CAUTION: becuase of QP-OASES, the scale "2" is included inside the function */
      g = - delta_cartesian.transpose()  * W_cartesian_err_constraint_ * jacobian;

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
      const auto robot_model = planner_->getRobotModelPtr();
      const auto joint_positions = planner_->getTargetJointVector<KDL::JntArray>();
      jacobian = robot_model->getJacobian(joint_positions, parent_link_, reference_frame_.p);

   
   
      if(!full_body_) jacobian.leftCols(6) = Eigen::MatrixXd::Zero(jacobian.rows(), 6);

      /* change to reference frame */
      Eigen::Matrix3d inv_rot = aerial_robot_model::kdlToEigen(target_frame_.M.Inverse());
      jacobian.topRows(3) =  inv_rot * jacobian.topRows(3);
      jacobian.bottomRows(3) = inv_rot * jacobian.bottomRows(3);

      /* apply free axis mask */
      jacobian = free_axis_mask_.asDiagonal() * jacobian;

      if(planner_->getMultilinkType() == motion_type::SE2)
        {
          if(!orientation_)
            jacobian.middleRows(2, 4) = Eigen::MatrixXd::Zero(4, jacobian.cols());
          else
            jacobian.middleRows(2, 3) = Eigen::MatrixXd::Zero(3, jacobian.cols());
        }
      else
        {
          if(!orientation_)
            jacobian.middleRows(3, 3) = Eigen::MatrixXd::Zero(3, jacobian.cols());
        }

      if(debug) std::cout << "jacobian: \n" << jacobian << std::endl;
      return true;
    }
  };
};

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(differential_kinematics::cost::CartersianConstraint, differential_kinematics::cost::Base);
