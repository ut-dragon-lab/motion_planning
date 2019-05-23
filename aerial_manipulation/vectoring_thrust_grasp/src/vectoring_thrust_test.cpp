#include <vectoring_thrust_grasp/vectoring_thrust_test.h>

GraspVectoringThrust::GraspVectoringThrust(ros::NodeHandle nh, ros::NodeHandle nhp, boost::shared_ptr<DragonRobotModel> robot_model_ptr): nh_(nh), nhp_(nhp), robot_model_ptr_(robot_model_ptr)
{
  nhp_.param("verbose", verbose_, false);
  /* 1. calculate the joint angles for the given grasp two end distance */

  /* TODO: solve the joint angles from IK manner: set the infinitesimal motion along the radial direction from the orign point 
  const MultilinkState& end_state = ik_solver_->getPathConst().back();
  sensor_msgs::JointState joint_angles;
  for(int i = 0 ; i < robot_model_ptr_->getLinkJointNames().size(); i++)
    {
      joint_angles.name.push_back(robot_model_ptr_->getLinkJointNames().at(i));
      joint_angles.position.push_back(end_state.getActuatorStateConst()(robot_model_ptr_->getLinkJointIndex().at(i)));
      ROS_INFO_STREAM(joint_angles.name.back() << ": " <<  joint_angles.position.back()); // test
    }
  */

  /* temporally solution for quad-type */
  auto seg_tf_map = robot_model_ptr_->fullForwardKinematics(KDL::JntArray(robot_model_ptr_->getTree().getNrOfJoints()));

  /* l1: the length of link1 */
  /* l2 = l3: the length of link2, same with that of link3 */
  /* l4 = l3: the length of link4 */
  double l1 = seg_tf_map.at("link2").p.x() - seg_tf_map.at("head_ball").p.x();
  double l4 = seg_tf_map.at("tail_ball").p.x() - seg_tf_map.at("link4").p.x();
  double l2 = seg_tf_map.at("link3").p.x() - seg_tf_map.at("link2").p.x();

  /* the end-effectors are ball, so the radius is constant */
  double grasp_two_end_distance;
  nhp_.param("grasp_two_end_distance", grasp_two_end_distance, 0.1); // m
  grasp_two_end_distance += seg_tf_map.at("head_ball").p.y() + seg_tf_map.at("tail_ball").p.y(); // approximation
  if(verbose_) std::cout << "l1: " << l1 << "; l2: " << l2  << "; l4: " << l4 << " grasp two end distance:" << grasp_two_end_distance << std::endl;

  /* calculate joint1_yaw, joint2_yaw, joint3_yaw */
  double j2 = M_PI / 2 ;  // hard-coding init angle for joint2_yaw
  double inter_length = 2 * l2 * sin(j2/2) - grasp_two_end_distance;
  double j1 =  M_PI - (acos((std::pow(l1, 2) + std::pow(inter_length, 2) - std::pow(l4,2)) / (2 * l1 * inter_length)) + (M_PI - j2) /2);
  double j3 =  M_PI - (acos((std::pow(l4, 2) + std::pow(inter_length, 2) - std::pow(l1, 2)) / (2 * l4 * inter_length)) + (M_PI - j2) /2);
  std::cout << "joint1_yaw, joint2_yaw, joint3_yaw: " << j1 << ", " << j2 << ", " << j3 << std::endl;

  sensor_msgs::JointState joint_angles;
  joint_angles.name.push_back(std::string("joint1_yaw"));
  joint_angles.name.push_back(std::string("joint2_yaw"));
  joint_angles.name.push_back(std::string("joint3_yaw"));
  joint_angles.position.push_back(j1);
  joint_angles.position.push_back(j2);
  joint_angles.position.push_back(j3);

  seg_tf_map = robot_model_ptr_->fullForwardKinematics(joint_angles);
  if(verbose_) std::cout << "the grasp distance vs the end grasp: [" << grasp_two_end_distance <<  " vs " << (seg_tf_map.at("tail_ball").p - seg_tf_map.at("head_ball").p).Norm() << "]" << std::endl;

  /* 2. calcualte the joint torque to balance the grasping force */
  /** 2.1 calcualte the jacobian **/
  auto tail_end_effector_jacobian =  getJacobian(std::string("root"), std::string("tail_ball"), joint_angles, false); // not full body
  if(verbose_) std::cout << "tail_end_effector_jacobian: \n" << tail_end_effector_jacobian << std::endl;

  /** 2.2. calculate the force direction in the first contact point (link1) **/
  double grasp_force_norm;
  nhp_.param("grasp_force_norm", grasp_force_norm, 5.0);
  Eigen::Vector3d  grasp_force_on_tail_ball = (aerial_robot_model::kdlToEigen(seg_tf_map.at("tail_ball").p - seg_tf_map.at("head_ball").p)).normalized() * grasp_force_norm;

  /** 2.3. calculate the torque based on the virutal work principle: tau = -J_ext^T * f_ext   **/
  Eigen::VectorXd grasp_torque = - tail_end_effector_jacobian.topRows(3).transpose() * grasp_force_on_tail_ball;
  std::cout << "grasp torque: \n" << grasp_torque.transpose() << std::endl;

  /* 3. calculate the vectoring force need for grasping without torque */
  /* virtual work priciple for multilink: tau = 0 = -J'_ext_head^T * f_ext_head -J'_ext_tail^T * f_ext_tail - J'_vec1^T * f_vec1 - J'_vec2^T * f_vec2 - .... - J'_vecN^T * f_vecN */
  /* J': extended jacobian with root link, R^{6 x (6 + N)} */
  /* force balance: f_vec1 + ... + f_vecN = 0 */
  /* moment balance: M [f_vec1 + ... + f_vecN] = 0 */

  auto J_ext_head = getJacobian(std::string("root"), std::string("head_ball"), joint_angles, true);
  auto J_ext_tail = getJacobian(std::string("root"), std::string("tail_ball"), joint_angles, true);

  if(verbose_) std::cout << "J ext head: \n" << J_ext_head << std::endl;
  if(verbose_) std::cout << "J ext tail: \n" << J_ext_tail << std::endl;

  Eigen::VectorXd virtual_extended_grasp_torque =  - J_ext_tail.topRows(3).transpose() * grasp_force_on_tail_ball;
  virtual_extended_grasp_torque.head(J_ext_head.cols()) += (- J_ext_head.topRows(3).transpose() * (-grasp_force_on_tail_ball));
  std::cout << "virtual extneded grasp torque: \n" << virtual_extended_grasp_torque.transpose() << std::endl;

  Eigen::MatrixXd J_combined = Eigen::MatrixXd::Zero(3 * robot_model_ptr_->getRotorNum(), virtual_extended_grasp_torque.size());

  bool force_on_thrust_frame;
  nhp_.param("force_on_thrust_frame", force_on_thrust_frame, false);
  for(int i = 0; i < robot_model_ptr_->getRotorNum(); i++)
    {
      Eigen::MatrixXd J_vec_i;
      if(force_on_thrust_frame)
        J_vec_i = getJacobian(std::string("root"), std::string("thrust") + std::to_string(i+1), joint_angles, true);
      else
        J_vec_i = getJacobian(std::string("root"), std::string("gimbal") + std::to_string(i+1) + std::string("_roll_module"), joint_angles, true);
      if(verbose_) std::cout << "extended jacobian for " << i+1 << "th rotor : \n" << J_vec_i << std::endl;
      J_combined.block(3 * i, 0, 3 , J_vec_i.cols()) = J_vec_i.topRows(3);
    }

  /*
  J_combined = Eigen::MatrixXd::Zero(3 * 2, virtual_extended_grasp_torque.size());
  auto J_vec_i = getJacobian(std::string("root"), std::string("gimbal1_roll_module"), joint_angles, true);
  J_combined.block(0, 0, 3 , J_vec_i.cols()) = J_vec_i.topRows(3);
  J_vec_i = getJacobian(std::string("root"), std::string("gimbal4_roll_module"), joint_angles, true);
  J_combined.block(3, 0, 3 , J_vec_i.cols()) = J_vec_i.topRows(3);
  */

  if(verbose_) std::cout << "J_combined: \n" << J_combined << std::endl;

  /** lagrange mothod **/
  Eigen::FullPivLU<Eigen::MatrixXd> lu_solver(J_combined.transpose() * J_combined);
  Eigen::VectorXd vectoring_f_vector = J_combined * lu_solver.solve(virtual_extended_grasp_torque);
  std::cout << "psuedo-inverse: vectoring_f_vector: \n" << vectoring_f_vector.transpose() << std::endl;
  std::cout << "psuedo-inverse: torque: \n" << J_combined.transpose() * vectoring_f_vector << std::endl;

  /** qp solving to minimize the joint torque **/
  OsqpEigen::Solver qp_solver;

  // settings
  //solver.settings()->setVerbosity(false);
  qp_solver.settings()->setWarmStart(true);

  // set the initial data of the QP solver
  qp_solver.data()->setNumberOfVariables(3 * robot_model_ptr_->getRotorNum());
  qp_solver.data()->setNumberOfConstraints(6);

  /* cost function:
     f^T W_f f + (J' f + t_ext)^T W_t (J' f + t_ext)      -------------- J' = -J_combined.block()^T
    = f^T (W_f + J'^T W_t J') f + 2 t_ext^T W_t J' f + ....(constant offset)
  */
  double weight_torque, weight_force;
  nhp_.param("weight_torque", weight_torque, 10.0);
  nhp_.param("weight_force", weight_force, 1.0);
  Eigen::MatrixXd J_dash = - J_combined.rightCols(J_combined.cols() - 6).transpose();
  Eigen::MatrixXd hessian = weight_force * Eigen::MatrixXd::Identity(3 * robot_model_ptr_->getRotorNum(), 3 * robot_model_ptr_->getRotorNum()) + J_dash.transpose() * weight_torque * J_dash;
  Eigen::SparseMatrix<double> hessian_sparse = hessian.sparseView();
  qp_solver.data()->setHessianMatrix(hessian_sparse);

  Eigen::VectorXd gradient = grasp_torque.transpose() * weight_torque * J_dash;
  qp_solver.data()->setGradient(gradient);

  /* equality constraint: total force and  moment by vectoring thrust is zero  */
  Eigen::SparseMatrix<double> constraint_sparse = (J_combined.leftCols(6).transpose()).sparseView();
  qp_solver.data()->setLinearConstraintsMatrix(constraint_sparse);
  Eigen::VectorXd lb = Eigen::VectorXd::Zero(6);
  qp_solver.data()->setLowerBound(lb);
  qp_solver.data()->setUpperBound(lb);

  // instantiate the solver
  if(!qp_solver.initSolver())
    {
      ROS_ERROR("can not initialize qp solver");
      return;
    }

  // solve the QP problem
  if(!qp_solver.solve())
    {
      ROS_ERROR("can not solve QP");
      return;
    }

  // get the controller input
  vectoring_f_vector  = qp_solver.getSolution();
  std::cout << "qp: vectoring_f_vector: \n" << vectoring_f_vector.transpose() << std::endl;
  std::cout << "qp: joint torque: \n" << -J_combined.transpose() * vectoring_f_vector + virtual_extended_grasp_torque << std::endl;
}

const Eigen::MatrixXd GraspVectoringThrust::getJacobian(std::string root_link, std::string tip_link, const sensor_msgs::JointState& joint_angles, bool full_body, KDL::Segment additional_frame)
{
  /* get chain from root_link to tip_link */
  KDL::Chain chain;
  if(!robot_model_ptr_->getTree().getChain(root_link, tip_link, chain))
    {
      ROS_ERROR_STREAM("can not get proper kdl chain from " << root_link << " to " << tip_link);
    }
  if(additional_frame.getName() != "NoName") chain.addSegment(additional_frame);

  /* get jacobian */
  /* assign the joint angle */
  KDL::JntArray joint_positions(chain.getNrOfJoints());

  std::vector<int> whitelist(0);
  int j = 0;
  for(auto seg : chain.segments)
    {
      if(seg.getJoint().getType() != KDL::Joint::JointType::None)
        {
          if(seg.getJoint().getName().find("joint") != std::string::npos)
            {
              auto itr = std::find(joint_angles.name.begin(), joint_angles.name.end(), seg.getJoint().getName());
              if(itr != joint_angles.name.end())
                joint_positions(j) = joint_angles.position.at(std::distance(joint_angles.name.begin(), itr));

              whitelist.push_back(j);
              if(verbose_) std::cout << "add " << seg.getJoint().getName() << " to whitelist" << std::endl;
            }
          else
            {
              //ROS_WARN("%s", seg.getJoint().getName().c_str());
            }
          j++;
        }
    }

  if(verbose_) std::cout << "joint array from " << root_link << " to " << tip_link << ": \n " << joint_positions.data.transpose() << std::endl;

  KDL::ChainJntToJacSolver jac_solver(chain);
  KDL::Jacobian jac(chain.getNrOfJoints());

  if(jac_solver.JntToJac(joint_positions, jac) != KDL::SolverI::E_NOERROR)
    ROS_ERROR_STREAM("Can not calculate the jacobian for " << tip_link << " w.r.t. " << root_link);

  Eigen::MatrixXd final_jacobian = Eigen::MatrixXd::Zero(6, whitelist.size());
  for(size_t i = 0; i < whitelist.size(); i++)
    final_jacobian.col(i) = jac.data.col(whitelist.at(i));

  if(full_body)
    {
      Eigen::MatrixXd jacobian_root_link = Eigen::MatrixXd::Zero(6, final_jacobian.cols() + 6);

      KDL::Frame end_frame;
      KDL::ChainFkSolverPos_recursive fk_solver(chain);
      fk_solver.JntToCart(joint_positions, end_frame);

      if(verbose_) std::cout << "end pose for full body: [" << end_frame.p.x() << ", " << end_frame.p.y() << ", " << end_frame.p.z() << "]" << std::endl;

      if(final_jacobian.cols() > 0)
        jacobian_root_link.block(0, 6, final_jacobian.rows(), final_jacobian.cols()) = final_jacobian;

      /* root link */
      jacobian_root_link.block(0, 0, 6, 6) = Eigen::MatrixXd::Identity(6, 6);
      jacobian_root_link.block(0, 3, 3, 1) = (Eigen::Vector3d(1, 0, 0)).cross(Eigen::Vector3d(end_frame.p.data));
      jacobian_root_link.block(0, 4, 3, 1) = (Eigen::Vector3d(0, 1, 0)).cross(Eigen::Vector3d(end_frame.p.data));
      jacobian_root_link.block(0, 5, 3, 1) = (Eigen::Vector3d(0, 0, 1)).cross(Eigen::Vector3d(end_frame.p.data));


      return jacobian_root_link;
    }

  return final_jacobian;
}
