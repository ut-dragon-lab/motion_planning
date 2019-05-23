#include <vectoring_thrust_grasp/vectoring_thrust_test.h>

GraspVectoringThrust::GraspVectoringThrust(ros::NodeHandle nh, ros::NodeHandle nhp, boost::shared_ptr<DragonRobotModel> robot_model_ptr): nh_(nh), nhp_(nhp_), robot_model_ptr_(robot_model_ptr)
{
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
  ROS_INFO_STREAM("l1: " << l1 << "; l2: " << l2  << "; l4: " << l4 << " grasp two end distance:" << grasp_two_end_distance);

  /* calculate joint1_yaw, joint2_yaw, joint3_yaw */
  double j2 = M_PI / 2 ;  // hard-coding init angle for joint2_yaw
  double inter_length = 2 * l2 * sin(j2/2) - grasp_two_end_distance;
  double j1 =  M_PI - (acos((std::pow(l1, 2) + std::pow(inter_length, 2) - std::pow(l4,2)) / (2 * l1 * inter_length)) + (M_PI - j2) /2);
  double j3 =  M_PI - (acos((std::pow(l4, 2) + std::pow(inter_length, 2) - std::pow(l1, 2)) / (2 * l4 * inter_length)) + (M_PI - j2) /2);
  ROS_INFO("joint1_yaw, joint2_yaw, joint3_yaw: [%f, %f, %f]", j1, j2, j3);

  sensor_msgs::JointState joint_angles;
  joint_angles.name.push_back(std::string("joint1_yaw"));
  joint_angles.name.push_back(std::string("joint2_yaw"));
  joint_angles.name.push_back(std::string("joint3_yaw"));
  joint_angles.position.push_back(j1);
  joint_angles.position.push_back(j2);
  joint_angles.position.push_back(j3);

  seg_tf_map = robot_model_ptr_->fullForwardKinematics(joint_angles);
  ROS_INFO("the grasp distance vs the end grasp : [%f vs %f]", grasp_two_end_distance, (seg_tf_map.at("tail_ball").p - seg_tf_map.at("head_ball").p).Norm());

  /* 2. calcualte the joint torque to balance the grasping force */
  /** 2.1 calcualte the jacobian **/
  auto tail_end_effector_jacobian =  getJacobian(std::string("root"), std::string("tail_ball"), joint_angles, false); // not full body
  std::cout << "tail_end_effector_jacobian: \n" << tail_end_effector_jacobian << std::endl;

  /** 2.2. calculate the force direction in the first contact point (link1) **/
  double grasp_force_norm;
  nhp_.param("grasp_force_norm", grasp_force_norm, 5.0);
  Eigen::Vector3d  grasp_force_on_tail_ball = (aerial_robot_model::kdlToEigen(seg_tf_map.at("tail_ball").p - seg_tf_map.at("head_ball").p)).normalized() * grasp_force_norm;

  /** 2.3. calculate the torque based on the virutal work principle: tau = -J_ext^T * f_ext   **/
  Eigen::VectorXd grasp_torque = - tail_end_effector_jacobian.topRows(3).transpose() * grasp_force_on_tail_ball;
  std::cout << "grasp torque: \n" << grasp_torque.transpose() << std::endl;

#if 0
  /** 2.4 check the external force on head ball **/
  head_end_effector_jacobian =  getJacobian(std::string("tail_ball"), std::string("head_ball"), joint_angles, false); // not full body
  Eigen::Vector3d  grasp_force_on_head_ball = - (head_end_effector_jacobian.tranpose()).inverse() * grasp_torque;
#endif

  /* 3. calculate the vectoring force need for grasping without torque */
  /* virtual work priciple for multilink: tau = 0 = -J'_ext_head^T * f_ext_head -J'_ext_tail^T * f_ext_tail - J'_vec1^T * f_vec1 - J'_vec2^T * f_vec2 - .... - J'_vecN^T * f_vecN */
  /* J': extended jacobian with root link, R^{6 x (6 + N)} */
  /* force balance: f_vec1 + ... + f_vecN = 0 */
  /* moment balance: M [f_vec1 + ... + f_vecN] = 0 */

  auto J_ext_head = getJacobian(std::string("root"), std::string("head_ball"), joint_angles, true);
  auto J_ext_tail = getJacobian(std::string("root"), std::string("tail_ball"), joint_angles, true);

  std::cout << "J ext head: \n" << J_ext_head << std::endl;
  std::cout << "J ext tail: \n" << J_ext_tail << std::endl;

  Eigen::VectorXd virtual_extended_grasp_torque =  - J_ext_tail.topRows(3).transpose() * grasp_force_on_tail_ball;
  virtual_extended_grasp_torque.head(J_ext_head.cols()) += (- J_ext_head.topRows(3).transpose() * (-grasp_force_on_tail_ball));
  std::cout << "virtual extneded grasp torque: \n" << virtual_extended_grasp_torque.transpose() << std::endl;

  Eigen::MatrixXd J_combined = Eigen::MatrixXd::Zero(3 * robot_model_ptr_->getRotorNum(), virtual_extended_grasp_torque.size());
  for(int i = 0; i < robot_model_ptr_->getRotorNum(); i++)
    {
      //auto J_vec_i = getJacobian(std::string("root"), std::string("thrust") + std::to_string(i+1), joint_angles, true);
      auto J_vec_i = getJacobian(std::string("root"), std::string("gimbal") + std::to_string(i+1) + std::string("_roll_module"), joint_angles, true);
      std::cout << "extended jacobian for " << i+1 << "th rotor : \n" << J_vec_i << std::endl;
      J_combined.block(3 * i, 0, 3 , J_vec_i.cols()) = J_vec_i.topRows(3);
    }

  /*
  J_combined = Eigen::MatrixXd::Zero(3 * 2, virtual_extended_grasp_torque.size());
  auto J_vec_i = getJacobian(std::string("root"), std::string("gimbal1_roll_module"), joint_angles, true);
  J_combined.block(0, 0, 3 , J_vec_i.cols()) = J_vec_i.topRows(3);
  J_vec_i = getJacobian(std::string("root"), std::string("gimbal4_roll_module"), joint_angles, true);
  J_combined.block(3, 0, 3 , J_vec_i.cols()) = J_vec_i.topRows(3);
  */

  std::cout << "J_combined: \n" << J_combined << std::endl;

  /** lagrange mothod **/
  Eigen::FullPivLU<Eigen::MatrixXd> solver(J_combined.transpose() * J_combined);
  Eigen::VectorXd vectoring_f_vector = J_combined * solver.solve(virtual_extended_grasp_torque);

  std::cout << "vectoring_f_vector: \n" << vectoring_f_vector.transpose() << std::endl;

  std::cout << "torque: \n" << J_combined.transpose() * vectoring_f_vector << std::endl;
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
              std::cout << "add " << seg.getJoint().getName() << " to whitelist" << std::endl;
            }
          else
            {
              //ROS_WARN("%s", seg.getJoint().getName().c_str());
            }
          j++;
        }
    }

  std::cout << "joint array from " << root_link << " to " << tip_link << ": \n " << joint_positions.data.transpose() << std::endl;

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

      ROS_INFO("end pose for full body: [%f %f %f]", end_frame.p.x(), end_frame.p.y(), end_frame.p.z());

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
