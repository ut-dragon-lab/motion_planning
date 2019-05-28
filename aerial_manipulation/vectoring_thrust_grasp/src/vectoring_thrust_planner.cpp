#include <vectoring_thrust_grasp/vectoring_thrust_planner.h>
#include <chrono>

namespace
{
  Eigen::VectorXd grasp_torque_;
  Eigen::MatrixXd J_combined_;

  /* contactConstraint */
  Eigen::VectorXd::Index pivot_torque_index_;
  double pivot_torque_value_;
  std::vector<int> nl_torque_index_;

  /* staticConstraint */
  std::vector<int> nl_statics_index_;

  /* vectoringConstraint */
  std::vector<int> nl_force_index_;
  double max_vectoring_force_;

  /* test */
  std::vector<double> last_x_;

  double constraint_debug_t = 0;
  double cost_debug_t = 0;

  double maxGraspForce(const std::vector<double> &x, std::vector<double> &grad, void *grasp_planner)
  {
    /* x = [f, tau]^T */
    /* A = [J^T I] */
    /* cost func:
        f_grasp^2 * ||tau_normal ||^2 = x^T * A^T * A * x
    */

    Eigen::VectorXd x_vec = Eigen::Map<const Eigen::VectorXd>(x.data(), x.size());
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(J_combined_.cols() - 6, J_combined_.rows() + J_combined_.cols() - 6);
    A.leftCols(J_combined_.rows()) = J_combined_.rightCols(J_combined_.cols() - 6).transpose();
    A.rightCols(J_combined_.cols() - 6) = Eigen::MatrixXd::Identity(J_combined_.cols() - 6, J_combined_.cols() - 6);

    /* differential: 2*A^T*A*x */
    if (!grad.empty())
      {
        auto gradient_vector = 2 * A.transpose() * A * x_vec;
        assert(grad.size() == gradient_vector.size());
        for(size_t i = 0; i < grad.size(); i++)
          grad.at(i) = gradient_vector[i];
      }

    if (ros::Time::now().toSec() - cost_debug_t > 1.0) // 1 ms
      {
        std::cout << "cost : " << x_vec.transpose() * A.transpose() * A * x_vec << std::endl;
        cost_debug_t = ros::Time::now().toSec();
      }

    last_x_ = x;

    return x_vec.transpose() * A.transpose() * A * x_vec;
  }

  /* the equality result should be zero */
  double contactConstraint(const std::vector<double> &x, std::vector<double> &grad, void *index_ptr)
  {
    /* the contact constraint:
       f_grasp * tau_normal =  J^T f + tau =  A x
     */

    //std::cout << "contactConstraint 1" << std::endl;

    int index =  *(reinterpret_cast<int*>(index_ptr));
    Eigen::VectorXd x_vec = Eigen::Map<const Eigen::VectorXd>(x.data(), x.size());
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(J_combined_.cols() - 6, J_combined_.rows() + J_combined_.cols() - 6);
    A.leftCols(J_combined_.rows()) = J_combined_.rightCols(J_combined_.cols() - 6).transpose();
    A.rightCols(J_combined_.cols() - 6) = Eigen::MatrixXd::Identity(J_combined_.cols() - 6, J_combined_.cols() - 6);

    double constant_ratio = (A.row(pivot_torque_index_) * x_vec / grasp_torque_(pivot_torque_index_))(0,0);
    /* A.rows(index) * x_vec = constant_ratio * grasp_torque_(index) */
    /* A.rows(index) * x_vec - constant_ratio * grasp_torque_(index) = 0 */

    if (!grad.empty())
      {
        assert(grad.size() == x.size());

        for(size_t i = 0; i < grad.size(); i++)
          grad.at(i) = A.row(index)[i];
      }
    return (A.row(index) * x_vec)(0,0) - constant_ratio * grasp_torque_(index);
  }

  /* the equality result should be zero */
  double staticConstraint(const std::vector<double> &x, std::vector<double> &grad, void *index_ptr)
  {
    int index = *(reinterpret_cast<int*>(index_ptr));

    /* b_i * x = 0 */
    Eigen::VectorXd x_vec = Eigen::Map<const Eigen::VectorXd>(x.data(), x.size());
    Eigen::VectorXd b_i_vector = J_combined_.col(index);

    if (!grad.empty())
      {
        grad.resize(x.size(), 0);

        for(size_t i = 0; i < b_i_vector.size(); i++)
          grad.at(i) = b_i_vector(i);
      }

    if (ros::Time::now().toSec() - constraint_debug_t > 1.0) // 1 ms
      {
        std::cout << "statics constraint: \n " << J_combined_.leftCols(6).transpose() * x_vec.head(b_i_vector.size()) << std::endl;
        std::cout << "x: " << x_vec.transpose() << std::endl;
        constraint_debug_t = ros::Time::now().toSec();
      }

    return b_i_vector.dot(x_vec.head(b_i_vector.size()));
  }

  /* the vectoring force norm should be smaller than the max value */
  double vectoringConstraint(const std::vector<double> &x, std::vector<double> &grad, void *index_ptr)
  {
    int index =  *(reinterpret_cast<int*>(index_ptr));

    if (!grad.empty())
      {
        grad.resize(x.size(), 0);
        grad.at(index * 3) = 2 * (x.at(index * 3) + x.at(index * 3 + 1));
        grad.at(index * 3 + 1) = 2 * (x.at(index * 3) + x.at(index * 3 + 1));
      }
    return std::pow(x.at(index * 3), 2) + std::pow(x.at(index * 3 + 1), 2) - std::pow(max_vectoring_force_, 2);
  }

};

GraspVectoringThrust::GraspVectoringThrust(ros::NodeHandle nh, ros::NodeHandle nhp, boost::shared_ptr<DragonRobotModel> robot_model_ptr, bool realtime_control): nh_(nh), nhp_(nhp), robot_model_ptr_(robot_model_ptr), realtime_control_(realtime_control)
{
  nhp_.param("verbose", verbose_, false);
  bool test;
  nhp_.param("test", test, false);

  if(test)
    {
      sensor_msgs::JointState joint_angles;
      if(!jointAnglesForQuadDragon(joint_angles)) return;

      calculateVectoringForce(joint_angles, true);

      return;
    }

  joint_state_sub_ = nh_.subscribe("joint_states", 1, &GraspVectoringThrust::jointStatesCallback, this);
  vectoring_force_pub_ = nh_.advertise<dragon::GraspVectoringForce>("/grasp_vectoring_force", 1);
}

void GraspVectoringThrust::jointStatesCallback(const sensor_msgs::JointStateConstPtr& state)
{
  current_joint_states_ = *state;
  robot_model_ptr_->updateRobotModel(*state);

  if(!realtime_control_) return;

  auto joint_angles = robot_model_ptr_->getGimbalProcessedJoint<sensor_msgs::JointState>();

  if(!calculateVectoringForce(joint_angles)) return;

  dragon::GraspVectoringForce msg;
  msg.clear_flag = false;
  for(int i = 0; i < vectoring_f_vector_root_.size(); i++)
    msg.grasp_vectoring_force.push_back(vectoring_f_vector_cog_[i]);

  vectoring_force_pub_.publish(msg);
}

bool GraspVectoringThrust::jointAnglesForQuadDragon(sensor_msgs::JointState& joint_angles)
{
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
  if(robot_model_ptr_->getRotorNum() != 4)
    {
      ROS_ERROR("this robot is not dragon quad type");
      return false;
    }

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

  /* hard-coding */
  joint_angles.name.push_back(std::string("joint1_yaw"));
  joint_angles.name.push_back(std::string("joint2_yaw"));
  joint_angles.name.push_back(std::string("joint3_yaw"));
  joint_angles.position.push_back(j1);
  joint_angles.position.push_back(j2);
  joint_angles.position.push_back(j3);

  seg_tf_map = robot_model_ptr_->fullForwardKinematics(joint_angles);
  if(verbose_) std::cout << "the grasp distance vs the end grasp: [" << grasp_two_end_distance <<  " vs " << (seg_tf_map.at("tail_ball").p - seg_tf_map.at("head_ball").p).Norm() << "]" << std::endl;

  return true;
}

bool GraspVectoringThrust::calculateVectoringForce(const sensor_msgs::JointState& joint_angles, bool calculate_maximum_force)
{
  /* 0. update the robot kinematics */
  auto seg_tf_map = robot_model_ptr_->fullForwardKinematics(joint_angles);

  /* 1. calcualte the joint torque to balance the grasping force */
  /** 1.1 calcualte the jacobian **/
  auto tail_end_effector_jacobian =  getJacobian(std::string("root"), std::string("tail_ball"), joint_angles, false); // not full body
  if(verbose_) std::cout << "tail_end_effector_jacobian: \n" << tail_end_effector_jacobian << std::endl;

  /** 1.2. calculate the force direction in the first contact point (link1) **/
  double grasp_force_norm;
  nhp_.param("grasp_force_norm", grasp_force_norm, 5.0);
  Eigen::Vector3d  grasp_force_on_tail_ball = (aerial_robot_model::kdlToEigen(seg_tf_map.at("tail_ball").p - seg_tf_map.at("head_ball").p)).normalized() * grasp_force_norm;

  /** 1.3. calculate the torque based on the virutal work principle: tau = -J_ext^T * f_ext   **/
  grasp_torque_ = - tail_end_effector_jacobian.topRows(3).transpose() * grasp_force_on_tail_ball;
  if(verbose_) std::cout << "grasp torque: \n" << grasp_torque_.transpose() << std::endl;

  /* 2. calculate the vectoring force need for grasping without torque */
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
  if(verbose_) std::cout << "virtual extneded grasp torque: \n" << virtual_extended_grasp_torque.transpose() << std::endl;

  J_combined_ = Eigen::MatrixXd::Zero(3 * robot_model_ptr_->getRotorNum(), virtual_extended_grasp_torque.size());

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
      J_combined_.block(3 * i, 0, 3 , J_vec_i.cols()) = J_vec_i.topRows(3);
    }

  /*
  J_combined_ = Eigen::MatrixXd::Zero(3 * 2, virtual_extended_grasp_torque.size());
  auto J_vec_i = getJacobian(std::string("root"), std::string("gimbal1_roll_module"), joint_angles, true);
  J_combined_.block(0, 0, 3 , J_vec_i.cols()) = J_vec_i.topRows(3);
  J_vec_i = getJacobian(std::string("root"), std::string("gimbal4_roll_module"), joint_angles, true);
  J_combined_.block(3, 0, 3 , J_vec_i.cols()) = J_vec_i.topRows(3);
  */

  if(verbose_) std::cout << "J_combined_: \n" << J_combined_ << std::endl;

  /** lagrange mothod **/
  Eigen::FullPivLU<Eigen::MatrixXd> lu_solver(J_combined_.transpose() * J_combined_);
  vectoring_f_vector_root_ = J_combined_ * lu_solver.solve(virtual_extended_grasp_torque);
  if(verbose_) 
    {
      std::cout << "psuedo-inverse: vectoring_f_vector: \n" << vectoring_f_vector_root_.transpose() << std::endl;
      std::cout << "psuedo-inverse: torque: \n" << J_combined_.transpose() * vectoring_f_vector_root_ << std::endl;
    }

  /** 3. qp solving to minimize the joint torque **/
  OsqpEigen::Solver qp_solver;
  qp_solver.settings()->setVerbosity(false);
  qp_solver.settings()->setWarmStart(true);

  /*** set the initial data of the QP solver ***/
  qp_solver.data()->setNumberOfVariables(3 * robot_model_ptr_->getRotorNum());
  qp_solver.data()->setNumberOfConstraints(6);

  /* cost function:
     f^T W_f f + (J' f + t_ext)^T W_t (J' f + t_ext)      -------------- J' = -J_combined_.block()^T
    = f^T (W_f + J'^T W_t J') f + 2 t_ext^T W_t J' f + ....(constant offset)
  */
  double weight_torque, weight_force;
  nhp_.param("weight_torque", weight_torque, 10.0);
  nhp_.param("weight_force", weight_force, 1.0);
  Eigen::MatrixXd J_dash = - J_combined_.rightCols(J_combined_.cols() - 6).transpose();
  Eigen::MatrixXd hessian = weight_force * Eigen::MatrixXd::Identity(3 * robot_model_ptr_->getRotorNum(), 3 * robot_model_ptr_->getRotorNum()) + J_dash.transpose() * weight_torque * J_dash;
  Eigen::SparseMatrix<double> hessian_sparse = hessian.sparseView();
  qp_solver.data()->setHessianMatrix(hessian_sparse);

  Eigen::VectorXd gradient = grasp_torque_.transpose() * weight_torque * J_dash;
  qp_solver.data()->setGradient(gradient);

  /* equality constraint: total force and  moment by vectoring thrust is zero  */
  Eigen::SparseMatrix<double> constraint_sparse = (J_combined_.leftCols(6).transpose()).sparseView();
  qp_solver.data()->setLinearConstraintsMatrix(constraint_sparse);
  Eigen::VectorXd lb = Eigen::VectorXd::Zero(6);
  qp_solver.data()->setLowerBound(lb);
  qp_solver.data()->setUpperBound(lb);

  if(!qp_solver.initSolver())
    {
      ROS_ERROR("can not initialize qp solver");
      return false;
    }

  if(!qp_solver.solve())
    {
      ROS_ERROR("can not solve QP");
      return false;
    }

  vectoring_f_vector_root_  = qp_solver.getSolution();
  if(verbose_)
    {
      std::cout << "qp: vectoring_f_vector: \n" << vectoring_f_vector_root_.transpose() << std::endl;
      std::cout << "qp: joint torque: \n" << -J_combined_.transpose() * vectoring_f_vector_root_ + virtual_extended_grasp_torque << std::endl;
    }

  /* convert to cog frame */
  auto cog_rotation = robot_model_ptr_->getCog<Eigen::Affine3d>().rotation().inverse();
  vectoring_f_vector_cog_ = Eigen::VectorXd::Zero(3 * robot_model_ptr_->getRotorNum());
  for(int i = 0; i < robot_model_ptr_->getRotorNum(); i++)
    vectoring_f_vector_cog_.segment(i * 3, 3) = cog_rotation * vectoring_f_vector_root_.segment(i * 3, 3);

    if(!calculate_maximum_force) return true;

  /** 4. nlopt solving to maximize the grasping force **/
  auto algorithm_type = nlopt::GN_ISRES; // nlopt::LD_SLSQP
  nlopt::opt nl_solver(algorithm_type, 3 * robot_model_ptr_->getRotorNum() + grasp_torque_.size());
  nl_solver.set_max_objective(maxGraspForce, this);

  /* contact (direction) constraint */
  if(fabs(grasp_torque_.maxCoeff()) > fabs(grasp_torque_.minCoeff()))
    pivot_torque_value_ = grasp_torque_.maxCoeff(&pivot_torque_index_);
  else
    pivot_torque_value_ = grasp_torque_.minCoeff(&pivot_torque_index_);

  nl_torque_index_.reserve(grasp_torque_.size());
  for(int i = 0; i < grasp_torque_.size(); i++)
    {
      if(i == pivot_torque_value_) continue;

      nl_torque_index_.push_back(i);
      nl_solver.add_equality_constraint(contactConstraint, nl_torque_index_.data() + i, 1e-8);
    }

  /* static constraint */
  nl_statics_index_.reserve(6); // important!!
  for(int i = 0; i < 6; i++)
    {
      nl_statics_index_.push_back(i);
      nl_solver.add_equality_constraint(staticConstraint, &nl_statics_index_[i], 1e-8);
    }

  /* vectoring force */
  nhp_.param("max_vectoring_force", max_vectoring_force_, 5.0);
  nl_force_index_.reserve(robot_model_ptr_->getRotorNum());
  for(int i = 0; i < robot_model_ptr_->getRotorNum(); i++)
    {
      nl_force_index_.push_back(i);
      nl_solver.add_inequality_constraint(vectoringConstraint, nl_force_index_.data() + i, 1e-3);
    }
  nl_solver.set_xtol_rel(1e-3); // important!
  nl_solver.set_maxeval(2 * 1e5);

  std::vector<double> nlopt_lb(3 * robot_model_ptr_->getRotorNum() + grasp_torque_.size(), -HUGE_VAL);
  std::vector<double> nlopt_ub(3 * robot_model_ptr_->getRotorNum() + grasp_torque_.size(), HUGE_VAL);
  for(int i = 0; i < robot_model_ptr_->getRotorNum(); i++)
    {
      nlopt_lb.at(3 * i) = -max_vectoring_force_;
      nlopt_ub.at(3 * i) = max_vectoring_force_;
      nlopt_lb.at(3 * i + 1) = -max_vectoring_force_;
      nlopt_ub.at(3 * i + 1) = max_vectoring_force_;
      nlopt_lb.at(3 * i + 2) = -0.1;
      nlopt_ub.at(3 * i + 2) = 0.1;
    }
  double max_torque;
  nhp_.param("max_torque", max_torque, 7.0);

  /* get chain from root_link to tip_link */
  KDL::Chain chain;
  robot_model_ptr_->getTree().getChain(std::string("head_ball"), std::string("tail_ball"), chain);
  assert(chain.getNrOfJoints() == grasp_torque_.size());
  int j = 0;
  int offset = 3 * robot_model_ptr_->getRotorNum();
  for(auto seg : chain.segments)
    {
      if(seg.getJoint().getType() != KDL::Joint::JointType::None)
        {
          if(seg.getJoint().getName().find("joint") != std::string::npos)
            {
              nlopt_lb.at(j + offset) = -1; // Nm, fixed paramter for not active joint torque
              nlopt_ub.at(j + offset) = 1;  // Nm, fixed paramter for not active joint torque

              auto itr = std::find(joint_angles.name.begin(), joint_angles.name.end(), seg.getJoint().getName());
              if(itr != joint_angles.name.end())
                {
                  // hard-coding for the active joint of which the angle is larger than a threshold (e.g. 0.1rad)
                  if(joint_angles.position.at(std::distance(joint_angles.name.begin(), itr)) > 0.1)
                    {
                      nlopt_lb.at(j + offset) = -1e-8; // Nm, fixed paramter for not active joint torque
                      nlopt_ub.at(j + offset) = max_torque;  // Nm, fixed paramter for not active joint torque
                    }

                  if(joint_angles.position.at(std::distance(joint_angles.name.begin(), itr)) < -0.1)
                    {
                      nlopt_lb.at(j + offset) = -max_torque; // Nm, fixed paramter for not active joint torque
                      nlopt_ub.at(j + offset) = 1e-8;  // Nm, fixed paramter for not active joint torque
                    }
                }
            }
          j++;
        }
    }

  std::cout << "nlopt nlopt_lb: " << Eigen::Map<const Eigen::VectorXd>(nlopt_lb.data(), nlopt_lb.size()).transpose() << std::endl;
  std::cout << "nlopt nlopt_ub: " << Eigen::Map<const Eigen::VectorXd>(nlopt_ub.data(), nlopt_ub.size()).transpose() << std::endl;
  nl_solver.set_lower_bounds(nlopt_lb);
  nl_solver.set_upper_bounds(nlopt_ub);

  /* set initial value */
  std::vector<double> x(3 * robot_model_ptr_->getRotorNum() + grasp_torque_.size(), 0.05); /* x = [f, tau]^T */
  // use following setting based on the previous result, but the big improment
  /*
  for (int i = 0; i < 3 * robot_model_ptr_->getRotorNum(); i++)
    x.at(i) = vectoring_f_vector(i);
  for (int i = 0; i < grasp_torque_.size(); i++)
    x.at(3 * robot_model_ptr_->getRotorNum() + i) = grasp_torque_(i);
  */

  double max_f;
  Eigen::VectorXd best_x;
  try{
    auto t_start = std::chrono::high_resolution_clock::now();
    nlopt::result result = nl_solver.optimize(x, max_f);
    auto t_end = std::chrono::high_resolution_clock::now();
    std::cout << "NLOPT: : "
              << std::chrono::duration<double, std::milli>(t_end-t_start).count()
              << " [ms]" << std::endl;

    best_x = Eigen::Map<Eigen::VectorXd>(x.data(), x.size());
    // TODO: the optimiazation process does not provide the last value (i.e. best_x != last_x)
    best_x = Eigen::Map<Eigen::VectorXd>(last_x_.data(), last_x_.size());


    std::cout << "nlopt best x:" << Eigen::Map<Eigen::VectorXd>(x.data(), x.size()) << std::endl;
    std::cout << "nlopt last x:" << best_x << std::endl;

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(J_combined_.cols() - 6, J_combined_.rows() + J_combined_.cols() - 6);
    A.leftCols(J_combined_.rows()) = J_combined_.rightCols(J_combined_.cols() - 6).transpose();
    A.rightCols(J_combined_.cols() - 6) = Eigen::MatrixXd::Identity(J_combined_.cols() - 6, J_combined_.cols() - 6);

    Eigen::VectorXd torque_normal = grasp_torque_ / grasp_force_norm;
    std::cout << "nlopt maximum grasp torque: \n" << A * best_x << std::endl;
    double grasp_maximum_force = (A * best_x)[pivot_torque_index_]  / torque_normal[pivot_torque_index_];
    std::cout << "nlopt maximum grasp force: \n" << grasp_maximum_force  << std::endl;

    std::cout << "grasp torque by contact: \n" << torque_normal * grasp_maximum_force << std::endl;


    std::cout << "nlopt statics: \n" << J_combined_.leftCols(6).transpose() * best_x.head(3 * robot_model_ptr_->getRotorNum()) << std::endl;
  }
  catch(std::exception &e) {
    std::cout << "nlopt failed: " << e.what() << std::endl;
  }
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
