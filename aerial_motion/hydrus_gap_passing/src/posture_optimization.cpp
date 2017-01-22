#include <hydrus_gap_passing/motion_control.h>
#include <hydrus_gap_passing/posture_optimization.h>

PostureOptimization::PostureOptimization(ros::NodeHandle nh, ros::NodeHandle nhp) :nh_(nh), nhp_(nhp)
{
  transform_controller_ = boost::shared_ptr<TransformController>(new TransformController(nh_, nhp_, false));
  
  link_num_ = transform_controller_->getLinkNum();
  ring_radius_ = 0.01+transform_controller_->getRingRadius();
  theta_.resize(link_num_ - 1);
  grad_f_.resize(link_num_);
  joint_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);
  add_extra_module_client_ = nh_.serviceClient<hydrus_transform_control::AddExtraModule>("add_extra_module");
  valid_theta_.resize(link_num_ - 1);
  last_variance_ = 10000000.0;
}


PostureOptimization::~PostureOptimization()
{
}

void PostureOptimization::rosParamInit()
{
  nhp_.param("alpha", alpha_, 0.1);
  nhp_.param("d_theta", d_theta_, 0.01);
  nhp_.param("grad_f_thresh", grad_f_thresh_, 0.1);
  nhp_.param("v_thresh", v_thresh_, 0.001);
  nhp_.param("time_thresh", time_thresh_, 10.0);
  nhp_.param("thread_num", thread_num_, 1);
  nhp_.param("extra_module_link_num_1", extra_module_link_num_1_, 0);
  nhp_.param("extra_module_link_num_2", extra_module_link_num_2_, 0);
  nhp_.param("extra_module_mass", extra_module_mass_, 0.5);
  nhp_.param("extra_module_offset", extra_module_offset_, 0.0);
  nhp_.param("linkend_radius", linkend_radius_, 0.02);
  initial_theta_.resize(link_num_ - 1);
  for (int i = 0; i < link_num_ - 1; i++) {
    std::string initial_theta_param_name("initial_theta");
    nhp_.param(initial_theta_param_name + boost::to_string(i), initial_theta_[i], 0.0);
  }
}

Eigen::VectorXd PostureOptimization::getX(TransformController& transform_controller, std::vector<double> theta) 
{
  Eigen::VectorXd g(4);
  g << 0, 0, 0, 9.8;
  Eigen::VectorXd x;
  transform_controller.distThreCheckFromJointValues(theta);
  transform_controller.stabilityCheck();
  Eigen::MatrixXd U = transform_controller.getU();
  Eigen::FullPivLU<Eigen::MatrixXd> solver((U * U.transpose())); 
  Eigen::VectorXd lamda;
  lamda = solver.solve(g);
  return U.transpose() * lamda;
}

bool PostureOptimization::collisionCheck(TransformController& transform_controller, std::vector<double> theta)
{
  transform_controller.distThreCheckFromJointValues(theta);
  std::vector<Eigen::Vector3d> links_origin_from_cog;
  std::vector<Eigen::Vector3d> linkends_origin_from_cog;
  transform_controller.getLinksOriginFromCog(links_origin_from_cog);
  transform_controller.getLinkendsOriginFromCog(linkends_origin_from_cog);
  for (int i = 0; i < links_origin_from_cog.size() - 1; i++) {
    for (int j = i + 1; j < links_origin_from_cog.size(); j++) {
      Eigen::Vector3d diff_vec = links_origin_from_cog[i] - links_origin_from_cog[j];
      if (diff_vec.norm() < ring_radius_ * 2) return false;
    }
  }
  
  for (int i = 0; i < linkends_origin_from_cog.size() - 1; i++) {
    for (int j = i + 1; j < linkends_origin_from_cog.size(); j++) {
      Eigen::Vector3d diff_vec = linkends_origin_from_cog[i] - linkends_origin_from_cog[j];
      if (diff_vec.norm() < linkend_radius_ * 2) return false;
    }
  }
  
  for (int i = 0; i < links_origin_from_cog.size(); i++) {
    for (int j = 0; j < linkends_origin_from_cog.size(); j++) {
      Eigen::Vector3d diff_vec = links_origin_from_cog[i] - linkends_origin_from_cog[j];
      if (diff_vec.norm() < linkend_radius_ + ring_radius_) return false;
    }
  }
  return true;
}

void PostureOptimization::steepestDescent(std::vector<double> initial_theta, std::vector<double>& optimized_theta, double& optimized_variance)
{
  std::vector<double> theta = initial_theta;
  std::vector<double> valid_theta = initial_theta;
  std::vector<double> grad_f(link_num_ - 1);
  double last_variance = 1000000.0;
  TransformController transform_controller(nh_, nhp_, false);
  transform_controller.addExtraModule(extra_module_link_num_1_, extra_module_mass_, extra_module_offset_);
  transform_controller.addExtraModule(extra_module_link_num_2_, extra_module_mass_, extra_module_offset_);
  //ros::Rate loop_rate(20);
  ros::Time start_time = ros::Time::now();
  ROS_ERROR("link_num:%d", link_num_);
  {
    VectorXd x = getX(transform_controller, theta);
    double sum = 0.0;
    for (int i = 0; i < x.size(); i++) {
      sum += x(i);
    }
    double variance = x.squaredNorm() / x.size() - sum * sum / x.size() / x.size();
    
    std::cout << "initial:" <<  variance << std::endl;
  }
  
  while (true) {
    double current_f = getX(transform_controller, theta).norm();
    //calc gradient
    for (unsigned int i = 0; i < theta.size(); i++) {
      std::vector<double> tmp_theta = theta;
      tmp_theta[i] += d_theta_;
      grad_f[i] = (getX(transform_controller, tmp_theta).norm() - current_f) / d_theta_;
    }

    //update
    for (unsigned i = 0; i < theta.size(); i++) {
      theta[i] = std::max(-M_PI/2, std::min(theta[i] - alpha_ * grad_f[i], M_PI/2));
    }

    bool break_flag = false;
    bool is_valid = true;

    //validity check
    if (!transform_controller.distThreCheckFromJointValues(theta)) {
      //ROS_WARN("dist is too small");
      is_valid = false;
    }
    
    if (!transform_controller.stabilityCheck()) {
      //ROS_WARN("not stable");
      is_valid = false;
    }
    
    if (!collisionCheck(transform_controller, theta)) {
      //ROS_WARN("collision");
      //is_valid = false;
    }

    VectorXd x = getX(transform_controller, theta);
    for (int i = 0; i < link_num_; i++) {
      if (x(i) < 0) {
        is_valid = false;
        //ROS_WARN("invalid U");
        break;
      }
    }

    //ROS_INFO("theta: %f, %f, %f, %f, %f value: %f", theta_[0], theta_[1], theta_[2], theta_[3], theta_[4], x.norm());
    double max = 0.0, sum = 0.0;
    for (int i = 0; i < x.size(); i++) {
        if (max < x(i)) {
          max = x(i);
        }
        sum += x(i);
    }
    double variance = x.squaredNorm() / x.size() - sum * sum / x.size() / x.size();
    //ROS_INFO("max force:%f, variance:%f", max, variance);
    //ROS_ERROR("theta:%f, %f, %f, %f, %f", theta_[0], theta_[1], theta_[2], theta_[3], theta_[4]);
    //std::cout << variance << std::endl;
    if (is_valid && (last_variance >= variance)) {
      last_variance = variance;
      //ROS_WARN("last variance:%f", last_variance);
      valid_theta = theta;
    }

    //break condition
    if (last_variance < v_thresh_) {
      break_flag = true; //force break
    }
  
    if (ros::Time::now().toSec() - start_time.toSec() > time_thresh_) {
      break_flag = true; //force break
    }
    /*
    sensor_msgs::JointState joint_state;
    joint_state.header.stamp = ros::Time::now();
    joint_state.name.resize(link_num_ - 1);
    joint_state.position.resize(link_num_ - 1);
    for (int i = 0; i < link_num_ - 1; i++) {
      joint_state.name[i] = std::string("joint") + boost::to_string(i + 1);
      joint_state.position[i] = theta[i];
    } 
    joint_pub_.publish(joint_state);
    ros::spinOnce();
    loop_rate.sleep();
    */
    if(break_flag) break;
  }
 /* 
  sensor_msgs::JointState joint_state;
  joint_state.name.resize(link_num_ - 1);
  joint_state.position.resize(link_num_ - 1);
  for (int i = 0; i < link_num_ - 1; i++) {
    joint_state.name[i] = std::string("joint") + boost::to_string(i + 1);
    joint_state.position[i] = valid_theta_[i];
  }
  for(int i = 0; i < 10; i++) {
    joint_state.header.stamp = ros::Time::now();
    joint_pub_.publish(joint_state);
    ros::spinOnce();
    loop_rate.sleep();
  }
  */
  
  optimized_theta = valid_theta;
  optimized_variance = last_variance;
}

void PostureOptimization::process()
{
  Eigen::VectorXd x;
  //initialize
  rosParamInit();
  hydrus_transform_control::AddExtraModule srv;
  srv.request.extra_module_link = extra_module_link_num_1_;
  srv.request.extra_module_mass = extra_module_mass_;
  srv.request.extra_module_offset = extra_module_offset_;
  add_extra_module_client_.call(srv);
  srv.request.extra_module_link = extra_module_link_num_2_;
  add_extra_module_client_.call(srv);
 
  //parallel process
  std::vector<std::pair<std::vector<double>, double> > theta_and_variance(thread_num_);
  std::vector<std::thread> threads;
  std::vector<std::vector<double> > initial_theta(thread_num_);
  for (int i = 0; i < thread_num_; i++) {
    initial_theta.at(i).resize(link_num_ - 1);
    for (int j = 0; j < link_num_ - 1; j++) {
      initial_theta.at(i).at(j) = i * 0.1;
    }
  }
  initial_theta_.at(0) = 1.57;
  initial_theta_.at(1) = 1.57;
  initial_theta_.at(2) = 1.57;
  //initial_theta_.at(3) = -1.57;
  //initial_theta_.at(4) = -1.57;
  ROS_WARN("thread_num:%d", thread_num_);
  for (int i = 0; i < thread_num_; i++) {
    //threads.push_back(std::thread(&PostureOptimization::steepestDescent, this, initial_theta.at(i), std::ref(theta_and_variance.at(i).first), std::ref(theta_and_variance.at(i).second)));
    threads.push_back(std::thread(&PostureOptimization::steepestDescent, this, initial_theta_, std::ref(theta_and_variance.at(i).first), std::ref(theta_and_variance.at(i).second)));
  }
  for (int i = 0; i < threads.size(); i++) {
    if (threads.at(i).joinable())
    threads.at(i).join();
  }
  //result 
  double min_variance = theta_and_variance.at(0).second;
  int min_variance_index = 0;
  for (int i = 1; i < thread_num_; i++) {
    if (min_variance > theta_and_variance.at(i).second) {
      min_variance = theta_and_variance.at(i).second;
      min_variance_index = i;
    }
  }
  std::vector<double> optimized_theta = theta_and_variance.at(min_variance_index).first;
  double optimized_variance = min_variance;
  ROS_ERROR("last theta:%f, %f, %f", optimized_theta[0], optimized_theta[1], optimized_theta[2]);
  //ROS_ERROR("last theta:%f, %f, %f, %f, %f", optimized_theta[0], optimized_theta[1], optimized_theta[2], optimized_theta[3], optimized_theta[4]);
  ROS_ERROR("last variance:%f", optimized_variance);
  ROS_ERROR("process finished");
 
  //publish result
  ros::Rate loop_rate(20);
  sensor_msgs::JointState joint_state;
  joint_state.name.resize(link_num_ - 1);
  joint_state.position.resize(link_num_ - 1);
  for (int i = 0; i < link_num_ - 1; i++) {
    joint_state.name[i] = std::string("joint") + boost::to_string(i + 1);
    joint_state.position[i] = optimized_theta[i];
  }
  for (int i = 0; i < 20; i++) {
    joint_state.header.stamp = ros::Time::now();
    joint_pub_.publish(joint_state);
    loop_rate.sleep();
    ros::spinOnce();
  }
}
