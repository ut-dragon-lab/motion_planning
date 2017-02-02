#include <hydrus_object_transportation/posture_optimization.h>

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
  nhp_.param("extra_module_num", extra_module_num_, 2);
  nhp_.param("use_initial_theta", use_initial_theta_, false);
  extra_module_link_num_.resize(extra_module_num_);
  for (int i = 0; i < extra_module_num_; i++) {
    std::string extra_module_link_num_param_name("extra_module_link_num");
    nhp_.param(extra_module_link_num_param_name + boost::to_string(i), extra_module_link_num_.at(i), 0);
  }
  extra_module_mass_.resize(extra_module_num_);
  for (int i = 0; i < extra_module_num_; i++) {
    std::string extra_module_mass_param_name("extra_module_mass");
    nhp_.param(extra_module_mass_param_name + boost::to_string(i), extra_module_mass_.at(i), 0.5);
  }
  extra_module_offset_.resize(extra_module_num_);
  for (int i = 0; i < extra_module_num_; i++) {
    std::string extra_module_offset_param_name("extra_module_offset");
    nhp_.param(extra_module_offset_param_name + boost::to_string(i), extra_module_offset_.at(i), 0.3);
  }
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

void PostureOptimization::addExtraModule(bool reset, double extra_module_link_num, double extra_module_mass, double extra_module_offset)
{
  hydrus_transform_control::AddExtraModule srv;
  srv.request.reset = reset;
  srv.request.extra_module_link = extra_module_link_num;
  srv.request.extra_module_mass = extra_module_mass;
  srv.request.extra_module_offset = extra_module_offset;
  add_extra_module_client_.call(srv);
}

void PostureOptimization::check()
{
  //std::vector<double> theta;
  //theta.resize(5);
#if 0
  theta.at(0) = 1.04;
  theta.at(1) = 1.04;
  theta.at(2) = 1.04;
  theta.at(3) = 1.04;
  theta.at(4) = 1.04;
#endif
#if 0
  theta.at(0) = 1.465299;
  theta.at(1) = 1.298540;
  theta.at(2) = 0.967127;
  theta.at(3) = 0.736881;
  theta.at(4) = 0.836105;
#endif

  double theta[105] =  {1.039792, -1.570796, -0.058256, 1.221489, -0.120981
		     ,1.570796, 1.043512, 0.782334, 1.295930, 1.551016
		     ,1.365085, 1.373287, 1.570796, 0.069240, -1.557727
		     ,1.566596, 1.375627, 1.504366, 0.044917, -1.561304
		     ,1.570796, 1.244642, 1.433501, 0.119196, 0.067807
		     ,-1.550700, -1.570796, -0.687237, -0.899684, -0.188285
		     ,-1.570796, -1.570796, -0.762261, -0.612901, -0.724461
		     ,-1.570796, -1.570796, -0.856542, -0.398901, -0.823072
		     ,-1.570796, -1.570796, -0.823131, -0.707165, 0.085968
		     ,-1.570796, -1.570796, -0.830098, -0.582737, -0.333849
		     ,-1.570796, -1.570796, -0.880379, -0.484007, -0.203092
		     ,1.570796, 1.570796, 0.863535, 0.546572, 0.255007
		     ,1.570796, 1.570796, 1.024079, 0.039914, 0.604622
		     ,-1.570796, -1.570796, -1.570796, 0.712283, 0.712687
		     ,1.570796, 1.570796, 0.832832, 0.627245, 1.036055
		     ,-1.570796, -1.570796, -0.850181, -0.678627, 0.047910
		     ,1.570796, 1.570796, 0.880627, 0.438982, 1.124587
		     ,-1.570796, -1.570796, -0.843197, -0.613396, -0.274928
		     ,-1.570796, -1.570796, -1.153217, -0.004353, -0.006239
		     ,-1.570796, -1.570796, -1.155142, -0.003291, -0.006085
		     ,-1.570796, -1.570796, -1.156627, -0.002813, -0.006175
  };

  double theta__[5] = {-1.570796, -1.570796, -0.843197, -0.613396, -0.274928};
  TransformController transform_controller(nh_, nhp_, false);
  for (int i = 0; i < 21; i++) {
    std::vector<double> theta_l;
    for (int j = 0; j < 5; j++) {
      theta_l.push_back(1.04);
      //theta_l.push_back(theta__[j]);
    }
    VectorXd x = getX(transform_controller, theta_l);
    ROS_INFO("%f", i*0.25);
    std::cout << x << std::endl;
    transform_controller.addExtraModule(0, 0.25, 0.3);
  }
}

void PostureOptimization::steepestDescent(std::vector<double> initial_theta, std::vector<double>& optimized_theta, double& optimized_variance)
{
  std::vector<double> theta = initial_theta;
  std::vector<double> valid_theta = initial_theta;
  std::vector<double> grad_f(link_num_ - 1);
  double last_variance = 1000000.0;
  TransformController transform_controller(nh_, nhp_, false);
  for (int i = 0; i < extra_module_num_; i++) {
  transform_controller.addExtraModule(extra_module_link_num_.at(i), extra_module_mass_.at(i), extra_module_offset_.at(i));
  }
  //ros::Rate loop_rate(20);
  ros::Time start_time = ros::Time::now();
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
      is_valid = false;
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
    double sum = 0.0;
    for (int i = 0; i < x.size(); i++) {
      sum += x(i);
    }
    double variance = x.squaredNorm() / x.size() - sum * sum / x.size() / x.size();
    if (is_valid && (last_variance >= variance)) {
      last_variance = variance;
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
  optimized_theta = valid_theta;
  optimized_variance = last_variance;
}

void PostureOptimization::process()
{
  Eigen::VectorXd x;
  //initialize
  rosParamInit();
  addExtraModule(true, 0, 0, 0);
  for (int i = 0; i < extra_module_num_; i++) {
    addExtraModule(false, extra_module_link_num_.at(i), extra_module_mass_.at(i), extra_module_offset_.at(i));
  }
  
  //parallel process
  std::vector<std::pair<std::vector<double>, double> > theta_and_variance(thread_num_);
  std::vector<std::thread> threads;
  std::vector<std::vector<double> > initial_theta(thread_num_);
  for (int i = 0; i < thread_num_; i++) {
    initial_theta.at(i).resize(link_num_ - 1);
    for (int j = 0; j < link_num_ - 1; j++) {
      if (!use_initial_theta_) {
        initial_theta.at(i).at(j) = i * 0.1;
      } else {
        initial_theta.at(i).at(j) = initial_theta_.at(j);
      }
    }
  }
  for (int i = 0; i < thread_num_; i++) {
    threads.push_back(std::thread(&PostureOptimization::steepestDescent, this, initial_theta.at(i), std::ref(theta_and_variance.at(i).first), std::ref(theta_and_variance.at(i).second)));
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
  ROS_INFO("last theta");
  for (int i = 0; i < link_num_ - 1; i++) {
    ROS_INFO("%f", optimized_theta.at(i));
  }
  ROS_INFO("last variance:%f", optimized_variance);
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
