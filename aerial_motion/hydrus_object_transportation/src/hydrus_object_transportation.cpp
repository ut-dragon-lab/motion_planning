#include <hydrus_object_transportation/hydrus_object_transportation.h>

//#define GRIPPER_INPUT_DEBUG
//#define GRIPPER_OUTPUT_DEBUG
//#define CAMERA_POS_DEBUG
//#define GRIPPER_POS_DEBUG
//#define OBJECT_POS_DEBUG
//#define POS_DEBUG

HydrusObjectTransportation::HydrusObjectTransportation(ros::NodeHandle nh, ros::NodeHandle nhp) :nh_(nh), nhp_(nhp)
{
  //topic and service name
  nhp_.param("joint_pub_topic_name", joint_pub_topic_name_, std::string("hydrus/joints_ctrl"));
  nhp_.param("add_extra_module_service_name", add_extra_module_service_name_, std::string("add_extra_module"));
  nhp_.param("gripper0_control_service_name", gripper_control_service_name_[0], std::string("/magnet0/serial_board/magnet_control"));
  nhp_.param("gripper1_control_service_name", gripper_control_service_name_[1], std::string("/magnet1/serial_board/magnet_control"));
  nhp_.param("gripper0_sub_topic_name", gripper_state_sub_topic_name_[0], std::string("/magnet0/serial_board/magnet_feedback"));
  nhp_.param("gripper1_sub_topic_name", gripper_state_sub_topic_name_[1], std::string("/magnet1/serial_board/magnet_feedback"));
  nhp_.param("odom_sub_topic_name", odom_sub_topic_name_, std::string("/uav/full_state"));
  nhp_.param("target_image_center_topic_name", target_image_center_topic_name_, std::string("/object_image_center"));
  nhp_.param("uav_pos_pub_topic_name", uav_pos_pub_topic_name_, std::string("/uav/nav"));
  nhp_.param("camera_info_sub_topic_name", camera_info_sub_topic_name_, std::string("/camera/fisheye_gimbal/camera_info"));
  nhp_.param("state_machine_pub", state_machine_pub_topic_name_, std::string("state_machine"));
  nhp_.param("joy_stick_sub_topic_name", joy_stick_sub_topic_name_, std::string("/joy"));
  nhp_.param("debug_sub_topic_name", debug_sub_topic_name_, std::string("/device_debug"));
  nhp_.param("object_world_pos_pub_topic_name", object_world_pos_pub_topic_name_, std::string("/object_world_pos"));
  nhp_.param("yaw_control_pub_topic_name", yaw_control_pub_topic_name_, std::string("/yaw_control_flag"));
  //param
  nhp_.param("control_frequency", control_frequency_, 20);
  nhp_.param("link_length", link_length_, 0.60);
  nhp_.param("gripper0_link_num", gripper_link_num_[0], 1); //zero index
  nhp_.param("gripper1_link_num", gripper_link_num_[1], 3); //zero index
  nhp_.param("gripper0_link_offset", gripper_link_offset_[0], 0.30);
  nhp_.param("gripper1_link_offset", gripper_link_offset_[1], 0.30);
  nhp_.param("camera_link_num", camera_link_num_, 3);
  nhp_.param("object_num", object_num_, 2);
  nhp_.param("link_num", link_num_, 4);
  nhp_.param("object_pos_thresh", object_pos_thresh_, 0.05);
  nhp_.param("go_down_vel", go_down_vel_, 0.6);
  nhp_.param("goal_pos_x", goal_pos_x_, 0.7);
  nhp_.param("goal_pos_y", goal_pos_y_, -0.6);
  nhp_.param("searching_x_0", searching_x_0_, -1.3);
  nhp_.param("searching_y_0", searching_y_0_, -0.9);
  nhp_.param("searching_x_1", searching_x_1_, 0.0);
  nhp_.param("searching_y_1", searching_y_1_, -0.3);
  nhp_.param("searching_z", searching_z_, 0.8);
  nhp_.param("object_mass", object_mass_, 0.375);
  nhp_.param("root_link", root_link_, 3);
  nhp_.param("object_search_time_thresh", object_search_time_thresh_, 20);
  nhp_.param("object_detect_time_thresh", object_detect_time_thresh_, 2);
  nhp_.param("wait_time_thresh", wait_time_thresh_, 3);
  nhp_.param("pos_nav_thresh", pos_nav_thresh_, 0.2);
  nhp_.param("vel_nav_limit", vel_nav_limit_, 0.4);
  nhp_.param("vel_nav_gain", vel_nav_gain_, 1.0);

  //publisher
  joint_pub_ = nh_.advertise<sensor_msgs::JointState>(joint_pub_topic_name_, 1);
  uav_pos_pub_ = nh_.advertise<aerial_robot_base::FlightNav>(uav_pos_pub_topic_name_, 1);
  state_machine_pub_ = nh_.advertise<std_msgs::UInt8>(state_machine_pub_topic_name_, 1);
  object_world_pos_pub_ = nh_.advertise<geometry_msgs::Pose>(object_world_pos_pub_topic_name_, 1);
  yaw_control_pub_ = nh_.advertise<std_msgs::Bool>(yaw_control_pub_topic_name_, 1);

  //subscriber
  gripper_state_sub_[0] = nh_.subscribe<std_msgs::Int16>(gripper_state_sub_topic_name_[0], 1, &HydrusObjectTransportation::gripper0StateCallback, this);
  gripper_state_sub_[1] = nh_.subscribe<std_msgs::Int16>(gripper_state_sub_topic_name_[1], 1, &HydrusObjectTransportation::gripper1StateCallback, this);
  odom_sub_ = nh_.subscribe<aerial_robot_base::States>(odom_sub_topic_name_, 1, &HydrusObjectTransportation::odomCallback, this);
  target_image_center_sub_ = nh_.subscribe<geometry_msgs::PoseArray>(target_image_center_topic_name_, 1, &HydrusObjectTransportation::targetImageCenterCallback, this);
  camera_info_sub_ = nh_.subscribe<sensor_msgs::CameraInfo>(camera_info_sub_topic_name_, 1, &HydrusObjectTransportation::cameraInfoCallback, this); 
  joy_stick_sub_ = nh_.subscribe<sensor_msgs::Joy>(joy_stick_sub_topic_name_, 1, &HydrusObjectTransportation::joyStickCallback, this);
  debug_sub_ = nh_.subscribe<std_msgs::Empty>(debug_sub_topic_name_, 1, &HydrusObjectTransportation::debugCallback, this);

  //client
  add_extra_module_client_ = nh_.serviceClient<hydrus_transform_control::AddExtraModule>(add_extra_module_service_name_);
  gripper_control_client_[0] = nh_.serviceClient<jsk_mbzirc_board::Magnet>(gripper_control_service_name_[0]);
  gripper_control_client_[1] = nh_.serviceClient<jsk_mbzirc_board::Magnet>(gripper_control_service_name_[1]);
  
  //timer
  timer_ = nh.createTimer(ros::Duration(1.0 / control_frequency_), &HydrusObjectTransportation::controlCallback, this);

  //member variables
  state_machine_ = StateMachine::SEARCH_OBJECT_;
  detected_object_num_ = 0;
  using_gripper_num_ = 0;
  for (int i = 0; i < link_num_ - 1; i++) {
    joint_values_[i] = joint_values_with_no_object_[i];
  }
  camera_info_update_ = false;
  object_search_cnt_ = 0;
  object_detect_cnt_ = 0;
  start_flag_ = false;
  object_throw_cnt_ = 0;
  transform_wait_cnt_ = 0;
  zero_point_cnt_ = 0;
  called_flag_ = false;
  yaw_track_flag_ = false;
}

HydrusObjectTransportation::~HydrusObjectTransportation()
{
}

void HydrusObjectTransportation::gripper0StateCallback(const std_msgs::Int16ConstPtr& msg)
{
  gripper_state_[0] = msg->data;
}

void HydrusObjectTransportation::gripper1StateCallback(const std_msgs::Int16ConstPtr& msg)
{
  gripper_state_[1] = msg->data;
}

void HydrusObjectTransportation::odomCallback(const aerial_robot_base::StatesConstPtr& msg)
{
  for (int i = 0; i < msg->states.size(); i++) {
    aerial_robot_base::State state = msg->states[i];
    if (state.id == "x") {
      uav_w_.setX(state.state[aerial_robot_base::State::EXPERIMENT_ESTIMATE].x);
    } else if (state.id == "y") {
      uav_w_.setY(state.state[aerial_robot_base::State::EXPERIMENT_ESTIMATE].x);
    } else if (state.id == "z") {
      uav_w_.setZ(state.state[aerial_robot_base::State::EXPERIMENT_ESTIMATE].x);
    } else if (state.id == "yaw") {
      uav_yaw_w_ = state.reserves[4];
    }
  }
}

void HydrusObjectTransportation::targetImageCenterCallback(const geometry_msgs::PoseArrayConstPtr& msg)
{
  detected_object_num_ = msg->poses.size();
  detected_object_pos_camera_.resize(detected_object_num_);
  for (int i = 0; i < detected_object_num_; i++) {
    tf::Vector3 object_pos(msg->poses[i].position.x, msg->poses[i].position.y, 0);
    detected_object_pos_camera_.at(i) = object_pos;
  }
  std::sort(detected_object_pos_camera_.begin(), detected_object_pos_camera_.end(), [](const tf::Vector3& l, const tf::Vector3& r){return l.x() > r.x();});
}

void HydrusObjectTransportation::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg)
{
  if (camera_info_update_) return;
  camera_intrinsic_matrix_.setValue(msg->K[0], 0.0, msg->K[2],
                                    0.0, msg->K[4], msg->K[5],
                                    0.0, 0.0, 1.0);
  camera_intrinsic_matrix_inv_ = camera_intrinsic_matrix_.inverse();
  if (msg->K[0] > 0) {
    camera_info_update_ = true;
  }
}

void HydrusObjectTransportation::joyStickCallback(const sensor_msgs::JoyConstPtr& msg)
{
  /* R1 */
  if (msg->buttons[11] == 1) {
    start_flag_ = true;
    ROS_WARN("Object Transportation Start!");
  }

  /* L2 */
  if (msg->buttons[8] == 1) {
    std_msgs::Bool yaw_control;
    yaw_control.data = false;
    yaw_control_pub_.publish(yaw_control);
    ROS_WARN("Disable yaw control");
  }

  /* R2 */
  if (msg->buttons[9] == 1) {
    std_msgs::Bool yaw_control;
    yaw_control.data = true;
    yaw_control_pub_.publish(yaw_control);
    ROS_WARN("Enable yaw control");   
    yaw_track_flag_ = true;
  }
}

void HydrusObjectTransportation::throwObject()
{
  jsk_mbzirc_board::Magnet srv;
  srv.request.on = 0;
  srv.request.time_ms = 1000;
  gripper_control_client_[0].call(srv);
  gripper_control_client_[1].call(srv);
}

void HydrusObjectTransportation::goPos(tf::Vector3 target_pos)
{
  tf::Vector3 delta = target_pos - uav_w_;
  delta.setZ(0.0);
  if (delta.length() < pos_nav_thresh_) {
    aerial_robot_base::FlightNav flight_nav;
    flight_nav.pos_xy_nav_mode = aerial_robot_base::FlightNav::POS_MODE;
    flight_nav.target_pos_x = target_pos.x();
    flight_nav.target_pos_y = target_pos.y();
    flight_nav.pos_z_nav_mode = aerial_robot_base::FlightNav::POS_MODE;
    flight_nav.target_pos_z = target_pos.z();
    if (yaw_track_flag_ == false) {
      flight_nav.psi_nav_mode = aerial_robot_base::FlightNav::NO_NAVIGATION;
    } else {
      flight_nav.psi_nav_mode = aerial_robot_base::FlightNav::POS_MODE;
      flight_nav.target_psi = uav_yaw_w_;
    }
    flight_nav.header.stamp = ros::Time::now();
    uav_pos_pub_.publish(flight_nav);
  } else {
    tf::Vector3 nav_vel = delta * vel_nav_gain_;
    if (nav_vel.length() > vel_nav_limit_) {
      nav_vel *= (vel_nav_limit_ / nav_vel.length());
    }
    aerial_robot_base::FlightNav flight_nav;
    flight_nav.pos_xy_nav_mode = aerial_robot_base::FlightNav::VEL_MODE;
    flight_nav.target_vel_x = nav_vel.x();
    flight_nav.target_vel_y = nav_vel.y();
    flight_nav.pos_z_nav_mode = aerial_robot_base::FlightNav::POS_MODE;
    flight_nav.target_pos_z = target_pos.z();
    if (yaw_track_flag_ == false) {
      flight_nav.psi_nav_mode = aerial_robot_base::FlightNav::NO_NAVIGATION;
    } else {
      flight_nav.psi_nav_mode = aerial_robot_base::FlightNav::POS_MODE;
      flight_nav.target_psi = uav_yaw_w_;
    }
    flight_nav.header.stamp = ros::Time::now();
    uav_pos_pub_.publish(flight_nav);
  }
}

void HydrusObjectTransportation::addExtraModule(bool reset, int extra_module_link, double extra_module_mass, double extra_module_offset)
{
  hydrus_transform_control::AddExtraModule srv;
  srv.request.reset = reset;
  srv.request.extra_module_link = extra_module_link;
  srv.request.extra_module_mass = extra_module_mass;
  srv.request.extra_module_offset = extra_module_offset;
  add_extra_module_client_.call(srv);
}

void HydrusObjectTransportation::controlCallback(const ros::TimerEvent& event)
{
  if (!camera_info_update_ || !start_flag_) return;
	   //何もなければその場にとどまる
  tf::Vector3 target_w = uav_w_;
   
#ifdef POS_DEBUG
  ROS_INFO("x:%f y:%f z:%f", uav_w_.x(), uav_w_.y(), uav_w_.z()); 
#endif

  tf::Vector3 camera_w;
  double camera_yaw_w;
  tf::Vector3 gripper_w;
  //calc gripper and camera pos
  camera_w.setValue(uav_w_.x() + cos(uav_yaw_w_) * 0.245, uav_w_.y() + sin(uav_yaw_w_) * 0.245, 0.0);
  camera_yaw_w = uav_yaw_w_; 

  //theta_1=theta_2=pi/2
  if (using_gripper_num_ == 1) {
    double x_offset = -0.3;
    gripper_w.setValue(uav_w_.x() + cos(uav_yaw_w_) * x_offset, uav_w_.y() + sin(uav_yaw_w_) * x_offset, 0.0);
  } else if (using_gripper_num_ == 0) {
    double x_offset = 0.3;
    double y_offset = -0.6;
    gripper_w.setValue(uav_w_.x() + cos(uav_yaw_w_) * x_offset - sin(uav_yaw_w_) * y_offset, uav_w_.y() + sin(uav_yaw_w_) * x_offset + cos(uav_yaw_w_) * y_offset, 0.0);
  }

  //state machine
  /* SEARCH_OBJECT */
  if (state_machine_ == StateMachine::SEARCH_OBJECT_) {
    if (using_gripper_num_ == 0) {
      target_w.setValue(searching_x_0_, searching_y_0_, searching_z_);
    } else if (using_gripper_num_ == 1) {
      target_w.setValue(searching_x_1_, searching_y_1_, searching_z_);
    }
    if (detected_object_pos_camera_.size() > 0 && std::abs(target_w.x() - uav_w_.x()) < object_pos_thresh_ && std::abs(target_w.y() - uav_w_.y()) < object_pos_thresh_ && std::abs(target_w.z() - uav_w_.z()) < object_pos_thresh_) {
	ROS_WARN("in the area and object detected");
	//object detected
      if (object_detect_cnt_ == 0) {
        object_pos_vec_.resize(0);
      }
      
      tf::Vector3 camera_coord_image_pos(detected_object_pos_camera_.at(0).x(), detected_object_pos_camera_.at(0).y(), 1.0);
      tf::Matrix3x3 R;
      R.setRPY(0, 0, M_PI/2);
      tf::Vector3 world_coord_image_pos = R * (camera_intrinsic_matrix_inv_ * camera_coord_image_pos);
     
      double target_object_x_camera_center = uav_w_.z() * world_coord_image_pos.x() / world_coord_image_pos.z() + 0.245;
      double target_object_y_camera_center = -uav_w_.z() * world_coord_image_pos.y() / world_coord_image_pos.z();

      double target_object_x_w = uav_w_.x() + cos(camera_yaw_w) * target_object_x_camera_center - sin(camera_yaw_w) * target_object_y_camera_center;
      double target_object_y_w = uav_w_.y() + sin(camera_yaw_w) * target_object_x_camera_center + cos(camera_yaw_w) * target_object_y_camera_center; 
    
      tf::Vector3 object_pos(target_object_x_w, target_object_y_w, 0);
      object_pos_vec_.push_back(object_pos);
#ifdef OBJECT_POS_DEBUG
      ROS_INFO("camera_coord_image_pos_x:%f y:%f", camera_coord_image_pos.x(), camera_coord_image_pos.y());
      ROS_INFO("world_coord_image_pos_x:%f y:%f z:%f", world_coord_image_pos.x(), world_coord_image_pos.y(), world_coord_image_pos.z());	      
      ROS_INFO("target_object_x_camera_center:%f, y:%f", target_object_x_camera_center, target_object_y_camera_center);
      ROS_INFO("target_object_x_w:%f y:%f", target_object_x_w, target_object_y_w);
#else
      object_detect_cnt_++;
#endif
      ROS_INFO("object detect cnt:%d", object_detect_cnt_);
      if (object_detect_cnt_ == object_detect_time_thresh_ * control_frequency_) {
        /*tf::Vector3 object_pos_average(0, 0, 0);
        for (int i = 0; i < object_pos_vec_.size(); i++) {
          object_pos_average += object_pos_vec_.at(i);
        }
        object_pos_average /= object_pos_vec_.size();
        target_object_w_.setValue(object_pos_average.x(), object_pos_average.y(), 0.0);
	*/
	target_object_w_.setValue(object_pos.x(), object_pos.y(), 0.0);
        state_machine_ = StateMachine::APPROACH_TARGET_;
        object_search_cnt_ = 0;
        object_detect_cnt_ = 0;
      }
    }
  /* APPROACH_TARGET */
  } else if (state_machine_ == StateMachine::APPROACH_TARGET_) {
    target_w.setValue(target_object_w_.x() - gripper_w.x() + uav_w_.x() - 0.1,
                      target_object_w_.y() - gripper_w.y() + uav_w_.y(),
                      uav_w_.z());
    if (std::abs(target_object_w_.x() - gripper_w.x()) < object_pos_thresh_ && std::abs(target_object_w_.y() - gripper_w.y()) < object_pos_thresh_){
      state_machine_ = StateMachine::GO_DOWN_UNTIL_TOUCH_;
    }
  /* GO_DOWN_UNTIL_TOUCH */
  } else if (state_machine_ == StateMachine::GO_DOWN_UNTIL_TOUCH_) {
    target_w.setValue(target_object_w_.x() - gripper_w.x() + uav_w_.x(),
                      target_object_w_.y() - gripper_w.y() + uav_w_.y(),
                      uav_w_.z() - (go_down_vel_ / control_frequency_));
    if (uav_w_.z() < 0.20) {
      state_machine_ = StateMachine::TRANSFORM_;
    }
  /* ADD_EXTRA_MODULE */
  } else if (state_machine_ == StateMachine::ADD_EXTRA_MODULE_) {
    if (using_gripper_num_ == 0) {
      for (int i = 0; i < link_num_ - 1; i++) {
        joint_values_[i] = joint_values_with_one_object_[i];
      }	
    } else if (using_gripper_num_ == 1) {
      for (int i = 0; i < link_num_ - 1; i++) {
        joint_values_[i] = joint_values_with_two_object_[i];
      } 
    }
    addExtraModule(false, gripper_link_num_[using_gripper_num_], object_mass_, gripper_link_offset_[using_gripper_num_]);
    state_machine_ = StateMachine::TRANSFORM_;
    //gripper update
    using_gripper_num_++;
  /* TRANSFORM */
  } else if (state_machine_ == StateMachine::TRANSFORM_) {
      transform_wait_cnt_++;
      if (transform_wait_cnt_ >= control_frequency_ * 10) {
        target_w.setZ(searching_z_);
        if (std::abs(searching_z_ - uav_w_.z()) < object_pos_thresh_) {
          if (using_gripper_num_ == object_num_ - 1) {
            state_machine_ = StateMachine::SEARCH_GOAL_;
          } else {
            state_machine_ = StateMachine::SEARCH_OBJECT_;
          }
          transform_wait_cnt_ = 0;
        }		
      }     
  /* SEARCH_GOAL */
  } else if (state_machine_ == StateMachine::SEARCH_GOAL_) {
    state_machine_ = StateMachine::APPROACH_GOAL_; 
    /* APPROACH_GOAL */
  } else if (state_machine_ == StateMachine::APPROACH_GOAL_) {
    target_w.setValue(goal_pos_x_, goal_pos_y_, searching_z_); 
    if (std::abs(uav_w_.x() - goal_pos_x_) < object_pos_thresh_ && std::abs(uav_w_.y() - goal_pos_y_) < object_pos_thresh_){
      object_throw_cnt_++;
      if (object_throw_cnt_ == wait_time_thresh_ * control_frequency_) {
        state_machine_ = StateMachine::THROW_OBJECT_;
        object_throw_cnt_ = 0;
      }
    }
  /* THROW_OBJECT */
  } else if (state_machine_ == StateMachine::THROW_OBJECT_) {
    throwObject();
  }

  //publish
  sensor_msgs::JointState joint_state;
  joint_state.name.resize(link_num_ - 1);
  joint_state.position.resize(link_num_ - 1);
  for (int i = 0; i < link_num_ - 1; i++) {
    joint_state.name[i] = std::string("joint") + boost::to_string(i + 1);
    joint_state.position[i] = joint_values_[i];
  }
  joint_state.header.stamp = ros::Time::now();
  joint_pub_.publish(joint_state);

  goPos(target_w);
  std_msgs::UInt8 state_msg;
  state_msg.data = static_cast<unsigned int>(state_machine_);
  state_machine_pub_.publish(state_msg);
	
  geometry_msgs::Pose object_world_pos;
  object_world_pos.position.x = target_object_w_.x();
  object_world_pos.position.y = target_object_w_.y();
  object_world_pos_pub_.publish(object_world_pos);

//debug
#ifdef GRIPPER_INPUT_DEBUG
  ROS_INFO("gripper0:%d gripper1:%d", gripper_state_[0], gripper_state_[1]);
#endif
#ifdef CAMERA_POS_DEBUG
  ROS_INFO("camera_x:%f camera_y:%f camera_yaw_:%f", camera_w.x(), camera_w.y(), camera_yaw_w);
#endif
#ifdef GRIPPER_POS_DEBUG
  ROS_INFO("gripper_x:%f gripper_y:%f", gripper_w.x(), gripper_w.y());
#endif
}

void HydrusObjectTransportation::debugCallback(const std_msgs::EmptyConstPtr& msg)
{
#ifdef GRIPPER_OUTPUT_DEBUG
  ROS_INFO("throw object");
  throwObject();
#endif
#ifdef GRIPPER_POS_DEBUG
  if (using_gripper_num_ == 0) using_gripper_num_ = 1;
  else using_gripper_num_ = 0;
  ROS_INFO("using gripper num:%d", using_gripper_num_);
#endif
}
