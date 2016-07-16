#include <object_transportation/object_transportation.h>
#include <object_transportation/grasp/grasp_base_plugin.h>

ObjectTransportation::ObjectTransportation(ros::NodeHandle nh, ros::NodeHandle nhp):nh_(nh), nhp_(nhp)
{
  rosParamInit();

  /* pub & sub */
  uav_nav_pub_ = nh_.advertise<aerial_robot_base::FlightNav>("/uav/nav", 1);
  uav_state_sub_ = nh_.subscribe<nav_msgs::Odometry>("/uav/state", 1, &ObjectTransportation::stateCallback, this, ros::TransportHints().udp());
  object_pos_sub_ = nh_.subscribe<geometry_msgs::Pose2D>("/object", 1, &ObjectTransportation::objectPoseCallback, this, ros::TransportHints().udp());
  joy_stick_sub_ = nh_.subscribe<sensor_msgs::Joy>("/joy", 1, &ObjectTransportation::joyStickCallback, this, ros::TransportHints().udp());

  /* plugin initialization */
  grasp_loader_ptr_ =  boost::shared_ptr< pluginlib::ClassLoader<grasp_base_plugin::GraspBase> >( new pluginlib::ClassLoader<grasp_base_plugin::GraspBase>("object_transportation", "grasp_base_plugin::GraspBase"));
  grasp_method_ = grasp_loader_ptr_->createInstance(grasp_plugin_name_);
  grasp_method_->initialize(nh_, nhp_, this);

  /* variables init */
  phase_ = IDLE_PHASE;
  uav_state_ = false;

  func_timer_ = nhp_.createTimer(ros::Duration(1.0 / func_loop_rate_), &ObjectTransportation::mainFunc,this);

}

void ObjectTransportation::rosParamInit()
{
  std::string ns = nhp_.getNamespace();
  nhp_.param ("func_loop_rate", func_loop_rate_, 40.0);
  printf("%s: func_loop_rate_ is %.3f\n", ns.c_str(), func_loop_rate_);

  nhp_.param("uav_state_sub_name", uav_state_sub_name_, std::string("/uav/states"));
  nhp_.param("uav_nav_pub_name", uav_nav_pub_name_, std::string("/uav/nav"));

  /* box point */
  nhp_.param("box_x", box_point_.x, 1.145);
  nhp_.param("box_y", box_point_.y, 0.02);
  nhp_.param("box_z", box_point_.z, 0.2);
  printf("box x: %f, y: %f, z: %f", box_point_.x, box_point_.y, box_point_.z);

  /* nav param */
  nhp_.param("nav_vel_limit", nav_vel_limit_, 0.2);
  nhp_.param("vel_nav_threshold", vel_nav_threshold_, 0.4);
  nhp_.param("vel_nav_gain", vel_nav_gain_, 1.0);

  nhp_.param("falling_speed", falling_speed_, -0.04);
  nhp_.param("grasping_height_offset", grasping_height_offset_, -0.02);
  nhp_.param("object_height", object_height_, 0.2); //this should be detected!!!!

  nhp_.param("approach_threshold", approach_threshold_, 0.05);
  nhp_.param("approach_count", approach_count_, 2.0); //sec

  nhp_.param("transportation_threshold", transportation_threshold_, 0.1);
  nhp_.param("transportation_count", transportation_count_, 2.0); //sec
  nhp_.param("dropping_offset", dropping_offset_, 0.15);
}

void ObjectTransportation::stateCallback(const nav_msgs::OdometryConstPtr & msg)
{
  if(!uav_state_) uav_state_ = true;

  uav_position_.position.x = msg->pose.pose.position.x;
  uav_position_.position.y = msg->pose.pose.position.y;
  uav_position_.position.z = msg->pose.pose.position.z;
  uav_position_.orientation.x = msg->pose.pose.orientation.x;
  uav_position_.orientation.y = msg->pose.pose.orientation.y;
  uav_position_.orientation.z = msg->pose.pose.orientation.z;
  uav_position_.orientation.w = msg->pose.pose.orientation.w;
}

void ObjectTransportation::joyStickCallback(const sensor_msgs::JoyConstPtr & joy_msg)
{
  if(joy_msg->buttons[4] == 1)
    {//start
      if(phase_ == IDLE_PHASE)
        {
          phase_ ++; // go to approach phase
          ROS_INFO("shift to APPROACH_PHASE");
          uav_init_position_ = uav_position_;
        }
    }

  if(joy_msg->buttons[10] == 1)
    {//reset
      phase_ = IDLE_PHASE;
    }
}

void ObjectTransportation::objectPoseCallback(const geometry_msgs::Pose2DConstPtr & object_msg)
{
  if(!object_found_) object_found_ = true;
  object_position_.x = object_msg->x;
  object_position_.y = object_msg->y;
  object_position_.theta = object_msg->theta;
}

void ObjectTransportation::mainFunc(const ros::TimerEvent & e)
{
  static int cnt = 0;

  if(!uav_state_) return;

  switch(phase_)
  {
  case APPROACH_PHASE:
    {
      if(!object_found_) break;

      /* nav part */
      tf::Vector3 delta(object_position_.x - uav_position_.position.x, object_position_.y - uav_position_.position.y, 0);

      if(delta.length() < vel_nav_threshold_)
        {
          aerial_robot_base::FlightNav nav_msg;
          nav_msg.pos_xy_nav_mode = aerial_robot_base::FlightNav::POS_MODE;
          nav_msg.target_pos_x = object_position_.x;
          nav_msg.target_pos_y = object_position_.y;
          nav_msg.pos_z_nav_mode = aerial_robot_base::FlightNav::NO_NAVIGATION;
          nav_msg.psi_nav_mode = aerial_robot_base::FlightNav::NO_NAVIGATION;
          uav_nav_pub_.publish(nav_msg);
        }
      else
        {// should use vel nav
          tf::Vector3 nav_vel = delta * vel_nav_gain_;
          ROS_INFO("DEBUG: nav vel is x: %f, y: %f, delta_x: %f, delta_y: %f", nav_vel.x(), nav_vel.y(), delta.x(), delta.y());

          double speed = nav_vel.length();
          if(speed  > nav_vel_limit_)
            {
              ROS_WARN("exceeds the vel limitation: %f", speed);
              nav_vel *= (nav_vel_limit_ / speed);
            }

          aerial_robot_base::FlightNav nav_msg;
          nav_msg.pos_xy_nav_mode = aerial_robot_base::FlightNav::VEL_MODE;
          nav_msg.target_vel_x = nav_vel.x();
          nav_msg.target_vel_y = nav_vel.y();
          nav_msg.pos_z_nav_mode = aerial_robot_base::FlightNav::NO_NAVIGATION;
          nav_msg.psi_nav_mode = aerial_robot_base::FlightNav::NO_NAVIGATION;
          uav_nav_pub_.publish(nav_msg);
        }

      /* phase shift condition */
      if(delta.length() <  approach_threshold_)
        {
          if(++cnt > (approach_count_ * func_loop_rate_))
            {
              ROS_INFO("succeed to approach to object, shift to GRASPING_PHASE");
              phase_ ++;
              cnt = 0;
              target_height_ = uav_position_.position.z;

              /* plugin */
              grasp_method_->start();
            }
        }
      break;
    }
  case GRASPING_PHASE:
    {
      /* height calc part */
      target_height_ -= (falling_speed_ / func_loop_rate_);
      if(target_height_ < (object_height_ + grasping_height_offset_))
        target_height_ = object_height_ + grasping_height_offset_;

      /* send nav msg */
      aerial_robot_base::FlightNav nav_msg;
      nav_msg.pos_xy_nav_mode = aerial_robot_base::FlightNav::POS_MODE;
      nav_msg.target_pos_x = object_position_.x;
      nav_msg.target_pos_y = object_position_.y;
      nav_msg.pos_z_nav_mode = aerial_robot_base::FlightNav::POS_MODE;
      nav_msg.target_pos_z = target_height_;
      nav_msg.psi_nav_mode = aerial_robot_base::FlightNav::NO_NAVIGATION;
      uav_nav_pub_.publish(nav_msg);

      break;
    }
  case GRASPED_PHASE:
    {
      /* send nav msg */
      aerial_robot_base::FlightNav nav_msg;
      nav_msg.pos_xy_nav_mode = aerial_robot_base::FlightNav::NO_NAVIGATION;
      nav_msg.pos_z_nav_mode = aerial_robot_base::FlightNav::POS_MODE;
      nav_msg.target_pos_z = box_point_.z + object_height_ + dropping_offset_;
      nav_msg.psi_nav_mode = aerial_robot_base::FlightNav::NO_NAVIGATION;
      uav_nav_pub_.publish(nav_msg);

      if(fabs(nav_msg.target_pos_z - uav_position_.position.z)  < 0.05) //0.05m, hard-coding
        {
          ROS_INFO("shift to transport phase");
          phase_ = TRANSPORT_PHASE;
          cnt = 0;
        }

      break;
    }
  case TRANSPORT_PHASE:
    {
      /* nav part */
      tf::Vector3 delta(box_point_.x - uav_position_.position.x, box_point_.y - uav_position_.position.y, 0);

      if(delta.length() < vel_nav_threshold_)
        {
          aerial_robot_base::FlightNav nav_msg;
          nav_msg.pos_xy_nav_mode = aerial_robot_base::FlightNav::POS_MODE;
          nav_msg.target_pos_x = box_point_.x;
          nav_msg.target_pos_y = box_point_.y;
          nav_msg.pos_z_nav_mode = aerial_robot_base::FlightNav::NO_NAVIGATION;
          nav_msg.psi_nav_mode = aerial_robot_base::FlightNav::NO_NAVIGATION;
          uav_nav_pub_.publish(nav_msg);
        }
      else
        {// should use vel nav
          tf::Vector3 nav_vel = delta * vel_nav_gain_;
          ROS_INFO("DEBUG: nav vel is x: %f, y: %f, delta_x: %f, delta_y: %f", nav_vel.x(), nav_vel.y(), delta.x(), delta.y());

          double speed = nav_vel.length();
          if(speed  > nav_vel_limit_)
            {
              ROS_WARN("exceeds the vel limitation: %f", speed);
              nav_vel *= (nav_vel_limit_ / speed);
            }

          aerial_robot_base::FlightNav nav_msg;
          nav_msg.pos_xy_nav_mode = aerial_robot_base::FlightNav::VEL_MODE;
          nav_msg.target_vel_x = nav_vel.x();
          nav_msg.target_vel_y = nav_vel.y();
          nav_msg.pos_z_nav_mode = aerial_robot_base::FlightNav::NO_NAVIGATION;
          nav_msg.psi_nav_mode = aerial_robot_base::FlightNav::NO_NAVIGATION;
          uav_nav_pub_.publish(nav_msg);
        }

      /* phase shift condition */
      if(delta.length() <  transportation_threshold_)
        {
          if(++cnt > (transportation_count_ * func_loop_rate_))
            {
              ROS_INFO("succeed to approach to box, shift to DROPPING_PHASE, and drop!!");
              phase_ ++;
              cnt = 0;

              /* plugin */
              grasp_method_->drop();
            }
        }

      break;
    }
  case DROPPING_PHASE:
    {
      break;
    }
  case RETURN_PHASE:
    {
      tf::Vector3 delta(uav_position_.position.x - uav_init_position_.position.x, uav_position_.position.y - uav_init_position_.position.y, 0);

      if(delta.length() < vel_nav_threshold_)
        {// shift to pos nav and also shift to idle phase
          aerial_robot_base::FlightNav nav_msg;
          nav_msg.pos_xy_nav_mode = aerial_robot_base::FlightNav::POS_MODE;
          nav_msg.target_pos_x = uav_init_position_.position.x;
          nav_msg.target_pos_y = uav_init_position_.position.y;
          nav_msg.pos_z_nav_mode = aerial_robot_base::FlightNav::NO_NAVIGATION;
          nav_msg.psi_nav_mode = aerial_robot_base::FlightNav::NO_NAVIGATION;
          uav_nav_pub_.publish(nav_msg);

          phase_ = IDLE_PHASE;
          ROS_INFO("shifht to idle phase");
        }
      else
        {// should use vel nav
          tf::Vector3 nav_vel = delta * vel_nav_gain_;
          ROS_INFO("DEBUG: nav vel is x: %f, y: %f, delta_x: %f, delta_y: %f", nav_vel.x(), nav_vel.y(), delta.x(), delta.y());

          double speed = nav_vel.length();
          if(speed  > nav_vel_limit_)
            {
              ROS_WARN("exceeds the vel limitation: %f", speed);
              nav_vel *= (nav_vel_limit_ / speed);
            }

          aerial_robot_base::FlightNav nav_msg;
          nav_msg.pos_xy_nav_mode = aerial_robot_base::FlightNav::VEL_MODE;
          nav_msg.target_vel_x = nav_vel.x();
          nav_msg.target_vel_y = nav_vel.y();
          nav_msg.pos_z_nav_mode = aerial_robot_base::FlightNav::NO_NAVIGATION;
          nav_msg.psi_nav_mode = aerial_robot_base::FlightNav::NO_NAVIGATION;
          uav_nav_pub_.publish(nav_msg);
        }

      break;
    }
  default:
    {
      cnt = 0;
      break;
    }
  }
}
