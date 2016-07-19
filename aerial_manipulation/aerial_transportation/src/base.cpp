#include <aerial_transportation/base.h>

namespace aerial_transportation
{
  void Base::baseInit(ros::NodeHandle nh, ros::NodeHandle nhp)
  {
    nh_ = nh;
    nhp_ = nhp;

    baseRosParamInit();

    /* pub & sub */
    uav_nav_pub_ = nh_.advertise<aerial_robot_base::FlightNav>(uav_nav_pub_name_, 1);
    uav_state_sub_ = nh_.subscribe<nav_msgs::Odometry>(uav_state_sub_name_, 1, &Base::stateCallback, this);
    object_pos_sub_ = nh_.subscribe<geometry_msgs::Pose2D>(object_pos_sub_name_, 1, &Base::objectPoseCallback, this);
    joy_stick_sub_ = nh_.subscribe<sensor_msgs::Joy>("/joy", 1, &Base::joyStickCallback, this);

    /* variables init */
    phase_ = IDLE_PHASE;
    get_uav_state_ = false;
    contact_cnt_ = 0;

    /* timer init */
    func_timer_ = nhp_.createTimer(ros::Duration(1.0 / func_loop_rate_), &Base::mainFunc,this);

  }

  void Base::baseRosParamInit()
  {
    std::string ns = nhp_.getNamespace();

    nhp_.param("debug", debug_, false); 
    nhp_.param("uav_state_sub_name", uav_state_sub_name_, std::string("/uav/state"));
    nhp_.param("uav_nav_pub_name", uav_nav_pub_name_, std::string("/uav/nav"));
    nhp_.param("object_pos_sub_name", object_pos_sub_name_, std::string("/object"));
    nhp_.param ("func_loop_rate", func_loop_rate_, 40.0);

    /* nav param */
    nhp_.param("nav_vel_limit", nav_vel_limit_, 0.2);
    nhp_.param("vel_nav_threshold", vel_nav_threshold_, 0.4);
    nhp_.param("vel_nav_gain", vel_nav_gain_, 1.0);

    nhp_.param("approach_pos_threshold", approach_pos_threshold_, 0.05);
    nhp_.param("approach_yaw_threshold", approach_yaw_threshold_, 0.1);
    nhp_.param("approach_count", approach_count_, 2.0); //sec
    nhp_.param("object_head_direction", object_head_direction_, false);

    nhp_.param("object_height", object_height_, 0.2);
    nhp_.param("falling_speed", falling_speed_, 0.04);
    nhp_.param("grasping_height_offset", grasping_height_offset_, -0.01);
    nhp_.param("ascending_speed", ascending_speed_, 0.1);

    nhp_.param("transportation_threshold", transportation_threshold_, 0.1);
    nhp_.param("transportation_count", transportation_count_, 2.0); //sec
    nhp_.param("dropping_offset", dropping_offset_, 0.15);

    /* box point */
    nhp_.param("box_x", box_point_.x, 1.145);
    nhp_.param("box_y", box_point_.y, 0.02);
    nhp_.param("box_z", box_point_.z, 0.2);
    printf("%s: box x: %f, y: %f, z: %f\n", ns.c_str(), box_point_.x, box_point_.y, box_point_.z);

    /* should detect!!! */
    nhp_.param("object_offset_x", object_offset_.m_floats[0], 0.0);
    nhp_.param("object_offset_y", object_offset_.m_floats[1], 0.0);
    nhp_.param("object_offset_yaw", object_offset_.m_floats[2], 0.0);
    // has four elements: x, y, height, yaw

    nhp_.param("box_offset_x", box_offset_.m_floats[0], 0.0);
    nhp_.param("box_offset_y", box_offset_.m_floats[1], 0.0);
    nhp_.param("box_offset_z", box_offset_.m_floats[2], 0.0);

  }

  void Base::stateCallback(const nav_msgs::OdometryConstPtr & msg)
  {
    if(!get_uav_state_) get_uav_state_ = true;

    uav_position_.position.x = msg->pose.pose.position.x;
    uav_position_.position.y = msg->pose.pose.position.y;
    uav_position_.position.z = msg->pose.pose.position.z;
    uav_position_.orientation.x = msg->pose.pose.orientation.x;
    uav_position_.orientation.y = msg->pose.pose.orientation.y;
    uav_position_.orientation.z = msg->pose.pose.orientation.z;
    uav_position_.orientation.w = msg->pose.pose.orientation.w;
  }

  void Base::joyStickCallback(const sensor_msgs::JoyConstPtr & joy_msg)
  {
    if(joy_msg->buttons[4] == 1) // up arrow
      {//start
        if(phase_ == IDLE_PHASE && get_uav_state_)
          {
            phase_ ++; // go to approach phase
            ROS_INFO("shift to APPROACH_PHASE");
            uav_init_position_ = uav_position_;
          }
      }

    if(joy_msg->buttons[10] == 1) // LEFT TOP Trigger
      {//reset
        phase_ = IDLE_PHASE;
      }

    if(debug_)
      {
        if(joy_msg->buttons[10] == 1) // LEFT TOP TRIGGER(L1)
          {//start grasp
            phase_ = GRASPING_PHASE;
          }
        if(joy_msg->buttons[8] == 1) // LEFT DOWM TRIGGER(L1)
          {//start drop
            phase_ = DROPPING_PHASE;
          }
      }


    joyStickAdditionalCallback(joy_msg);
  }

  void Base::objectPoseCallback(const geometry_msgs::Pose2DConstPtr & object_msg)
  {
    if(!object_found_) object_found_ = true;
    object_position_.x = object_msg->x;
    object_position_.y = object_msg->y;
    object_position_.theta = object_msg->theta;

    /* calculate the offset to approach the object(x,y,yaw) */
    objectPoseApproachOffsetCal();
  }

  void Base::mainFunc(const ros::TimerEvent & e)
  {
    static int cnt = 0;

    if(!get_uav_state_) return;

    switch(phase_)
      {
      case APPROACH_PHASE:
        {
          if(!object_found_) break;

          /* nav part */
          tf::Vector3 delta((object_position_.x + object_offset_.x()) - uav_position_.position.x, (object_position_.y + object_offset_.y()) - uav_position_.position.y, 0.0);

          if(delta.length() < vel_nav_threshold_)
            {
              aerial_robot_base::FlightNav nav_msg;
              nav_msg.header.stamp = ros::Time::now();
              nav_msg.pos_xy_nav_mode = aerial_robot_base::FlightNav::POS_MODE;
              nav_msg.target_pos_x = object_position_.x + object_offset_.x();
              nav_msg.target_pos_y = object_position_.y + object_offset_.y();
              nav_msg.pos_z_nav_mode = aerial_robot_base::FlightNav::NO_NAVIGATION;
              if(object_head_direction_)
                {
                  nav_msg.psi_nav_mode = aerial_robot_base::FlightNav::POS_MODE;
                  nav_msg.target_psi = object_position_.theta + object_offset_.z();
                }
              nav_msg.psi_nav_mode = aerial_robot_base::FlightNav::NO_NAVIGATION;
              uav_nav_pub_.publish(nav_msg);
            }
          else
            {// should use vel nav
              tf::Vector3 nav_vel = delta * vel_nav_gain_;
              //ROS_INFO("DEBUG: nav vel is x: %f, y: %f, delta_x: %f, delta_y: %f", nav_vel.x(), nav_vel.y(), delta.x(), delta.y());

              double speed = nav_vel.length();
              if(speed  > nav_vel_limit_)
                {
                  //ROS_WARN("exceeds the vel limitation: %f", speed);
                  nav_vel *= (nav_vel_limit_ / speed);
                }

              aerial_robot_base::FlightNav nav_msg;
              nav_msg.pos_xy_nav_mode = aerial_robot_base::FlightNav::VEL_MODE;
              nav_msg.target_vel_x = nav_vel.x();
              nav_msg.target_vel_y = nav_vel.y();
              nav_msg.pos_z_nav_mode = aerial_robot_base::FlightNav::NO_NAVIGATION;
              if(object_head_direction_)
                {
                  nav_msg.psi_nav_mode = aerial_robot_base::FlightNav::POS_MODE;
                  nav_msg.target_psi = object_position_.theta + object_offset_.z();
                }
              nav_msg.psi_nav_mode = aerial_robot_base::FlightNav::NO_NAVIGATION;
              uav_nav_pub_.publish(nav_msg);
            }

          /* phase shift condition */
          bool approach = true;
          /* position check */
          if(delta.length() >  approach_pos_threshold_) approach = false;
          if(object_head_direction_)
            {
              //ROS_INFO("DEBUG: check yaw");
              tf::Matrix3x3 rotation(tf::Quaternion(uav_position_.orientation.x,
                                                    uav_position_.orientation.y,
                                                    uav_position_.orientation.z,
                                                    uav_position_.orientation.w));
              tfScalar r, p, y;
              rotation.getRPY(r, p, y);
              if(fabs(object_position_.theta + object_offset_.z() - y) > approach_yaw_threshold_) approach = false;
            }

            if(approach)
              {
                if(++cnt > (approach_count_ * func_loop_rate_))
                  {
                    ROS_WARN("Succeed to approach to object, shift to GRASPING_PHASE");
                    phase_ ++;
                    cnt = 0; // convergence reset
                    target_height_ = uav_position_.position.z; //falling down init
                  }
            }
          break;
        }
      case GRASPING_PHASE:
        {
          graspPhase();
          break;
        }
      case GRASPED_PHASE:
        {
	  /* height calc part */
	  target_height_ += (ascending_speed_ / func_loop_rate_);
	  if(target_height_ > (box_point_.z + object_height_ + dropping_offset_))
	    target_height_ = box_point_.z + object_height_ + dropping_offset_;

          /* send nav msg */
          aerial_robot_base::FlightNav nav_msg;
	  nav_msg.header.stamp = ros::Time::now();
          nav_msg.pos_xy_nav_mode = aerial_robot_base::FlightNav::VEL_MODE;
          nav_msg.target_vel_x = 0;
          nav_msg.target_vel_y = 0;
          nav_msg.pos_z_nav_mode = aerial_robot_base::FlightNav::POS_MODE;
          nav_msg.target_pos_z = target_height_;
          nav_msg.psi_nav_mode = aerial_robot_base::FlightNav::NO_NAVIGATION;
          uav_nav_pub_.publish(nav_msg);

          if(fabs(box_point_.z + object_height_ + dropping_offset_ - uav_position_.position.z)  < 0.05) //0.05m, hard-coding
            {
              ROS_INFO("Shift to TRANSPORT_PHASE");
              phase_ = TRANSPORT_PHASE;
              cnt = 0;
            }
          break;
        }
      case TRANSPORT_PHASE:
        {
          /* nav part */
          tf::Vector3 delta(box_point_.x + box_offset_.x() - uav_position_.position.x, box_point_.y  + box_offset_.y() - uav_position_.position.y, 0.0);

          if(delta.length() < vel_nav_threshold_)
            {
              aerial_robot_base::FlightNav nav_msg;
	      nav_msg.header.stamp = ros::Time::now();
              nav_msg.pos_xy_nav_mode = aerial_robot_base::FlightNav::POS_MODE;
              nav_msg.target_pos_x = box_point_.x + box_offset_.x();
              nav_msg.target_pos_y = box_point_.y + box_offset_.y();
              nav_msg.pos_z_nav_mode = aerial_robot_base::FlightNav::NO_NAVIGATION;
              nav_msg.psi_nav_mode = aerial_robot_base::FlightNav::NO_NAVIGATION;
              uav_nav_pub_.publish(nav_msg);
            }
          else
            {// should use vel nav
              tf::Vector3 nav_vel = delta * vel_nav_gain_;
              //ROS_INFO("DEBUG: nav vel is x: %f, y: %f, delta_x: %f, delta_y: %f", nav_vel.x(), nav_vel.y(), delta.x(), delta.y());

              double speed = nav_vel.length();
              if(speed  > nav_vel_limit_)
                {
                  //ROS_WARN("exceeds the vel limitation: %f", speed);
                  nav_vel *= (nav_vel_limit_ / speed);
                }

              aerial_robot_base::FlightNav nav_msg;
	      nav_msg.header.stamp = ros::Time::now();
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
                  ROS_INFO("Succeed to approach to box, shift to DROPPING_PHASE, and drop!!");
                  phase_ ++;
                  cnt = 0;
              }
            }
          break;
        }
      case DROPPING_PHASE:
        {
          dropPhase();
          break;
        }
      case RETURN_PHASE:
        {
          tf::Vector3 delta(uav_init_position_.position.x - uav_position_.position.x , uav_init_position_.position.y - uav_position_.position.y , 0);

          if(delta.length() < vel_nav_threshold_)
            {// shift to pos nav and also shift to idle phase
              aerial_robot_base::FlightNav nav_msg;
	      nav_msg.header.stamp = ros::Time::now();
              nav_msg.pos_xy_nav_mode = aerial_robot_base::FlightNav::POS_MODE;
              nav_msg.target_pos_x = uav_init_position_.position.x;
              nav_msg.target_pos_y = uav_init_position_.position.y;
              nav_msg.pos_z_nav_mode = aerial_robot_base::FlightNav::NO_NAVIGATION;
              nav_msg.psi_nav_mode = aerial_robot_base::FlightNav::NO_NAVIGATION;
              uav_nav_pub_.publish(nav_msg);

              phase_ = IDLE_PHASE;
              ROS_INFO("Shifht to IDLE_PHASE");
            }
          else
            {// should use vel nav
              tf::Vector3 nav_vel = delta * vel_nav_gain_;
              //ROS_INFO("DEBUG: nav vel is x: %f, y: %f, delta_x: %f, delta_y: %f", nav_vel.x(), nav_vel.y(), delta.x(), delta.y());

              double speed = nav_vel.length();
              if(speed  > nav_vel_limit_)
                {
                  //ROS_WARN("exceeds the vel limitation: %f", speed);
                  nav_vel *= (nav_vel_limit_ / speed);
                }

              aerial_robot_base::FlightNav nav_msg;
              nav_msg.header.stamp = ros::Time::now();
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
};
