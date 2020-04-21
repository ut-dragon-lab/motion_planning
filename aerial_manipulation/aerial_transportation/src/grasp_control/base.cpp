#include <aerial_transportation/grasp_control/base.h>

namespace aerial_transportation
{
  void Base::baseInit(ros::NodeHandle nh, ros::NodeHandle nhp)
  {
    nh_ = nh;
    nhp_ = nhp;

    baseRosParamInit();

    /* pub & sub */
    uav_nav_pub_ = nh_.advertise<aerial_robot_msgs::FlightNav>(uav_nav_pub_name_, 1);
    uav_state_sub_ = nh_.subscribe(uav_state_sub_name_, 1, &Base::stateCallback, this);
    object_pos_sub_ = nh_.subscribe(object_pos_sub_name_, 1, &Base::objectPoseCallback, this);
    joy_stick_sub_ = nh_.subscribe("/joy", 1, &Base::joyStickCallback, this);

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

    nhp_.param("joy_debug", joy_debug_, false);
    nhp_.param("control_cheat_mode", control_cheat_mode_, false);

    nhp_.param("uav_state_sub_name", uav_state_sub_name_, std::string("/uav/odom"));
    nhp_.param("uav_nav_pub_name", uav_nav_pub_name_, std::string("/uav/nav"));
    nhp_.param("object_pos_sub_name", object_pos_sub_name_, std::string("/object"));
    nhp_.param("func_loop_rate", func_loop_rate_, 40.0);

    nhp_.param("approach_pos_threshold", approach_pos_threshold_, 0.05);
    nhp_.param("approach_yaw_threshold", approach_yaw_threshold_, 0.1);
    nhp_.param("approach_count", approach_count_, 2.0); //sec
    nhp_.param("object_head_direction", object_head_direction_, false);

    nhp_.param("falling_speed", falling_speed_, 0.04);
    nhp_.param("grasping_height_offset", grasping_height_offset_, -0.01);
    nhp_.param("ascending_speed", ascending_speed_, 0.1);

    nhp_.param("transportation_threshold", transportation_threshold_, 0.1);
    nhp_.param("transportation_count", transportation_count_, 2.0); //sec
    nhp_.param("dropping_offset", dropping_offset_, 0.15);

    /* TODO the height of object */
    //if(control_cheat_mode_)
    //{
    nhp_.param("object_height", object_height_, 0.2);
    printf("%s: object, height: %f\n", ns.c_str(), object_height_);
    //}

    //if(recog_cheat_mode_)
    /* recycle box config */
    nhp_.param("box_x", box_point_.x, 1.145);
    nhp_.param("box_y", box_point_.y, 0.02);
    nhp_.param("box_z", box_point_.z, 0.2);
    nhp_.param("box_offset_x", box_offset_.m_floats[0], 0.0);
    nhp_.param("box_offset_y", box_offset_.m_floats[1], 0.0);
    nhp_.param("box_offset_z", box_offset_.m_floats[2], 0.0);
    printf("%s: box x: %f, y: %f, z: %f\n", ns.c_str(), box_point_.x, box_point_.y, box_point_.z);

  }

  void Base::stateCallback(const nav_msgs::OdometryConstPtr & msg)
  {
    if(!get_uav_state_) get_uav_state_ = true;

    tf::pointMsgToTF(msg->pose.pose.position, uav_position_);
    //for(int axis = 0; axis < 3; axis++)
    //uav_position_[axis] = msg->states[axis].state[state_mode_].x;

    uav_yaw_ = tf::getYaw(msg->pose.pose.orientation);
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

    if(joy_debug_)
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
    /* in terms of object{1} frame */
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
          tf::Vector3 delta((object_position_.x + object_offset_.x()) - uav_position_.x(), (object_position_.y + object_offset_.y()) - uav_position_.y(), 0.0);

          aerial_robot_msgs::FlightNav nav_msg;
          nav_msg.header.stamp = ros::Time::now();
          nav_msg.target = aerial_robot_msgs::FlightNav::BASELINK;
          nav_msg.pos_xy_nav_mode = aerial_robot_msgs::FlightNav::POS_MODE;
          nav_msg.target_pos_x = object_position_.x + object_offset_.x();
          nav_msg.target_pos_y = object_position_.y + object_offset_.y();
          nav_msg.pos_z_nav_mode = aerial_robot_msgs::FlightNav::NO_NAVIGATION;
          nav_msg.psi_nav_mode = aerial_robot_msgs::FlightNav::NO_NAVIGATION;
          if(object_head_direction_)
            {
              nav_msg.psi_nav_mode = aerial_robot_msgs::FlightNav::POS_MODE;
              nav_msg.target_psi = object_position_.theta + object_offset_.z();
              if(nav_msg.target_psi > M_PI) nav_msg.target_psi -= (2 * M_PI);
              if(nav_msg.target_psi < -M_PI) nav_msg.target_psi += (2 * M_PI);
            }
          uav_nav_pub_.publish(nav_msg);

          /* phase shift condition */
          bool approach = true;
          /* position check */
          if(delta.length() >  approach_pos_threshold_) approach = false;
          if(object_head_direction_)
            {
              float target_yaw = object_position_.theta + object_offset_.z();
              if(target_yaw > M_PI) target_yaw -= (2 * M_PI);
              if(target_yaw < -M_PI) target_yaw += (2 * M_PI);
              //ROS_INFO("DEBUG: check yaw, target_yaw:%f, y: %f, approach_yaw_threshold_: %f", target_yaw, y, approach_yaw_threshold_);
              if(fabs(target_yaw - uav_yaw_) > approach_yaw_threshold_) approach = false;
            }

            if(approach)
              {
                if(++cnt > (approach_count_ * func_loop_rate_))
                  {
                    ROS_WARN("Succeed to approach to object, shift to GRASPING_PHASE");
                    phase_ ++;
                    cnt = 0; // convergence reset
                    target_height_ = uav_position_.z(); //falling down init
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
          aerial_robot_msgs::FlightNav nav_msg;
	  nav_msg.header.stamp = ros::Time::now();
          nav_msg.target = aerial_robot_msgs::FlightNav::BASELINK;
          nav_msg.pos_xy_nav_mode = aerial_robot_msgs::FlightNav::POS_MODE;
          nav_msg.target_pos_x = uav_position_.x();
          nav_msg.target_pos_y = uav_position_.y();
          nav_msg.pos_z_nav_mode = aerial_robot_msgs::FlightNav::POS_MODE;
          nav_msg.target_pos_z = target_height_;
          nav_msg.psi_nav_mode = aerial_robot_msgs::FlightNav::NO_NAVIGATION;
          uav_nav_pub_.publish(nav_msg);

          if(fabs(box_point_.z + object_height_ + dropping_offset_ - uav_position_.z())  < 0.05) //0.05m, hard-coding
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
          tf::Vector3 delta(box_point_.x + box_offset_.x() - uav_position_.x(), box_point_.y  + box_offset_.y() - uav_position_.y(), 0.0);

          aerial_robot_msgs::FlightNav nav_msg;
          nav_msg.header.stamp = ros::Time::now();
          nav_msg.target = aerial_robot_msgs::FlightNav::BASELINK;
          nav_msg.pos_xy_nav_mode = aerial_robot_msgs::FlightNav::POS_MODE;
          nav_msg.target_pos_x = box_point_.x + box_offset_.x();
          nav_msg.target_pos_y = box_point_.y + box_offset_.y();
          nav_msg.pos_z_nav_mode = aerial_robot_msgs::FlightNav::NO_NAVIGATION;
          nav_msg.psi_nav_mode = aerial_robot_msgs::FlightNav::NO_NAVIGATION;
          uav_nav_pub_.publish(nav_msg);

          /* phase shift condition */
          if(delta.length() < transportation_threshold_)
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
          //tf::Vector3 delta(uav_init_position_.x() - uav_position_.x() , uav_init_position_.y() - uav_position_.y() , 0);

          aerial_robot_msgs::FlightNav nav_msg;
          nav_msg.header.stamp = ros::Time::now();
          nav_msg.target = aerial_robot_msgs::FlightNav::BASELINK;
          nav_msg.pos_xy_nav_mode = aerial_robot_msgs::FlightNav::POS_MODE;
          nav_msg.target_pos_x = uav_init_position_.x();
          nav_msg.target_pos_y = uav_init_position_.y();
          nav_msg.pos_z_nav_mode = aerial_robot_msgs::FlightNav::NO_NAVIGATION;
          nav_msg.psi_nav_mode = aerial_robot_msgs::FlightNav::NO_NAVIGATION;
          uav_nav_pub_.publish(nav_msg);

          phase_ = IDLE_PHASE;
          ROS_INFO("Shifht to IDLE_PHASE");
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
