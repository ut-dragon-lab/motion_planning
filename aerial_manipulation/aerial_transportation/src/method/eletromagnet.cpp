#include <aerial_transportation/method/eletromagnet.h>

namespace aerial_transportation
{
  void Eletromagnet::initialize(ros::NodeHandle nh, ros::NodeHandle nhp)
    {
      /* super class init */
      baseInit(nh, nhp);

      /* ros param init */
      rosParamInit();

      /* ros pub sub init */
      grasp_pub_ = nh_.advertise<std_msgs::Empty>(grasp_pub_name_, 1);
      eletromagnet_pub_ = nh_.advertise<std_msgs::UInt8>(mag_control_pub_name_, 1);
      eletromagnet_sub_ = nh_.subscribe<std_msgs::UInt8>(eletromagnet_sub_name_, 1, &Eletromagnet::switchCallback, this, ros::TransportHints().udp());

    }

  void Eletromagnet::joyStickAdditionalCallback(const sensor_msgs::JoyConstPtr & joy_msg)
  {
    if(joy_msg->buttons[9] == 1) // RIGHT TOP TRIGGER(R1)
      {//dropping(disable eletomagnet)
        std_msgs::UInt8 drop_msg;
        drop_msg.data = 0;
        eletromagnet_pub_.publish(drop_msg);
      }
    if(joy_msg->buttons[11] == 1) // RIGHT DOWM TRIGGER(R1)
      {//set eletromagnet
        std_msgs::UInt8 set_msg;
        set_msg.data = 1;
        eletromagnet_pub_.publish(set_msg);
      }

  }

  void Eletromagnet::switchCallback(std_msgs::UInt8 switch_msg)
  {
    uint8_t s1 = (switch_msg.data & 0x08) >> 3;
    uint8_t s2 = (switch_msg.data & 0x10) >> 4;
    uint8_t s3 = (switch_msg.data & 0x20) >> 5;
    uint8_t s4 = (switch_msg.data & 0x40) >> 6;
    uint8_t s5 = (switch_msg.data & 0x80) >> 7;

    //ROS_INFO("contact status is %d, %d, %d, %d, %d", s1, s2, s3, s4, s5);
    switch(phase_)
      {
      case GRASPING_PHASE:
        {
          if(s1 + s2 + s3 + s4 + s5 >= grasped_num_) contact_cnt_ ++;
          else contact_cnt_ = 0;

          if(contact_cnt_ >= grasped_count_)
            {
              ROS_WARN("Pick the object up!! Shift to GRSPED_PHASE");
              phase_ = GRASPED_PHASE;
              contact_cnt_ = 0;
	      target_height_ = uav_position_.position.z;

              std_msgs::Empty msg;
              grasp_pub_.publish(msg);
            }
          break;
        }
      case DROPPING_PHASE:
        {
          if(s1 + s2 + s3 + s4 + s5 == 0) contact_cnt_ ++;
          else contact_cnt_ = 0;

          if(contact_cnt_ >= grasped_count_)
            {
              ROS_WARN("Object dropped!! Shift to RETURN_PHASE");
              contact_cnt_ = 0;
              phase_ = RETURN_PHASE;

	      //reset eletromagnet status
	      std_msgs::UInt8 reset_msg;
	      reset_msg.data = 1;
	      eletromagnet_pub_.publish(reset_msg);
            }
          break;
        }
      default:
        break;
    }
  }

  void Eletromagnet::graspPhase()
  {
    static bool once_flag = true;

    if(once_flag)
      {
	//reset eletromagnet status
	std_msgs::UInt8 reset_msg;
	reset_msg.data = 1;
	eletromagnet_pub_.publish(reset_msg);
	once_flag = false;
      }

    /* height calc part */
    target_height_ -= (falling_speed_ / func_loop_rate_);
    if(target_height_ < (object_height_ + grasping_height_offset_))
      target_height_ = object_height_ + grasping_height_offset_;

    /* send nav msg */
    aerial_robot_base::FlightNav nav_msg;
    nav_msg.header.stamp = ros::Time::now();
    nav_msg.pos_xy_nav_mode = aerial_robot_base::FlightNav::POS_MODE;
    nav_msg.target_pos_x = object_position_.x + object_offset_.x();
    nav_msg.target_pos_y = object_position_.y + object_offset_.y();
    nav_msg.pos_z_nav_mode = aerial_robot_base::FlightNav::POS_MODE;
    nav_msg.target_pos_z = target_height_;
    nav_msg.psi_nav_mode = aerial_robot_base::FlightNav::NO_NAVIGATION;
    uav_nav_pub_.publish(nav_msg);
  }

  void Eletromagnet::dropPhase()
  {
    static double prev_time = 0;

    /* send command per  sec */
    if(ros::Time::now().toSec() - prev_time > 0)
      {
        std_msgs::UInt8 drop_msg;
        drop_msg.data = 0;
        eletromagnet_pub_.publish(drop_msg);
        prev_time = ros::Time::now().toSec();
      }
  }

  void Eletromagnet::rosParamInit()
  {
    std::string ns = nhp_.getNamespace();

    nhp_.param("grasp_pub_name", grasp_pub_name_, std::string("/grasp"));
    nhp_.param("mag_control_pub_name", mag_control_pub_name_, std::string("/mag_control"));
    nhp_.param("eletromagnet_sub_name", eletromagnet_sub_name_, std::string("/contact_status"));
    nhp_.param("switch_num", switch_num_, 5);
    nhp_.param("grasped_num", grasped_num_, 3); 
    nhp_.param("grasped_count", grasped_count_, 5); 
  }

};

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_transportation::Eletromagnet, aerial_transportation::Base);
