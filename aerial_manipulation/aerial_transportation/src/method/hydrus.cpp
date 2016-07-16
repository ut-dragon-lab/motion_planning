#include <aerial_transportation/method/eletromagnet.h>

namespace aerial_transportation
{
  void Eletromagnet::initialize(ros::NodeHandle nh, ros::NodeHandle nhp)
    {
      /* super class init */
      baseInit();

      /* ros param init */
      rosParamInit();

      /* ros pub sub init */
      grasp_pub_ = nh_.advertise<std_msgs::Empty>(grasp_pub_name_, 1);
      eletromagnet_pub_ = nh_.advertise<std_msgs::UInt8>(mag_control_pub_name_, 1);
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

    if(debug_)
      {
        if(joy_msg->buttons[10] == 1) // LEFT TOP TRIGGER(L1)
          {//start grasp
            grasp_flag_ = true;
          }
        if(joy_msg->buttons[8] == 1) // LEFT DOWM TRIGGER(L1)
          {//start drop
            drop_flag_ = true;
          }
      }
  }

  void Eletromagnet::switchCallback(std_msgs::UInt8 switch_msg)
  {
    uint8_t s1 = (switch_msg.data & 0x08) >> 3;
    uint8_t s2 = (switch_msg.data & 0x10) >> 4;
    uint8_t s3 = (switch_msg.data & 0x20) >> 5;
    uint8_t s4 = (switch_msg.data & 0x40) >> 6;
    uint8_t s5 = (switch_msg.data & 0x80) >> 7;

    ROS_INFO("contact status is %d, %d, %d, %d, %d", s1, s2, s3, s4, s5);
    if(grasp_flag_)
      {
        if(s1 + s2 + s3 + s4 + s5 > grasp_num_)
          cnt_ ++;
        else cnt_ = 0;

        if(cnt_ >= grasp_count_)
          {
            ROS_WARN("PICK THE OBJECT UP!! Shift to GRSPED_PHASE");
            std_msgs::Empty msg;
            grasp_pub_.publish(msg);
            grasp_flag_ = false;
            cnt_ = 0;
            ob_trans_->setPhase(ObjectTransportation::GRASPED_PHASE);
          }
      }
    if(drop_flag_)
      {
        if(s1 + s2 + s3 + s4 + s5 == 0)
          cnt_ ++;
        else cnt_ = 0;

        if(cnt_ >= grasp_count_)
          {
            ROS_WARN("OBJECT DROPPED!! Shift to RETURN_PHASE");
            drop_flag_ = false;
            cnt_ = 0;
            ob_trans_->setPhase(ObjectTransportation::RETURN_PHASE);
          }
      }
  }

  void Eletromagnet::rosParamInit()
  {
    std::string ns = nhp_.getNamespace();

  }

};

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(grasp_plugin::Hydrus, grasp_plugin::Base);
