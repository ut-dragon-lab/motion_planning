#ifndef ELETROMAGNET_H
#define ELETROMAGNET_H

//* ros
#include <ros/ros.h>
#include <aerial_robot_base/basic_state_estimation.h>
#include <aerial_robot_base/sensor/sensor_base_plugin.h>

#include <std_msgs/UInt8.h>
#include <sensor_msgs/Joy.h>
#include <vector>

namespace grasp_plugin
{

  class Eletromagnet :public grasp_base_plugin::GraspBase
  {
  public:

    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp, ObjectTransportation* ob_trans)
    {
      nh_ = ros::NodeHandle(nh, "eletromagnet");
      nhp_ = ros::NodeHandle(nhp, "eletromagnet");

      ob_trans_ = ob_trans;

      baseRosParamInit();
      rosParamInit();

      eletromagnet_pub_ = nh_.advertise<std_msgs::UInt8>(mag_control_pub_name_, 1);

      joy_stick_sub_ = nh_.subscribe<sensor_msgs::Joy>("/joy", 1, &Eletromagnet::joyCallback, this, ros::TransportHints().udp());

    }

    ~Eletromagnet() {}
    Eletromagnet() {}


    void start()
    {
      grasp_flag_ = true;
      cnt_ = 0;
    }

    void drop()
    {
      drop_flag_ = true;
      std_msgs::UInt8 drop_msg;
      drop_msg.data = 0;
      eletromagnet_pub_.publish(drop_msg);
    }

  private:
    ros::Publisher  grasp_pub_;
    ros::Publisher eletromagnet_pub_;
    ros::Subscriber eletromagnet_sub_;
    ros::Subscriber joy_stick_sub_;

    std::string grasp_pub_name_,mag_control_pub_name_;
    std::string eletromagnet_sub_name_;

    int switch_num_, grasp_num_, grasp_count_;
    int cnt_;

    bool debug_;

    void switchCallback(std_msgs::UInt8 switch_msg)
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

    void joyCallback(const sensor_msgs::JoyConstPtr & joy_msg)
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

    void rosParamInit()
    {
      std::string ns = nhp_.getNamespace();

      nhp_.param("grasp_pub_name", grasp_pub_name_, std::string("/grasp"));
      nhp_.param("mag_control_pub_name", mag_control_pub_name_, std::string("/mag_control"));
      nhp_.param("eletromagnet_sub_name", eletromagnet_sub_name_, std::string("/contact_status"));
      nhp_.param("switch_num", switch_num_, 5);
      nhp_.param("grasp_num", grasp_num_, 3); //min num to detect the contact between eletromagnet and object
      nhp_.param("grasp_count", grasp_count_, 5); //the count to decide to grasp, x 0.1sec(the publish hz is 10hz)
      nhp_.param("debug", debug_, false); //the count to decide to grasp, x 0.1sec(the publish hz is 10hz)
    }
  };
};
#endif
