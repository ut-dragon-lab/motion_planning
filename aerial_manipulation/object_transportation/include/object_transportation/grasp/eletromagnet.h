#ifndef ELETROMAGNET_H
#define ELETROMAGNET_H

//* ros
#include <ros/ros.h>
#include <aerial_robot_base/basic_state_estimation.h>
#include <aerial_robot_base/sensor/sensor_base_plugin.h>

#include <aerial_robot_base/States.h>
#include <std_msgs/UInt8.h>
#include <sensor_msgs/Joy.h>
#include <vector>

namespace grasp_plugin
{

  class Eletromagnet :public grasp_base_plugin::GraspBase
  {
  public:

    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp)
    {
      nh_ = ros::NodeHandle(nh, "eletromagnet");
      nhp_ = ros::NodeHandle(nhp, "eletromagnet");

      rosParamInit();

      eletromagnet_pub_ = nh_.advertise<std_msgs::UInt8>(mag_control_pub_name_, 1);

      joy_stick_sub_ = nh_.subscribe<sensor_msgs::Joy>("/joy", 1, &Eletromagnet::joyCallback, this, ros::TransportHints().udp());

    }

    ~Eletromagnet() {}
    Eletromagnet() {}

    static const int TIME_SYNC_CALIB_COUNT = 10;

  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;
    ros::Publisher  grasp_pub_;
    ros::Publisher eletromagnet_pub_;
    ros::Subscriber eletromagnet_sub_;
    ros::Subscriber joy_stick_sub_;

    std::string grasp_pub_name_,mag_control_pub_name_;
    std::string eletromagnet_sub_name_;

    int switch_num_;
    std::vector<bool> switch_status_;

    void switchCallback(std_msgs::UInt8 switch_msg)
    {
      //ROS_INFO("ok");
    }

    void joyCallback(const sensor_msgs::JoyConstPtr & joy_msg)
    {
      if(joy_msg->buttons[9] == 1)
        {//dropping
          std_msgs::UInt8 drop_msg;
          drop_msg.data = 0;
          eletromagnet_pub_.publish(drop_msg);
        }
    }

    void rosParamInit()
    {
      std::string ns = nhp_.getNamespace();

      nhp_.param("grasp_pub_name", grasp_pub_name_, std::string("/grasp"));
      nhp_.param("mag_control_pub_name", mag_control_pub_name_, std::string("/mag_control"));
      nhp_.param("eletromagnet_sub_name", eletromagnet_sub_name_, std::string("/contact_status"));
      nhp_.param("switch_num", switch_num_, 5);
    }
  };
};
#endif












