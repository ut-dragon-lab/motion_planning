#ifndef ELETROMAGNET_H
#define ELETROMAGNET_H

//* ros
#include <ros/ros.h>
#include <aerial_transportation/base.h> // plugin base class
#include <std_msgs/UInt8.h>
#include <std_msgs/Empty.h>

namespace aerial_transportation
{
  class Eletromagnet :public aerial_transportation::Base
  {
  public:

    ~Eletromagnet() {}
    Eletromagnet() {}

    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp);

  protected:
    void joyStickAdditionalCallback(const sensor_msgs::JoyConstPtr & joy_msg);
    void graspPhase();
    void dropPhase();

  private:
    /* ros publisher & subscirber */
    ros::Publisher  grasp_pub_;
    ros::Publisher eletromagnet_pub_;
    ros::Subscriber eletromagnet_sub_;

    /* rosparam based variables */
    std::string grasp_pub_name_;
    std::string mag_control_pub_name_;
    std::string eletromagnet_sub_name_;
    int switch_num_; // the number of switches
    int grasped_num_; //min num to detect the contact between eletromagnet and object
    int grasped_count_; //the count to decide to grasp, x 0.1sec(the publish hz is 10hz)

    /* base function */
    void switchCallback(std_msgs::UInt8 switch_msg);
    void rosParamInit();
  };
};
#endif
