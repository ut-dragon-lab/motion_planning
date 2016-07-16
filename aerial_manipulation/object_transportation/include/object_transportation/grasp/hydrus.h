#ifndef HYDRUS_H
#define HYDRUS_H

//* ros
#include <ros/ros.h>
#include <aerial_robot_base/basic_state_estimation.h>
#include <aerial_robot_base/sensor/sensor_base_plugin.h>

#include <aerial_robot_base/States.h>
#include <sensor_msgs/Joy.h>
#include <vector>

namespace grasp_plugin
{

  class Hydrus :public grasp_base_plugin::GraspBase
  {
  public:

    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp, ObjectTransportation* ob_trans)
    {
      nh_ = ros::NodeHandle(nh, "hydrus");
      nhp_ = ros::NodeHandle(nhp, "hydrus");

      ob_trans_ = ob_trans;

      rosParamInit();
    }

    ~Hydrus() {}
    Hydrus() {}

    static const int TIME_SYNC_CALIB_COUNT = 10;

    void drop()
    {
      drop_flag_ = true;
    }

  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;

    void joyCallback(const sensor_msgs::JoyConstPtr & joy_msg)
    {
    }

    void rosParamInit()
    {
      std::string ns = nhp_.getNamespace();
    }
  };
};
#endif












