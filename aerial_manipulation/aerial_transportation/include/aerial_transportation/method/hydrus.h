#ifndef HYDRUS_H
#define HYDRUS_H

//* ros
#include <ros/ros.h>

#include <aerial_transportation/base.h> // plugin base class
#include <sensor_msgs/Joy.h>
#include <vector>

namespace grasp_plugin
{

  class Hydrus : :public aerial_transportation::Base
  {
  public:

    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp, ObjectTransportation* ob_trans)
    {
      rosParamInit();
    }

    ~Hydrus() {}
    Hydrus() {}

  private:
  };
};
#endif












