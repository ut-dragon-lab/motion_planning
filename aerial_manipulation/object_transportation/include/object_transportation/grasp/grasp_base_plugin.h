#ifndef GRASP_BASE_PLUGIN_H
#define GRASP_BASE_PLUGIN_H

//* ros
#include <ros/ros.h>

#include <aerial_robot_base/basic_state_estimation.h>

namespace grasp_base_plugin
{
  class GraspBase
  {
  public:
    virtual void initialize(ros::NodeHandle nh, ros::NodeHandle nhp)  = 0;
    virtual ~GraspBase(){}

  protected:

    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;

    GraspBase(){}

    void baseRosParamInit()
    {
    }
  };

};

#endif
