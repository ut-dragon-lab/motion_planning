#ifndef GRASP_BASE_PLUGIN_H
#define GRASP_BASE_PLUGIN_H

//* ros
#include <ros/ros.h>
#include <object_transportation/object_transportation.h> // Cross-reference

namespace grasp_base_plugin
{
  class GraspBase
  {
  public:
    virtual void initialize(ros::NodeHandle nh, ros::NodeHandle nhp, ObjectTransportation* ob_trans)  = 0;

    virtual ~GraspBase()
    {
      delete ob_trans_;
    }


    virtual void start()
    {
      grasp_flag_ = true;
    }

    virtual void drop() = 0;

  protected:

    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;

    ObjectTransportation* ob_trans_;
    bool grasp_flag_, drop_flag_;

    GraspBase(){}

    void baseRosParamInit()
    {
      grasp_flag_ = false;
      drop_flag_ = false;
    }
  };

};



#endif
