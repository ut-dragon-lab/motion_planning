#include <ros/ros.h>
#include <jsk_quadcopter/SlamDebug.h>
#include <tf/transform_broadcaster.h>
#include <jsk_quadcopter/FlightNav.h>

class MoveBase
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;



public:
  MoveBase(ros::NodeHandle nh, ros::NodeHandle nhp)
    :nh_(nh), nhp_(nhp)
  {    
  }

  ~MoveBase()
  {
    
  }

  

};

