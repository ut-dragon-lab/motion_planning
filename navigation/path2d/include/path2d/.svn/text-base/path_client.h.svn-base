#ifndef PATH_CLIENT_H_
#define PATH_CLIENT_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <path2d/PathDistance.h>

//#define USE_MOJU_PLUGIN
#ifdef USE_MOJU_PLUGIN
#include <moju_rviz_plugins/Pose2Ds.h> //not good
#endif

#include <vector>
#include <string>
#include <cmath>

class PathClient {
public:
  PathClient(ros::NodeHandle nh, ros::NodeHandle private_nh);
 virtual ~PathClient();

 const static uint8_t START_POINT = 0;
 const static uint8_t GOAL_POINT = 0;

private:
 ros::NodeHandle nh_,private_nh_;

 ros::Subscriber start_sub_, goal_sub_;
 ros::ServiceClient distance_client_;

 void startPointCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& point_msg);
 void goalPointCallback(const geometry_msgs::PoseStampedConstPtr& point_msg);

#ifdef USE_MOJU_PLUGIN
 ros::Subscriber points_sub_;
 void twoPointsCallback(const moju_rviz_plugins::Pose2Ds points_msg);
#endif

 geometry_msgs::PoseStamped start_point, goal_point;
 uint8_t state_;

};

#endif

