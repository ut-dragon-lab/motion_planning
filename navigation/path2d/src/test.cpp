#include "ros/ros.h"
#include <path2d/PathDistance.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_client");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<path2d::PathDistance>("path_distance");
  path2d::PathDistance srv;
  nav_msgs::Path path;

  
  for(int i = 0; i < 10; i++)
    {
      geometry_msgs::PoseStamped point;
      point.pose.position.x = 0.01 + i * 0.01;
      point.pose.position.y = 0;
      path.poses.push_back(point);
    }
  srv.request.path = path;


  if (client.call(srv))
    {
      ROS_INFO("Distance: %f", srv.response.distance);
    }
  else
    {
      ROS_ERROR("Failed to call service add_two_ints");
      return 1;
    }

  return 0;
}
