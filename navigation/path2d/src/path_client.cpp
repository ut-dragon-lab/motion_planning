#include <path2d/path_client.h>


PathClient::PathClient(ros::NodeHandle nh, ros::NodeHandle private_nh):  nh_(nh), private_nh_(private_nh)
{

#ifdef USE_MOJU_PLUGIN
  points_sub_ = nh.subscribe<moju_rviz_plugins::Pose2Ds>("start_goal_points", 1, &PathClient::makePlanCallback, this, ros::TransportHints().tcpNoDelay());
#endif

  start_sub_ = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1, &PathClient::startPointCallback, this, ros::TransportHints().tcpNoDelay());
  goal_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("move_base_simple/goal", 1, &PathClient::goalPointCallback, this, ros::TransportHints().tcpNoDelay());

  distance_client_  = nh.serviceClient<path2d::PathDistance>("path_distance");

  state_ = START_POINT;
}


PathClient::~PathClient(){
}

#ifdef USE_MOJU_PLUGIN
void PathClient::twoPointsCallback(const moju_rviz_plugins::Pose2Ds points_msg)
{
  tf::Quaternion quat;
  quat.setRPY(0.0, 0.0, 0);
  start_point.header.frame_id = std::string("/map");
  start_point.pose.position.x = points_msg.points[0].x;
  start_point.pose.position.y = points_msg.points[0].y;
  start_point.pose.position.z = 0;
  tf::quaternionTFToMsg(quat, start_point.pose.orientation);
  goal_point.header.frame_id = std::string("/map");
  goal_point.pose.position.x = points_msg.points[1].x;
  goal_point.pose.position.y = points_msg.points[1].y;
  goal_point.pose.position.z = 0;
  tf::quaternionTFToMsg(quat, goal_point.pose.orientation);

  path2d::PathDistance path_distance_srv;
  path_distance_srv.request.start_point = start_point;
  path_distance_srv.request.goal_point = goal_point;

}
#endif

void PathClient::startPointCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& point_msg)
{
  start_point.header = point_msg->header;
  start_point.pose = point_msg->pose.pose;

  state_ = GOAL_POINT;
}

void PathClient::goalPointCallback(const geometry_msgs::PoseStampedConstPtr& point_msg)
{
  if( state_ == GOAL_POINT)
    {
      goal_point = *point_msg;
      // path distance calculation, ros service
      path2d::PathDistance path_distance_srv;
      path_distance_srv.request.start_point = start_point;
      path_distance_srv.request.goal_point = goal_point;

      if (distance_client_.call(path_distance_srv))
	{
	  ROS_INFO("Path Distance is: %f, One-line Distance is: %f", path_distance_srv.response.path_distance, path_distance_srv.response.straight_distance);
	}
      else
	{
	  ROS_ERROR("Failed to call service path distance");
	}

      state_ = START_POINT;
    }
}





int main(int argc, char** argv){
  ros::init(argc, argv, "path_client");

  ros::NodeHandle node_handle;
  ros::NodeHandle node_handle_private("~");
  PathClient* path_client = new PathClient(node_handle, node_handle_private);

  ros::spin();
  delete path_client;

  return(0);

}
