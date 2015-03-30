#include <path2d/2d_path.h>


Path2D::Path2D(ros::NodeHandle nh, ros::NodeHandle private_nh, tf::TransformListener& tf):  nh_(nh), private_nh_(private_nh)
{

  private_nh_.param("costmap_name", costmap_name_, std::string("test_costmap"));
  private_nh_.param("navfn_name", navfn_name_, std::string("test_navfn_planner"));

  private_nh_.param("use_wave_propagation", use_wave_propagation_, false);

  costmap_ = new costmap_2d::Costmap2DROS(costmap_name_, tf);

  if(use_wave_propagation_)
    {
      wave_propagation_ = new  wave_propagation::WavePropagationROS(navfn_name_, costmap_);
      ROS_INFO("use wave propagation algorithm");
    }
  else
    {
      navfn_ = new  navfn::NavfnROS(navfn_name_, costmap_);
      ROS_INFO("use navfn");
    }

  path_pub_ = nh.advertise<nav_msgs::Path>("path_planning/path",1);
#ifdef USE_MOJU_PLUGIN
  points_sub_ = nh.subscribe<moju_rviz_plugins::Pose2Ds>("start_goal_points", 1, &Path2D::makePlanCallback, this, ros::TransportHints().tcpNoDelay());
#endif


  distance_service_ = nh.advertiseService("path_distance", &Path2D::distanceService, this);

  state_ = START_POINT;
  two_points_setup_ = false;
}


Path2D::~Path2D(){
  delete costmap_;
  if(use_wave_propagation_)
    delete wave_propagation_;
  else
    delete navfn_; 
}


wave_propagation::WavePropagationROS*  Path2D::getWavePropagation()
{
  return wave_propagation_;
}


bool Path2D::distanceService(path2d::PathDistance::Request  &req,
                             path2d::PathDistance::Response &res)
{
  start_point = req.start_point;
  goal_point = req.goal_point;
  std::vector<geometry_msgs::PoseStamped> path_data;
  double time_before = ros::Time::now().toSec();

  if(use_wave_propagation_)
    wave_propagation_->makePlan(start_point, goal_point, path_data);
  else
    navfn_->makePlan(start_point, goal_point, path_data);
  double time_after = ros::Time::now().toSec();
  ROS_INFO("make plan takes %.3f [sec] to calucate path, %d points", time_after - time_before, (int)path_data.size());

  distanceCalc(path_data);
  res.path_distance = path_distance;
  res.straight_distance = straight_distance;
  
  return true;
}

double Path2D::distanceCalc(std::vector<geometry_msgs::PoseStamped> path_data)
{
  // path pubish
  nav_msgs::Path path_msg;
  path_msg.header.stamp = ros::Time::now();
  path_msg.header.frame_id = std::string("/map");
  path_msg.poses = path_data;
  path_pub_.publish(path_msg);
  /*
    for(int i = 0; i < path_data.size(); i++)
    path_msgs.poses.push_bach(path_data[i];
  */

  float total_dis = 0;
  int points =  path_data.size();

  for(int i = 0; i < points -1 ; i++)
    {
      float x1 = path_data[i].pose.position.x;
      float y1 = path_data[i].pose.position.y;
      float x2 = path_data[i + 1].pose.position.x;
      float y2 = path_data[i + 1].pose.position.y;
      float dis = distanceBetweenTwoPoints(x1, x2, y1, y2);
      total_dis += dis;

      //ROS_INFO("%d: total_dis: %f, dis: %f ", i, total_dis, dis);
    }

  path_distance = total_dis;

  float straight_dis 
    = distanceBetweenTwoPoints(start_point.pose.position.x, 
			       goal_point.pose.position.x, 
			       start_point.pose.position.y, 
			       goal_point.pose.position.y); 

  straight_distance = straight_dis;

  boost::unique_lock<boost::mutex> lock(obstacle_mutex_);
  two_points_setup_ = true;
  lock.unlock();

  ROS_INFO("the path distance is %f, straight distance is %f", total_dis, straight_dis);
  return path_distance;
}



float Path2D::distanceBetweenTwoPoints(float x1, float x2, float y1, float y2)
{
  return sqrt((x1 - x2) * (x1 -x2) + (y1 - y2) * (y1 - y2));
}

