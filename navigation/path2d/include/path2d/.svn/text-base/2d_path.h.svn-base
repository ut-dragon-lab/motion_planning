#ifndef PATH2D_H_
#define PATH2D_H_

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <navfn/navfn.h>
#include <navfn/navfn_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <path2d/PathDistance.h>
#include <wave_propagation/wave_propagation_ros.h>

#include <vector>
#include <string>
#include <cmath>



class Path2D {
public:
  Path2D(ros::NodeHandle nh, ros::NodeHandle private_nh, tf::TransformListener& tf);
 virtual ~Path2D();

 const static uint8_t START_POINT = 0;
 const static uint8_t GOAL_POINT = 0;

 wave_propagation::WavePropagationROS* getWavePropagation();
 double distanceCalc(std::vector< geometry_msgs::PoseStamped > path_data);

protected:
 ros::NodeHandle nh_,private_nh_;
 costmap_2d::Costmap2DROS* costmap_; //must be pointer if use class structure
 navfn::NavfnROS* navfn_; //must be pointer if use class structure
 wave_propagation::WavePropagationROS* wave_propagation_;

 
 std::string costmap_name_, navfn_name_;
 bool use_wave_propagation_;

 ros::Publisher  path_pub_;
 ros::Subscriber start_sub_, goal_sub_;
 ros::ServiceServer distance_service_;

 bool distanceService(path2d::PathDistance::Request  &req, path2d::PathDistance::Response &res);




 float distanceBetweenTwoPoints(float x1, float x2, float y1, float y2);

#ifdef USE_MOJU_PLUGIN
 ros::Subscriber points_sub_;
 void makePlanCallback(const moju_rviz_plugins::Pose2Ds points_msg);
#endif

 geometry_msgs::PoseStamped start_point, goal_point;
 float path_distance, straight_distance;
 uint8_t state_;

 bool two_points_setup_;

 boost::mutex obstacle_mutex_;

};

#endif

