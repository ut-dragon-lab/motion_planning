#ifndef OBJECT_TRANSPORTATION_H
#define OBJECT_TRANSPORTATION_H

/* ros */
#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <pluginlib/class_loader.h>


#include <nav_msgs/Odometry.h>
#include <aerial_robot_base/FlightNav.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_listener.h> /* for vector3 */

namespace grasp_base_plugin
{
class GraspBase;
};

class ObjectTransportation
{
public:
  ObjectTransportation(ros::NodeHandle nh, ros::NodeHandle nhp);
  ~ObjectTransportation(){};

  static const uint8_t IDLE_PHASE = 0;
  static const uint8_t APPROACH_PHASE = 1;
  static const uint8_t GRASPING_PHASE = 2;
  static const uint8_t GRASPED_PHASE = 3;
  static const uint8_t TRANSPORT_PHASE = 4;
  static const uint8_t DROPPING_PHASE = 5;
  static const uint8_t RETURN_PHASE = 6;

  inline int getPhase() {return phase_;}
  inline void  setPhase(int phase){phase_ = phase;}

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  ros::Timer  func_timer_;

  ros::Publisher uav_nav_pub_;
  ros::Subscriber uav_state_sub_;
  ros::Subscriber object_pos_sub_;
  ros::Subscriber joy_stick_sub_;

  double func_loop_rate_;
  geometry_msgs::Point box_point_;
  std::string grasp_plugin_name_;
  std::string uav_nav_pub_name_;
  std::string uav_state_sub_name_;
  std::string object_pos_sub_name_;
  double nav_vel_limit_;
  double vel_nav_threshold_;
  double vel_nav_gain_;
  double falling_speed_; //the vel to fall down to object
  double target_height_;
  double grasping_height_offset_; //the offset between the bottom of uav and the top plat of object
  double object_height_; //this should be detected!!!!

  double approach_threshold_; // the convergence condition for object approach
  double approach_count_; //the convergence duration (sec)
  double transportation_threshold_; // the convergence condition to carry to box
  double transportation_count_; // the convergence duration
  double dropping_offset_;  //the offset between the top of box and the bottom of object

  int phase_;
  bool uav_state_;
  bool object_found_;
  geometry_msgs::Pose uav_position_;
  geometry_msgs::Pose uav_init_position_; // not important
  geometry_msgs::Pose2D object_position_;

  void rosParamInit();

  void stateCallback(const nav_msgs::OdometryConstPtr & msg);
  void objectPoseCallback(const geometry_msgs::Pose2DConstPtr & object_msg);

  void joyStickCallback(const sensor_msgs::JoyConstPtr & joy_msg);

  boost::shared_ptr< pluginlib::ClassLoader<grasp_base_plugin::GraspBase> > grasp_loader_ptr_;

  boost::shared_ptr<grasp_base_plugin::GraspBase> grasp_method_;

  void mainFunc(const ros::TimerEvent & e);
};



#endif  // OBJECT_TRANSPORTATION_H
