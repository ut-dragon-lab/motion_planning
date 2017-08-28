#ifndef HYDRUS_OBJECT_TRANSPORTATION_H
#define HYDRUS_OBJECT_TRANSPORTATION_H

/*
1. most of the consturctor of pointer should be shared pointer!!
  e.g. transform_controller
 */

#include <ros/ros.h>

// MoveIt!
#include <hydrus/transform_control.h>
#include <aerial_robot_base/States.h>
#include <aerial_robot_base/FlightNav.h>

#include <iostream>
#include <valarray>
#include <limits>
#include <vector>
#include <algorithm>
#include <string>
#include <boost/lexical_cast.hpp>
#include <sensor_msgs/JointState.h>
//#include <jsk_mbzirc_board/Magnet.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/PoseArray.h>
#include <aerial_robot_base/States.h>
#include <aerial_robot_base/State.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>

class HydrusObjectTransportation 
{
public:
  HydrusObjectTransportation(ros::NodeHandle nh, ros::NodeHandle nhp);
  ~HydrusObjectTransportation();
private:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;

  //publisher
  ros::Publisher joint_pub_;
  ros::Publisher uav_pos_pub_;
  ros::Publisher yaw_control_pub_;
  //for debug
  ros::Publisher state_machine_pub_;
  ros::Publisher object_world_pos_pub_; 
 
  //subscriber
  ros::Subscriber gripper_state_sub_[2];
  ros::Subscriber odom_sub_;
  ros::Subscriber target_image_center_sub_;
  ros::Subscriber camera_info_sub_;
  ros::Subscriber joy_stick_sub_;
  //for debug
  ros::Subscriber debug_sub_;

  //client
  ros::ServiceClient add_extra_module_client_;
  //ros::ServiceClient gripper_control_client_[2];

  //timer
  ros::Timer timer_;
  
  //callback functions
  void controlCallback(const ros::TimerEvent& event);
  void gripper0StateCallback(const std_msgs::Int16ConstPtr& msg);
  void gripper1StateCallback(const std_msgs::Int16ConstPtr& msg);
  void odomCallback(const aerial_robot_base::StatesConstPtr& msg);
  void targetImageCenterCallback(const geometry_msgs::PoseArrayConstPtr& msg);
  void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);
  void joyStickCallback(const sensor_msgs::JoyConstPtr& msg);
  void debugCallback(const std_msgs::EmptyConstPtr& msg);

  //topic names
  std::string joint_pub_topic_name_;
  std::string add_extra_module_service_name_;
  std::string gripper_control_service_name_[2];
  std::string gripper_state_sub_topic_name_[2];
  std::string odom_sub_topic_name_;
  std::string target_image_center_topic_name_;
  std::string uav_pos_pub_topic_name_;
  std::string camera_info_sub_topic_name_;
  std::string state_machine_pub_topic_name_;
  std::string joy_stick_sub_topic_name_;
  std::string debug_sub_topic_name_;
  std::string object_world_pos_pub_topic_name_;
  std::string yaw_control_pub_topic_name_;

  //ros params
  int control_frequency_;
  double link_length_;
  int gripper_link_num_[2];
  double gripper_link_offset_[2];
  int camera_link_num_;
  int object_num_; 
  int link_num_;
  double object_pos_thresh_; // m
  double go_down_vel_; // m/s
  double goal_pos_x_, goal_pos_y_;
  double searching_x_0_; 
  double searching_y_0_;
  double searching_x_1_;
  double searching_y_1_;
  double searching_z_; 
  double object_mass_;
  int root_link_;
  int object_search_time_thresh_;
  int object_detect_time_thresh_;
  int wait_time_thresh_;
  int zero_point_cnt_; 
  double pos_nav_thresh_;
  double vel_nav_limit_;
  double vel_nav_gain_;
 
  //member variables
  int gripper_state_[2];
  tf::Vector3 uav_w_;
  double uav_yaw_w_;
  enum class StateMachine{
    SEARCH_OBJECT_,
    APPROACH_TARGET_,
    GO_DOWN_UNTIL_TOUCH_,
    ADD_EXTRA_MODULE_,
    TRANSFORM_,
    SEARCH_GOAL_,
    APPROACH_GOAL_,
    THROW_OBJECT_,
  };
  StateMachine state_machine_;
  int detected_object_num_;
  std::vector<tf::Vector3> detected_object_pos_camera_;
  double joint_values_[3];
  int using_gripper_num_;
  tf::Vector3 target_object_w_;
  tf::Matrix3x3 camera_intrinsic_matrix_, camera_intrinsic_matrix_inv_;
  bool camera_info_update_;
  int object_search_cnt_;
  int object_detect_cnt_;
  std::vector<tf::Vector3> object_pos_vec_;
  bool start_flag_;
  int object_throw_cnt_;
  int transform_wait_cnt_;
  bool called_flag_;
  bool yaw_track_flag_;

  //member functions
  void throwObject();
  void goPos(tf::Vector3 target_pos);
  void addExtraModule(bool reset, int extra_module_link, double extra_module_mass, double extra_module_offset);

  //constant
  const double joint_values_with_no_object_[3] = {1.57, 1.57, 1.57};
  const double joint_values_with_one_object_[3] = {-0.70, 1.57, 1.57};
  const double joint_values_with_two_object_[3] = {1.57, 1.57, 1.57};
};

#endif
