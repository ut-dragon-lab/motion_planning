#ifndef HYDRUS_H
#define HYDRUS_H

//* ros
#include <ros/ros.h>
#include <aerial_transportation/base.h> // plugin base class
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include <dynamixel_msgs/MotorStateList.h>
#include <vector>
#include <string>

// 50mm

namespace aerial_transportation
{
  typedef struct
  {
    double current_angle;
    double target_angle;
    double approach_angle; //the approach angle to the object
    double hold_angle; //the just angle to hold object
    double tighten_angle; //the extra angle to hold object tightly
    int holding_rotation_direction; //the rotation direction for grasping
    double load_rate;
    bool moving;
    double temperature; //reserve
    double angle_error; //reserve
  }JointControl;

  class Hydrus :public aerial_transportation::Base
   {
   public:

     void initialize(ros::NodeHandle nh, ros::NodeHandle nhp);

     ~Hydrus() {}
     Hydrus() {}

     static const uint8_t SUB_PHASE1 = 0;
     static const uint8_t SUB_PHASE2 = 1;
     static const uint8_t SUB_PHASE3 = 2;

   protected:
     /* overwrite */
     void joyStickAdditionalCallback(const sensor_msgs::JoyConstPtr & joy_msg);
     void graspPhase();
     void dropPhase();
     void objectPoseApproachOffsetCal();

   private:
     /* ros publisher & subscirber */
     ros::Publisher joint_ctrl_pub_;
     ros::Subscriber joint_states_sub_;
     ros::Subscriber joint_motors_sub_;

     /* rosparam based variables */
     //bool cheat_mode_;
     std::string joint_ctrl_pub_name_;
     std::string joint_states_sub_name_;
     std::string joint_motors_sub_name_;
     double object_approach_offset_x_; //the pos offset while going to grasp object
     double object_approach_offset_y_; //the pos offset while going to grasp object
     double pose_fixed_count_; //the convergence duration (sec)
     double grasping_height_threshold_; //the height condition to grasp to object
     double grasping_duration_; //the grasping motion time
     double torque_min_threshold_; //rate(0~1), the minimum torque to hold an object
     double torque_max_threshold_; //rate(0~1), the maximum torque to haod an object
     double modification_delta_angle_; //[rad]  slightly modify each joint angle to hold object
     double modification_duration_; //[sec] the duration to wait for result of modification
     double hold_count_; //the hold ok time count

     /* base variable */
     int joint_num_;
     int sub_phase_; // sub pahse for GRASPPING_PAHSE
     std::vector<aerial_transportation::JointControl> joints_control_; //TODO: should calculated from motion planning
     bool contact_; // the status of holding: ture means holding, false means not holding yet
     ros::Time holding_start_time_; //the start time to grasp object
     ros::Time modification_start_time_; //the start time to modification the grasp

     /* base function */
    void rosParamInit();
    void jointControlParamInit();
    void jointStatesCallback(const sensor_msgs::JointStateConstPtr& joint_states__msg); //get calibrated joints angle vector
    void jointMotorStatusCallback(const dynamixel_msgs::MotorStateListConstPtr& joint_motors_msg); //get the torque load nad temprature from each joint


  };
};
#endif
