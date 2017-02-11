#ifndef HYDRUS_H
#define HYDRUS_H

/* ros */
#include <ros/ros.h>
#include <aerial_transportation/grasp_control/base.h> // plugin base class
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include <dynamixel_msgs/MotorStateList.h>
#include <std_msgs/UInt8.h>
#include <std_srvs/SetBool.h>
#include <vector>
#include <string>

/* grasp planning plugin */
#include <boost/shared_ptr.hpp>
#include <pluginlib/class_loader.h>
#include <aerial_transportation/grasp_planning/grasp_planner.h>

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
    int angle_error; //reserve
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

     static const uint8_t OVERLOAD_FLAG = 0x20;

   protected:
     /* overwrite */
     void joyStickAdditionalCallback(const sensor_msgs::JoyConstPtr & joy_msg);
     void graspPhase();
     void dropPhase();
     void objectPoseApproachOffsetCal();

   private:
     /* ros publisher & subscirber */
     ros::Publisher joint_ctrl_pub_;
     ros::Publisher aerial_grasping_flight_velocity_control_pub_; /* to change the gain of flight control LQI */
     ros::Subscriber joint_states_sub_;
     ros::Subscriber joint_motors_sub_;

     /* plugin for grasp planning */
     boost::shared_ptr<pluginlib::ClassLoader<grasp_planning::Base> > grasp_planning_loader_;
     boost::shared_ptr<grasp_planning::Base> grasp_planning_method_;

     /* rosparam based variables */
     std::string grasp_planning_plugin_name_;
     std::string aerial_grasping_flight_velocity_control_pub_name_;
     std::string joint_ctrl_pub_name_;
     std::string joint_states_sub_name_;
     std::string joint_motors_sub_name_;
     std::string overload_check_activate_srv_name_;

     double pose_fixed_count_; //the convergence duration (sec)
     double grasping_height_threshold_; //the height condition to grasp to object
     double grasping_duration_; //the grasping motion time
     double torque_min_threshold_; //rate(0~1), the minimum torque to hold an object
     /* deprecated */
     double torque_max_threshold_; //rate(0~1), the maximum torque to haod an object
     /* modification, when exceed the max torque threshold */
     double modification_delta_angle_; //[rad]  slightly modify each joint angle to hold object, which is based on the max change joint angle
     double modification_duration_; //[sec] the duration to wait for result of modification
     double hold_count_; //the hold ok time count
     double grasping_rate_; //the grasping joint vel rate time count
     double envelope_joint_angle_thre_; // the threshold for checking eveloping in term of joint angle
     double approach_delta_angle_; //the incerease amount of angle to approach the object(which is bigger than the angle vector of hold)
     double tighten_delta_angle_; //the tighten amount of angle to force-closure the object, which is based on the biggest torque joint.
     double release_delta_angle_; //the release amount of angle to force-closure the object, which is based on the biggest torque joint.


     /* base variable */
     int joint_num_;
     int sub_phase_; // sub pahse for GRASPPING_PAHSE
     std::vector<aerial_transportation::JointControl> joints_control_; //TODO: should calculated from motion planning
     double object_approach_offset_x_; //the pos offset while going to grasp object with the repect to the object{1} frame, in crontrol_cheat_mode, this is obtained by rosparam
     double object_approach_offset_y_; //the pos offset while going to grasp object with the repect to the object{1} frame, in crontrol_cheat_mode, this is obtained by rosparam
     double object_approach_offset_yaw_; //the yaw offset while going to grasp object with the repect to the object{1} frame, in crontrol_cheat_mode, this is obtained by rosparam
     ros::Time holding_start_time_; //the start time to grasp object
     ros::Time modification_start_time_; //the start time to modification the grasp
     /* the flag for check the enveloping grasp */
     /* 1. all joint torque is bigger than the min threshold, this will successively occur, when we shift to tighten angles*/
     /* 2. the error between predicted contact points at link and object is smaller than the max threshold */
     /* 3. the error between current angles and hold angles are smaller than the max threshold */
     bool envelope_closure_; 
     bool one_time_tighten_; //right now we just tighten once, no iteration!!
     bool force_closure_; // the status of holding: ture means holding, false means not holding yet
     /* envelop_closure_ states contains force_closure_ states */

     /* base function */
    void rosParamInit();
    void jointControlParamInit();
    void jointStatesCallback(const sensor_msgs::JointStateConstPtr& joint_states__msg); //get calibrated joints angle vector
    void jointMotorStatusCallback(const dynamixel_msgs::MotorStateListConstPtr& joint_motors_msg); //get the torque load nad temprature from each joint

     /* tools */
     std::vector<float> getApproachAngles()
     {
       std::vector<float> approach_angles(joint_num_);
        for(int i = 0; i < joint_num_; i++)
            approach_angles[i] = joints_control_[i].approach_angle;
        return approach_angles;
     }

     std::vector<float> getTightenAngles()
     {
       std::vector<float> tighten_angles(joint_num_);
        for(int i = 0; i < joint_num_; i++)
            tighten_angles[i] = joints_control_[i].tighten_angle;
        return tighten_angles;
     }

     std::vector<float> getHoldAngles()
     {
       std::vector<float> hold_angles(joint_num_);
        for(int i = 0; i < joint_num_; i++)
            hold_angles[i] = joints_control_[i].hold_angle;
        return hold_angles;
     }

     void  setApproachAngles(std::vector<float> approach_angles)
     {
        for(int i = 0; i < joint_num_; i++)
          joints_control_[i].approach_angle = approach_angles[i];
     }

     void  setTightenAngles(std::vector<float> tighten_angles)
     {
        for(int i = 0; i < joint_num_; i++)
          joints_control_[i].tighten_angle = tighten_angles[i];
     }

     void  setHoldAngles(std::vector<float> hold_angles)
     {
        for(int i = 0; i < joint_num_; i++)
          joints_control_[i].hold_angle = hold_angles[i];
     }

  };
};
#endif
