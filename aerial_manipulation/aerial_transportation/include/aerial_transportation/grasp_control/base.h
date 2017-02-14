#ifndef BASE_PLUGIN_H
#define BASE_PLUGIN_H


/* ros */
#include <ros/ros.h>

#include <aerial_robot_base/FlightNav.h>
#include <aerial_robot_base/States.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_listener.h> /* for vector3 */

namespace aerial_transportation
{
  class Base
  {
  public:
    virtual void initialize(ros::NodeHandle nh, ros::NodeHandle nhp)  = 0;

    virtual ~Base() { }

    static const uint8_t IDLE_PHASE = 0;
    static const uint8_t APPROACH_PHASE = 1;
    static const uint8_t GRASPING_PHASE = 2;
    static const uint8_t GRASPED_PHASE = 3;
    static const uint8_t TRANSPORT_PHASE = 4;
    static const uint8_t DROPPING_PHASE = 5;
    static const uint8_t RETURN_PHASE = 6;

    static const uint8_t EGOMOTION_ESTIMATE = 0;
    static const uint8_t EXPERIMENT_ESTIMATE = 1;
    static const uint8_t GROUND_TRUTH = 2;


  protected:
    /* ros node */
    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;

    /* main thread */
    ros::Timer  func_timer_;

    /* ros publisher & subscirberrber */
    ros::Publisher uav_nav_pub_;
    ros::Subscriber uav_state_sub_;
    ros::Subscriber object_pos_sub_;
    ros::Subscriber joy_stick_sub_;

    /* rosparam based variables */
    bool joy_debug_;
    bool control_cheat_mode_;
    bool recog_cheat_mode_;
    std::string uav_nav_pub_name_;
    std::string uav_state_sub_name_;
    std::string object_pos_sub_name_;
    double func_loop_rate_;
    int state_mode_;
    double nav_vel_limit_;
    double vel_nav_threshold_;
    double vel_nav_gain_;
    double approach_pos_threshold_; // the pos convergence posecondition for object approach
    double approach_yaw_threshold_; // the yaw convergence condition for object approach
    double approach_count_; //the convergence duration (sec)
    bool object_head_direction_; //whether need to consider the head direction of object
    double falling_speed_; //the vel to fall down to object
    double grasping_height_offset_; //the offset between the bottom of uav and the top plat of object
    double ascending_speed_; //the vel to carry up to object
    double transportation_threshold_; // the convergence condition to carry to box
    double transportation_count_; // the convergence duration
    double dropping_offset_;  //the offset between the top of box and the bottom of object

    /* base variable */
    int phase_;
    int contact_cnt_;
    bool get_uav_state_;
    bool object_found_;
    double target_height_;
    tf::Vector3 uav_position_;
    float uav_yaw_;
    tf::Vector3 uav_init_position_; // not important
    geometry_msgs::Pose2D object_position_;

    /* config of target object */
    tf::Vector3 object_offset_; //[x,y,psi], the offset from the COG of object to the position control point of uav with the respect to world frame
    double object_height_; // in cheat mode, this is obtained by rosparam
    /* config of recycle box */
    geometry_msgs::Point box_point_; // in cheat mode, this is obtained by rosparam
    tf::Vector3 box_offset_; // in cheat mode, this is obtained by rosparam


    /* base function */
    void baseInit(ros::NodeHandle nh, ros::NodeHandle nhp);
    void mainFunc(const ros::TimerEvent & e);
    virtual void graspPhase() = 0;
    virtual void dropPhase() = 0;

    virtual void rosParamInit() = 0;
    void baseRosParamInit();

    void stateCallback(const aerial_robot_base::StatesConstPtr & msg);
    void objectPoseCallback(const geometry_msgs::Pose2DConstPtr & object_msg);
    virtual void objectPoseApproachOffsetCal() {};
    void joyStickCallback(const sensor_msgs::JoyConstPtr & joy_msg);
    virtual void joyStickAdditionalCallback(const sensor_msgs::JoyConstPtr & joy_msg){}

  };

};

#endif  // OBJECT_TRANSPORTATION_H
