#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include <pluginlib/class_loader.h>
#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <hydra_gap_passing/PlanningMode.h>
#include <hydra_transform_control/transform_control.h>
#include <aerial_robot_base/States.h>
#include <aerial_robot_base/FlightNav.h>

#include <ompl/control/SpaceInformation.h>
#include <ompl/base/StateStorage.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/config.h>

#include <iostream>
#include <valarray>
#include <limits>
#include <vector>
#include <sstream>

// file
#include <fstream>

#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>


struct configuration_space{
  std::vector<double> state_values; //x,y,theta,joints
  int stable_mode; //0: full-stable, 1: semi-stable
  int dist_thre_value; //0: bad, 1; good
  Eigen::MatrixXd k;
  float angle_cos;
  float angle_sin;
  int control_mode;
  configuration_space()
  {
    state_values.resize(6);
    stable_mode = 0;
    dist_thre_value = 0;
    //k = Eigen::MatrixXd::Zero(4, 12);
    angle_cos = 1;
    angle_sin = 0;
    control_mode = hydra_gap_passing::PlanningMode::POSITION_MODE;
  }

};
typedef struct configuration_space conf_values;


class MotionControl
{
  /*func
    1) log
      |- the path info 
      |-- joints entry
      |-- best cost/ calculation time
      |-- full/semi stable
      |- calculate the gains and rotates angles for each state
      |  path_states * (link_num * (size_of_float + 12state_gains * size_of_float))  : 1000 * (4* (4 + 12 * 4) ) = 208000bit
    2) control
     |- get control info
     |-- get the path and gains from file
     |-- get the path and gains realtimely

     |- get real-robot-state info
     |-- joints 
     |-- position 

     |- transform control
     |-- use the real joints values to search the best entry in the path/gains table, send to robot system(transform)
     - moving control
     -- use the real position to calculate the best velocity to robot system

   */
 public:
  MotionControl(ros::NodeHandle nh, ros::NodeHandle nhp, boost::shared_ptr<TransformController>  transform_controller);
  ~MotionControl();
  void planStoring(const ompl::base::StateStorage* plan_states, int planning_mode, const std::vector<double>& start_state, const std::vector<double>& goal_state, double best_cost, double calculation_time);

  void planFromFile();

  void getPlanningPath(std::vector<conf_values> & planning_path)
  {
    for(int i = 0; i < planning_path_.size(); i++)
      planning_path.push_back(planning_path_[i]);
  }

  conf_values getState(int index){      return planning_path_[index];  }
  inline int getPathSize(){return  planning_path_.size();}
  inline double getMotionCost(){return best_cost_;}
  inline float getPlanningTime(){return calculation_time_;}
  inline int getSemiStableStates(){return semi_stable_states_;}
  void getMinimumDist(std::vector<float>& min_dists)
  {
    min_dists[0] = minimum_x_performance_;
    min_dists[1] = minimum_y_performance_;
  }
  void getMinimumDistState(std::vector<int>& min_dist_state_indexs)
  {
    min_dist_state_indexs[0] = minimum_x_performance_state_;
    min_dist_state_indexs[1] = minimum_y_performance_state_;
  }

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;

  //control component
  ros::Publisher joint_cmd_pub_; //joints control
  ros::Publisher move_cmd_pub_; // move base control

  ros::Subscriber control_flag_sub_;
  ros::Subscriber joint_values_sub_;
  ros::Subscriber robot_states_sub_;

  //thread components
  boost::thread joint_cmd_thread_;
  boost::thread move_cmd_thread_;
  boost::thread gain_cmd_thread_;

  boost::mutex real_state_mutex_;

  boost::shared_ptr<TransformController> transform_controller_;


  bool control_flag_;
  double joint_cmd_rate_;
  double move_cmd_rate_;
  double gain_cmd_rate_;

  int control_index_;


  bool play_log_path_;//if true, use file, if false, get form realtime thing
  bool log_flag_;//log info to file?

  std::vector<conf_values> planning_path_;
  //std::vector<conf_values> real_robot_path_;

  std::string file_name_;

  //some additional 
  double best_cost_;
  int planning_mode_;
  double calculation_time_;
  int semi_stable_states_;
  float minimum_x_performance_, minimum_y_performance_;
  int minimum_x_performance_state_, minimum_y_performance_state_;

  //real state
  //aerial_robot_base::States robot_states_;
  std::vector<double> real_states_;

  //joint cmd
  int backward_offset_;
  int forward_offset_;


  void jointStateCallback(const sensor_msgs::JointStateConstPtr& joint_state);
  void robotStateCallback(const aerial_robot_base::StatesConstPtr& pose_state);
  void controlFlagCallback(const std_msgs::UInt8ConstPtr& control_msg);

  void getRealStates(std::vector<double>& real_states);
  void setMoveBaseStates(std::vector<double> move_base_states);
  void setJointStates(std::vector<double> joint_states);

  void jointCmd();
  void gainCmd();
  void moveCmd(); 
};

#endif
