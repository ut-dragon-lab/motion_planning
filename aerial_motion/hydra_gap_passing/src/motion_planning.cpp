/***
TODO list:
1. right planning for ku-model
2. path plannig from start to first access point and final point to goal => path inclination

3.the trajectory of ellispe and parabola is wired, and the curve of parabola and ellipse is also wired
4. temporarily change the lenght of link(0.475m) => the model of multi-rotor should be linked with transform_control.cpp
5. change the model of hydra
 ***/

/* Author: Sachin Chitta */
#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <collision_check/correlation_coefficient_check.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/StateStorage.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/config.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/DiscreteMotionValidator.h>

#include <hydra/transform_control.h>
#include <jsk_quadcopter/SlamDebug.h>


//#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
//#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <iostream>
#include <valarray>
#include <limits>
#include <vector>

class SinpleGapEnv
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  planning_scene::PlanningScene* planning_scene_;
  
};

struct configuration_space{
  float x;
  float y;
  float theta;
  float joint1;
  float joint2;
  float joint3;
};
typedef struct configuration_space conf_values;

class PlanningScene
{
private:

  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  ros::Publisher planning_scene_diff_pub_;
  ros::Subscriber pose_sub_;
  ros::Publisher  flight_nav_;

  //+++ hydra
  TransformController*  transform_controller_;

  robot_model_loader::RobotModelLoader *robot_model_loader_;
  robot_model::RobotModelPtr kinematic_model_;
  planning_scene::PlanningScene* planning_scene_;
  //planning_scene::PlanningScenePtr planning_scene_;
  moveit_msgs::CollisionObject collision_object_;
  moveit_msgs::PlanningScene planning_scene_msg_;
  collision_detection::AllowedCollisionMatrix acm_;

  double  collision_detection_rate_;
  ros::Timer collision_detection_timer_;
  float calculation_time_;

  double tolerance_;

  //real robot 
  bool real_robot_move_base_;
  bool get_init_state_;
  double mocap_center_to_joint_x_, mocap_center_to_joint_y_;

  //gap env
  //int gap_overlap_type_;
  double gap_left_x_, gap_left_y_;
  double gap_x_offset_;
  double gap_y_offset_;
  double gap_left_width_;
  double gap_right_width_;
  tf::Vector3 left_half_corner;
  tf::Vector3 right_half_corner;

  //original planning
  std::vector<conf_values> original_path_;
  std::vector<conf_values> real_robot_path_;
  std::vector<double> start_state_;
  std::vector<double> goal_state_;


  //ompl 
  //ompl::base::CompoundStateSpace* hydra_space_;
  ompl::base::StateSpacePtr hydra_space_;
  ompl::base::SpaceInformationPtr space_information_;
  ompl::base::PathPtr path_;
  bool solved_;

  //visualization
  int state_index_;
  ompl::base::StateStorage* plan_states_;
  int planning_mode_;
  int ompl_mode_;
  double coefficient_rate_;



  void collisionFunc(const ros::TimerEvent &e)
  {
    static float minimum_x_performance = 1e6;
    static float minimum_y_performance = 1e6;
    static float joint1_x = 0, joint2_x = 0, joint3_x = 0;
    static float joint1_y = 0, joint2_y = 0, joint3_y = 0;

      if(solved_)
        {
          robot_state::RobotState& robot_state = planning_scene_->getCurrentStateNonConst();
          std::vector<double> joint_values;
          robot_state.getStateValues(joint_values);

          if(planning_mode_ == ORIGINAL_MODE)
            {
              ROS_INFO("size is %d", (int)original_path_.size());
              if(state_index_ == (int)original_path_.size())
                state_index_ = 0;

              joint_values[0] = original_path_[state_index_].x;
              joint_values[1] = original_path_[state_index_].y;
              joint_values[2] = original_path_[state_index_].theta;
              joint_values[3] = original_path_[state_index_].joint1;
              joint_values[4] = original_path_[state_index_].joint2;
              joint_values[5] = original_path_[state_index_].joint3;
            }
          else
            {
              ROS_INFO("size is %d", (int)plan_states_->size());
              if(state_index_ == (int)plan_states_->size())
                state_index_ = 0;
              if(planning_mode_ == ONLY_JOINTS_MODE)
                {
                  joint_values[3] = plan_states_->getState(state_index_)->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
                  joint_values[4] = plan_states_->getState(state_index_)->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];
                  joint_values[5] = plan_states_->getState(state_index_)->as<ompl::base::RealVectorStateSpace::StateType>()->values[2];

                  correlationCoefficientCheck(joint_values);


                  float max_x_length = 0, max_y_length = 0;
                  for(int i = 0; i < (int)transform_controller_->links_origin_from_cog_.size(); i ++)
                    {
                      ROS_INFO("x:%f, y:%f",transform_controller_->links_origin_from_cog_[i](0), transform_controller_->links_origin_from_cog_[i](1));
                      if(max_x_length < fabs(transform_controller_->links_origin_from_cog_[i](0)))
                        max_x_length = fabs(transform_controller_->links_origin_from_cog_[i](0));             
                      if(max_y_length < fabs(transform_controller_->links_origin_from_cog_[i](1)))
                        max_y_length = fabs(transform_controller_->links_origin_from_cog_[i](1));             
                    }
                  if(max_x_length < minimum_x_performance)
                    {
                      minimum_x_performance = max_x_length;
                      joint1_x = joint_values[3];
                      joint2_x = joint_values[4];
                      joint3_x = joint_values[5];
                    }
                  if(max_y_length < minimum_y_performance)
                    {
                      minimum_y_performance = max_y_length;
                      joint1_y = joint_values[3];
                      joint2_y = joint_values[4];
                      joint3_y = joint_values[5];
                    }


                }
              else if(planning_mode_ == ONLY_BASE_MODE || planning_mode_ == ONLY_BASE_MODE + ORIGINAL_MODE)
                {

                  joint_values[0] = plan_states_->getState(state_index_)->as<ompl::base::SE2StateSpace::StateType>()->getX();
                  joint_values[1] = plan_states_->getState(state_index_)->as<ompl::base::SE2StateSpace::StateType>()->getY();
                  joint_values[2] = plan_states_->getState(state_index_)->as<ompl::base::SE2StateSpace::StateType>()->getYaw();

                }
              else if(planning_mode_ == JOINTS_AND_BASE_MODE)
                {
                  const ompl::base::CompoundState* state_tmp = dynamic_cast<const ompl::base::CompoundState*>(plan_states_->getState(state_index_));
                  joint_values[0] = state_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->getX();
                  joint_values[1] = state_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->getY();
                  joint_values[2] = state_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->getYaw();
                  joint_values[3] = state_tmp->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[0];
                  joint_values[4] = state_tmp->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[1];
                  joint_values[5] = state_tmp->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[2];

                  correlationCoefficientCheck(joint_values);


                  float max_x_length = 0, max_y_length = 0;
                  for(int i = 0; i < (int)transform_controller_->links_origin_from_cog_.size(); i ++)
                    {
                      ROS_INFO("x:%f, y:%f",transform_controller_->links_origin_from_cog_[i](0), transform_controller_->links_origin_from_cog_[i](1));
                      if(max_x_length < fabs(transform_controller_->links_origin_from_cog_[i](0)))
                        max_x_length = fabs(transform_controller_->links_origin_from_cog_[i](0));             
                      if(max_y_length < fabs(transform_controller_->links_origin_from_cog_[i](1)))
                        max_y_length = fabs(transform_controller_->links_origin_from_cog_[i](1));             
                    }
                  if(max_x_length < minimum_x_performance)
                    {
                      minimum_x_performance = max_x_length;
                      joint1_x = joint_values[3];
                      joint2_x = joint_values[4];
                      joint3_x = joint_values[5];
                    }
                  if(max_y_length < minimum_y_performance)
                    {
                      minimum_y_performance = max_y_length;
                      joint1_y = joint_values[3];
                      joint2_y = joint_values[4];
                      joint3_y = joint_values[5];
                    }
                }
            }
          ROS_INFO("index is %d, calculation time is %f, minimum_x_performance is %f, minimum_y_performance is %f", state_index_, calculation_time_, minimum_x_performance, minimum_y_performance);
          ROS_INFO("joint1_x :%f, joint2_x :%f,joint3_x :%f, joint1_y :%f, joint2_y :%f,joint3_y :%f", joint1_x, joint2_x, joint3_x, joint1_y, joint2_y, joint3_y);

          state_index_ ++;

          robot_state.setStateValues(joint_values);

        }

      planning_scene_->getPlanningSceneMsg(planning_scene_msg_);
      planning_scene_msg_.is_diff = true;
      planning_scene_diff_pub_.publish(planning_scene_msg_);
  }

  bool correlationCoefficientCheck(std::vector<double> joint_values)
  {
    //temporary for quad-type
    std::vector<tf::Vector3> origins;
    std::vector<tf::StampedTransform>  transforms;
    transforms.resize(transform_controller_->getLinkNum());
    float length = transform_controller_->getLinkLength(); 

    transforms[0].setOrigin( tf::Vector3(-length/2, 0, 0) );
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    transforms[0].setRotation(q);

    transforms[1].setOrigin( tf::Vector3(length/2 * cos(joint_values[3]), 
                                         length/2 * sin(joint_values[3]), 0) );
    q.setRPY(0, 0, joint_values[3]);
    transforms[1].setRotation(q);

    transforms[2].setOrigin( tf::Vector3(length/2 * cos(joint_values[3] + joint_values[4]) + length * cos(joint_values[3]), 
                                         length/2 * sin(joint_values[3] + joint_values[4]) + length * sin(joint_values[3]), 0) );
    q.setRPY(0, 0, joint_values[3] + joint_values[4]);
    transforms[2].setRotation(q);

    transforms[3].setOrigin( tf::Vector3(length/2 * cos(joint_values[3] + joint_values[4] + joint_values[5]) + length * cos(joint_values[3] + joint_values[4]) + length * cos(joint_values[3]), 
                                         length/2 * sin(joint_values[3] + joint_values[4] + joint_values[5]) + length * sin(joint_values[3] + joint_values[4]) + length * sin(joint_values[3]), 0) );
    q.setRPY(0, 0, joint_values[3] + joint_values[4] + joint_values[5]);
    transforms[3].setRotation(q);

    transform_controller_->cogComputation(transforms);
      
    bool no_continuous_flag = false;
    transform_controller_->principalInertiaComputation(transforms, no_continuous_flag);
    
    //1bit: roll+, 2bit: roll-, 3bit: pitch+, 4bit:pitch-
    uint8_t propeller_distrubution = 0x00;
    for(int i = 0; i < (int)transform_controller_->links_origin_from_cog_.size(); i ++)
      {
        if(transform_controller_->links_origin_from_cog_[i](0) > coefficient_rate_) //prev: 0.06
          {//harcode, 0.06m is the propeller diameter
            propeller_distrubution |= 0x01;
            
          }
        if(transform_controller_->links_origin_from_cog_[i](0) < -coefficient_rate_)
          {//harcode, 0.06m is the propeller diameter
            propeller_distrubution |= 0x02;
          }
        if(transform_controller_->links_origin_from_cog_[i](1) > coefficient_rate_)
          {//harcode, 0.06m is the propeller diameter
            propeller_distrubution |= 0x04;
          }
        if(transform_controller_->links_origin_from_cog_[i](1) < -coefficient_rate_)
          {//harcode, 0.06m is the propeller diameter
            propeller_distrubution |= 0x08;
          }
        //ROS_INFO("x:%f, y:%f",transform_controller_->links_origin_from_cog_[i](0), transform_controller_->links_origin_from_cog_[i](1));
      }
    if(propeller_distrubution != 15)
      {
        //ROS_WARN("bad correlation" );
        return true;
      }
    return false;
  }

  bool isStateValid(const ompl::base::State *state)
  {
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    robot_state::RobotState& robot_state = planning_scene_->getCurrentStateNonConst();
    std::vector<double> joint_values;
    robot_state.getStateValues(joint_values);

    double x,y,theta,angle1, angle2,angle3;

    if(planning_mode_ == ONLY_JOINTS_MODE)
      {
         angle1 = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
         angle2 = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];
         angle3 = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[2];
         joint_values[3] = angle1;
         joint_values[4] = angle2;
         joint_values[5] = angle3;

      }
    else if(planning_mode_ == ONLY_BASE_MODE || planning_mode_ == ONLY_BASE_MODE + ORIGINAL_MODE)
      {
         x = state->as<ompl::base::SE2StateSpace::StateType>()->getX();
         y = state->as<ompl::base::SE2StateSpace::StateType>()->getY();
         theta = state->as<ompl::base::SE2StateSpace::StateType>()->getYaw();
         joint_values[0] = x;
         joint_values[1] = y;
         joint_values[2] = theta;
      }
    else if(planning_mode_ == JOINTS_AND_BASE_MODE)
      {
        const ompl::base::CompoundState* state_tmp = dynamic_cast<const ompl::base::CompoundState*>(state);

         x = state_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->getX();
         y = state_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->getY();
         theta = state_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->getYaw();
         angle1 = state_tmp->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[0];
         angle2 = state_tmp->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[1];
         angle3 = state_tmp->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[2];
         joint_values[0] = x;
         joint_values[1] = y;
         joint_values[2] = theta;
         joint_values[3] = angle1;
         joint_values[4] = angle2;
         joint_values[5] = angle3;
      }

    robot_state.setStateValues(joint_values);

    //check the correlation coefficient
    if(planning_mode_ == JOINTS_AND_BASE_MODE || planning_mode_ == ONLY_JOINTS_MODE)
      {
        bool bad_correlation_coefficient = correlationCoefficientCheck(joint_values);
        if(bad_correlation_coefficient) return false;
        if(planning_mode_ == ONLY_JOINTS_MODE) return true;
      }

    //check collision
    planning_scene_->checkCollision(collision_request, collision_result, robot_state, acm_);

    if(collision_result.collision) return false;
    else return true;
  }

  ompl::base::OptimizationObjectivePtr getPathLengthObjective(const ompl::base::SpaceInformationPtr& si)
  {
    return ompl::base::OptimizationObjectivePtr(new ompl::base::PathLengthOptimizationObjective(si));
  }

  // ompl::base::OptimizationObjectivePtr getThresholdPathLengthObj(const ompl::base::SpaceInformationPtr& si)
  // {
  //   ompl::base::OptimizationObjectivePtr obj(new ompl::base::PathLengthOptimizationObjective(si));
  //   obj->setCostThreshold(ompl::base::Cost(1.51));
  //   return obj;
  // }

  float distance(float x1, float x2, float y1, float y2)
  {
    return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
  }

public:
  static const int ONLY_JOINTS_MODE = 0;
  static const int ONLY_BASE_MODE = 1;
  static const int JOINTS_AND_BASE_MODE = 2;
  static const int 
ORIGINAL_MODE = 3;

  static const int RRT_START_MODE = 0;
  static const int LBKPIECE1_MODE = 1;

  static const int NO_OVERLAP = 0; 
  static const int Y_AXIS_OVERLAP = 1; //gap coord
  static const int X_AXIS_OVERLAP = 2; //gap coord

  PlanningScene(ros::NodeHandle nh, ros::NodeHandle nhp)
    :nh_(nh), nhp_(nhp)
  {    
    planning_scene_diff_pub_ = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

    //gap env
    nhp_.param("gap_left_x", gap_left_x_, 1.0); 
    nhp_.param("gap_left_y", gap_left_y_, 0.3); 
    nhp_.param("gap_x_offset", gap_x_offset_, 0.6); //minus: overlap
    nhp_.param("gap_y_offset", gap_y_offset_, 0.0); //minus: overlap
    nhp_.param("gap_left_width", gap_left_width_, 0.3); //minus: bandwidth
    nhp_.param("gap_right_width", gap_right_width_, 0.3); //minus: bandwidth

    nhp_.param("coefficient_rate", coefficient_rate_, 0.07);
    nhp_.param("ompl_mode", ompl_mode_, RRT_START_MODE);
    nhp_.param("planning_mode", planning_mode_, ONLY_JOINTS_MODE);
    nhp_.param("collision_detection_rate", collision_detection_rate_, 10.0);

    start_state_.resize(6);
    nhp_.param("start_state_x", start_state_[0], 0.0);
    nhp_.param("start_state_y", start_state_[1], 0.5);
    nhp_.param("start_state_theta", start_state_[2], 0.785);
    nhp_.param("start_state_joint1", start_state_[3], -1.57);
    nhp_.param("start_state_joint2", start_state_[4], -1.57);
    nhp_.param("start_state_joint3", start_state_[5], -1.57);
    goal_state_.resize(6);
    nhp_.param("goal_state_x", goal_state_[0], 3.0);
    nhp_.param("goal_state_y", goal_state_[1], 0.5);
    nhp_.param("goal_state_theta", goal_state_[2], 0.785);
    nhp_.param("goal_state_joint1", goal_state_[3], -1.57);
    nhp_.param("goal_state_joint2", goal_state_[4], -1.57);
    nhp_.param("goal_state_joint3", goal_state_[5], -1.57);


    nhp_.param("real_robot_move_base", real_robot_move_base_, false);
    nhp_.param("mocap_center_to_link_center_x", mocap_center_to_link_center_x_, 0.0);
    nhp_.param("mocap_center_to_link_center_y", mocap_center_to_link_center_y_, 0.0);
    if(real_robot_move_base_)
      {
        get_init_state_ = false;
        pose_sub_ = nh_.subscribe<jsk_quadcopter::SlamDebug>("ground_truth/pose", 1, &PlanningScene::poseCallback, this, ros::TransportHints().tcpNoDelay());
        while(1)
          {
            //for start state: mocap center
            if(get_init_state_) break;
          }
        //for goal state
        //TODO goal state
      }
    
    bool no_callback_flag = false;
    transform_controller_ = new TransformController(nh_, nhp_, no_callback_flag);

    while(planning_scene_diff_pub_.getNumSubscribers() < 1)
      {
        ros::WallDuration sleep_t(0.5);
        sleep_t.sleep();
      }
    ros::Duration sleep_time(1.0);
    sleep_time.sleep();
    
    robot_model_loader_ = new robot_model_loader::RobotModelLoader("robot_description");
    kinematic_model_ = robot_model_loader_->getModel();
    planning_scene_ = new planning_scene::PlanningScene(kinematic_model_);
    tolerance_ = 0.01;
    acm_ = planning_scene_->getAllowedCollisionMatrix();

    calculation_time_ = 0;


    //pub the collision object => gap env
    left_half_corner = tf::Vector3(gap_left_x_, gap_left_y_ , gap_left_width_);
    right_half_corner = tf::Vector3(gap_left_x_ + gap_y_offset_, gap_left_y_ + gap_x_offset_ , gap_right_width_);

    collision_object_.header.frame_id = "world";
    collision_object_.id = "box";
    geometry_msgs::Pose pose1, pose2,pose3,pose4;
    pose1.position.x = left_half_corner.getX() + gap_left_width_ /2;
    pose1.position.y =  (2.5 + gap_x_offset_/2) /2;
    pose1.position.z = 0.0;
    pose1.orientation.w = 1.0;
    pose2.position.x = right_half_corner.getX() + gap_right_width_ /2;
    pose2.position.y = - (2.5 + gap_x_offset_/2) /2;
    pose2.position.z = 0.0;
    pose2.orientation.w = 1.0;

    pose3.position.x = 1.0;
    pose3.position.y = 2.5;
    pose3.position.z = 0.0;
    pose3.orientation.w = 1.0;
    pose4.position.x = 1.0;
    pose4.position.y = -2.5;
    pose4.position.z = 0.0;
    pose4.orientation.w = 1.0;


    shape_msgs::SolidPrimitive primitive1, primitive2, primitive3, primitive4;
    primitive1.type = primitive1.BOX;
    primitive1.dimensions.resize(3);

    primitive1.dimensions[0] = gap_left_width_;
    primitive1.dimensions[1] = 2.5 - gap_x_offset_ / 2;
    primitive1.dimensions[2] = 1;
    collision_object_.primitives.push_back(primitive1);
    collision_object_.primitive_poses.push_back(pose1);
    primitive2.type = primitive2.BOX;
    primitive2.dimensions.resize(3);
    primitive2.dimensions[0] = gap_right_width_;
    primitive2.dimensions[1] = 2.5 - gap_x_offset_ / 2;
    primitive2.dimensions[2] = 1;
    collision_object_.primitives.push_back(primitive2);
    collision_object_.primitive_poses.push_back(pose2);
    primitive3.type = primitive2.BOX;
    primitive3.dimensions.resize(3);
    primitive3.dimensions[0] = 8;
    primitive3.dimensions[1] = 0.6;
    primitive3.dimensions[2] = 1;
    collision_object_.primitives.push_back(primitive3);
    collision_object_.primitive_poses.push_back(pose3);
    primitive4.type = primitive2.BOX;
    primitive4.dimensions.resize(3);
    primitive4.dimensions[0] = 8;
    primitive4.dimensions[1] = 0.6;
    primitive4.dimensions[2] = 1;

    collision_object_.primitives.push_back(primitive4);
    collision_object_.primitive_poses.push_back(pose4);

    collision_object_.operation = collision_object_.ADD;
    planning_scene_->processCollisionObjectMsg(collision_object_);

    //temporarily
    robot_state::RobotState& init_state = planning_scene_->getCurrentStateNonConst();
    init_state.setStateValues(start_state_);
    /**/

    planning_scene_->getPlanningSceneMsg(planning_scene_msg_);
    planning_scene_msg_.is_diff = true;
    planning_scene_diff_pub_.publish(planning_scene_msg_);

#if 1
    //planning
    ompl::base::StateSpacePtr se2(new ompl::base::SE2StateSpace());
    ompl::base::RealVectorBounds motion_bounds(2);
    motion_bounds.low[0] = -2;
    motion_bounds.low[1] = -2.2;
    motion_bounds.high[0] = 5;
    motion_bounds.high[1] = 2.2;
    se2->as<ompl::base::SE2StateSpace>()->setBounds(motion_bounds);

    ompl::base::StateSpacePtr r3(new ompl::base::RealVectorStateSpace(3));
    ompl::base::RealVectorBounds joint_bounds(3);
    joint_bounds.setLow(-1.6);
    joint_bounds.setHigh(1.6);
    r3->as<ompl::base::RealVectorStateSpace>()->setBounds(joint_bounds);


    if(planning_mode_ == ONLY_JOINTS_MODE)
      {
        hydra_space_ = r3;
        hydra_space_->as<ompl::base::RealVectorStateSpace>()->setValidSegmentCountFactor(20);
        space_information_  = boost::shared_ptr<ompl::base::SpaceInformation> (new ompl::base::SpaceInformation(hydra_space_));
        space_information_->setup();
        space_information_->setStateValidityChecker(boost::bind(&PlanningScene::isStateValid, this, _1));
        space_information_->setStateValidityCheckingResolution(0.03); // 3%
        space_information_->setMotionValidator(ompl::base::MotionValidatorPtr(new ompl::base::DiscreteMotionValidator(space_information_)));
        space_information_->setup();

      }
    else if(planning_mode_ == ONLY_BASE_MODE || planning_mode_ == ORIGINAL_MODE)
      {
        hydra_space_ = se2;
        hydra_space_->as<ompl::base::SE2StateSpace>()->setValidSegmentCountFactor(200);
        space_information_  = boost::shared_ptr<ompl::base::SpaceInformation> (new ompl::base::SpaceInformation(hydra_space_));
        space_information_->setup();
        space_information_->setStateValidityChecker(boost::bind(&PlanningScene::isStateValid, this, _1));
        space_information_->setStateValidityCheckingResolution(0.03); // 3%
        space_information_->setMotionValidator(ompl::base::MotionValidatorPtr(new ompl::base::DiscreteMotionValidator(space_information_)));
        space_information_->setup();
      }
    else if(planning_mode_ == JOINTS_AND_BASE_MODE)
      {
        hydra_space_ = se2 + r3;
        space_information_  = boost::shared_ptr<ompl::base::SpaceInformation> (new ompl::base::SpaceInformation(hydra_space_));
        space_information_->setStateValidityChecker(boost::bind(&PlanningScene::isStateValid, this, _1));
      }


    //init state
    robot_state::RobotState& current_state = planning_scene_->getCurrentStateNonConst();
    std::vector<double> joint_values;
    current_state.getStateValues(joint_values);
    

    ompl::base::ScopedState<> start(hydra_space_);
    ompl::base::ScopedState<> goal(hydra_space_);
    if(planning_mode_ == ONLY_JOINTS_MODE)
      {
        joint_values[0] = 0;
        joint_values[1] = 0.5;
        joint_values[2] = 0.785;
        joint_values[3] = 1.57;
        joint_values[4] = 1.57;
        joint_values[5] = 1.57;

        start->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = joint_values[3];
        start->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] = joint_values[4];
        start->as<ompl::base::RealVectorStateSpace::StateType>()->values[2] = joint_values[5];
        goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = -1.57;
        goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] = -1.57;
        goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[2] = -1.57;

      }
    else if(planning_mode_ == ONLY_BASE_MODE)
      {
        joint_values[0] = 0;
        joint_values[1] = 0.5;
        joint_values[2] = 0.785;
        joint_values[3] = 0;
        joint_values[4] = 1.57;
        joint_values[5] = 0;

        start->as<ompl::base::SE2StateSpace::StateType>()->setXY(joint_values[0], joint_values[1]);
        start->as<ompl::base::SE2StateSpace::StateType>()->setYaw(joint_values[2]);
        goal->as<ompl::base::SE2StateSpace::StateType>()->setXY(3, 0.5);
        goal->as<ompl::base::SE2StateSpace::StateType>()->setYaw(joint_values[2]);
      }

    else if(planning_mode_ == JOINTS_AND_BASE_MODE)
      {

        joint_values[0] = 0;
        joint_values[1] = 0.5;
        joint_values[2] = 0.785;
        joint_values[3] = 1.57;
        joint_values[4] = 1.57;
        joint_values[5] = 1.57;

        ompl::base::CompoundState* start_tmp = dynamic_cast<ompl::base::CompoundState*> (start.get());

        start_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->setXY(joint_values[0], joint_values[1]);
        start_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->setYaw(joint_values[2]);
        start_tmp->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[0] = joint_values[3];
        start_tmp->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[1] = joint_values[4];
        start_tmp->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[2] = joint_values[5];

        ompl::base::CompoundState* goal_tmp = dynamic_cast<ompl::base::CompoundState*> (goal.get());
        goal_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->setXY(3, 0.05);
        goal_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->setYaw(joint_values[2]);
        goal_tmp->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[0] = 1.57;
        goal_tmp->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[1] = 1.57;
        goal_tmp->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[2] = 1.57;


      }
    else if(planning_mode_ == ORIGINAL_MODE)
      {

        solved_ = gapDetection(left_half_corner, right_half_corner, transform_controller_, start_state_, goal_state_, original_path_, planning_scene_, collision_object_);
      }


    current_state.setStateValues(joint_values);

    if(planning_mode_ < ORIGINAL_MODE)
      {

        ompl::base::ProblemDefinitionPtr pdef(new ompl::base::ProblemDefinition(space_information_));
        pdef->setStartAndGoalStates(start, goal);

        ompl::base::PlannerPtr planner;
        if(ompl_mode_ == RRT_START_MODE)
          {
            //original for optimization
            pdef->setOptimizationObjective(getPathLengthObjective(space_information_));
            planner = ompl::base::PlannerPtr(new ompl::geometric::RRTstar(space_information_));
          }
        else if(ompl_mode_ == LBKPIECE1_MODE)
          {
            planner = ompl::base::PlannerPtr(new ompl::geometric::LBKPIECE1(space_information_));
          }
        else
          {
            planner = ompl::base::PlannerPtr(new ompl::geometric::SBL(space_information_));
          }
        planner->setProblemDefinition(pdef);
        planner->setup();
        space_information_->printSettings(std::cout);


        solved_ = false;
        ros::Time start_time = ros::Time::now();
        ompl::base::PlannerStatus solved = planner->solve(3600.0);
        if (solved)
          {
            calculation_time_ = ros::Time::now().toSec() - start_time.toSec();
            solved_ = true;
            path_ = pdef->getSolutionPath();
            std::cout << "Found solution:" << std::endl;
            path_->print(std::cout);

            //visualztion
            plan_states_ = new ompl::base::StateStorage(hydra_space_);
            int index = (int)(boost::static_pointer_cast<ompl::geometric::PathGeometric>(path_)->getStateCount());
            for(int i = 0; i < index; i++)
              {
                ompl::base::State *state1;
                ompl::base::State *state2;
            
                if(i == 0)
                  state1 = start.get();
                else 
                  state1 = boost::static_pointer_cast<ompl::geometric::PathGeometric>(path_)->getState(i - 1);
        
                state2 = boost::static_pointer_cast<ompl::geometric::PathGeometric>(path_)->getState(i);

                plan_states_->addState(state1);
                int nd = hydra_space_->validSegmentCount(state1, state2);
                if (nd > 1)
                  {
                    ompl::base::State *interpolated_state = space_information_->allocState();
                    for (int j = 1 ; j < nd ; ++j)
                      {
                        hydra_space_->interpolate(state1, state2, (double)j / (double)nd, interpolated_state);
                        plan_states_->addState(interpolated_state);
                      }
                  }
                plan_states_->addState(state2);
              }
          }
        else
          std::cout << "No solution found" << std::endl;


      }
    collision_detection_timer_ = nhp_.createTimer(ros::Duration(1.0 / collision_detection_rate_), &PlanningScene::collisionFunc, this);

#endif
  }

  tf::Vector3 world2GapCoord(tf::Vector3 world_info)
  {
    return tf::Vector3(-world_info.getY(), world_info.getX(), world_info.getZ());
  }
  tf::Vector3 gap2WorldCoord(tf::Vector3 gap_coord_info)
  {
    return tf::Vector3(gap_coord_info.getY(),-gap_coord_info.getX(), gap_coord_info.getZ());

  }


  bool gapDetection(tf::Vector3 left_bound, tf::Vector3 right_bound,  TransformController* transform_controller, std::vector<double> start_state, std::vector<double> goal_state, std::vector<conf_values>& original_path, planning_scene::PlanningScene* planning_scene,   moveit_msgs::CollisionObject collision_object)
  {
    original_path.resize(0);
    real_robot_path_.resize(0);

    tf::Vector3 left_corner = world2GapCoord(left_bound);
    tf::Vector3 right_corner = world2GapCoord(right_bound);

    //float l = transform_controller->getLinkLength() - 0.025; //temporary
    //float d = transform_controller->getPropellerDiameter() + 0.1;
    //temporary
    float l = 0.55; //link1
    float d = 0.3; //

    //overlap type 
    int overlap_type = NO_OVERLAP;
    if(left_corner.getX() >= right_corner.getX())
      {
        overlap_type = X_AXIS_OVERLAP;
        ROS_INFO("x axis overlap");
      }
    else if((left_corner.getY() + left_corner.getZ() < right_corner.getY() )||
            right_corner.getY() + right_corner.getZ() < left_corner.getY())
      {
        ROS_INFO("no overlap");
      }
    else
      {
        overlap_type = Y_AXIS_OVERLAP;
        ROS_INFO("y axis overlap");
      }

    //length of gap
    float length_of_gap = 0;
    float gap_inclination = 0;
    if(overlap_type == NO_OVERLAP)
      {
        if(left_corner.getY() + left_corner.getZ() < right_corner.getY())
          {
            length_of_gap = distance(left_corner.getX(), right_corner.getX(),
                                     left_corner.getY() + left_corner.getZ(), 
                                     right_corner.getY());
            gap_inclination = atan2(right_corner.getY() - (left_corner.getY() + left_corner.getZ()), right_corner.getX() - left_corner.getX());
            ROS_INFO("left half lower, length:%f, inclination:%f",length_of_gap, gap_inclination);
          }
        else
          {
            length_of_gap = distance(left_corner.getX(), right_corner.getX(),
                                   right_corner.getY() + right_corner.getZ(), 
                                   left_corner.getY());
            gap_inclination = atan2((right_corner.getY() + right_corner.getZ()) -left_corner.getY() , right_corner.getX() - left_corner.getX());
            ROS_INFO("right half lower, length:%f, inclination:%f",length_of_gap, gap_inclination);
          }
      }
    else if(overlap_type == X_AXIS_OVERLAP)
      {
        if(left_corner.getY() + left_corner.getZ() < right_corner.getY())
          {
            length_of_gap = right_corner.getY() - left_corner.getY() - left_corner.getZ();
            ROS_INFO("left half lower, length:%f",length_of_gap);
          }
        else
          { 
            length_of_gap =  left_corner.getY() - right_corner.getZ() - right_corner.getY();
            ROS_INFO("right half lower, length:%f",length_of_gap);
          }
      }
    else if(overlap_type == Y_AXIS_OVERLAP)
      {
        length_of_gap = right_corner.getX() - left_corner.getX();
        ROS_INFO("length:%f",length_of_gap);
      }

    if(length_of_gap > l + d)
      {
        ROS_WARN("traverse without transform");
        //TODO: ompl temporary
        planning_mode_ = ONLY_BASE_MODE + ORIGINAL_MODE;

        ompl::base::StateSpacePtr se2(new ompl::base::SE2StateSpace());
        ompl::base::RealVectorBounds motion_bounds(2);
        motion_bounds.low[0] = start_state[0] - 0.5; //margin
        motion_bounds.low[1] = (start_state[1] < goal_state[1])?(start_state[1] - 0.5):(goal_state[1] - 0.5);
        motion_bounds.high[0] = goal_state[0] + 0.5;;
        motion_bounds.high[1] = (start_state[1] < goal_state[1])?(goal_state[1] + 0.5):(start_state[1] + 0.5);
        se2->as<ompl::base::SE2StateSpace>()->setBounds(motion_bounds);

        hydra_space_ = se2;
        hydra_space_->as<ompl::base::SE2StateSpace>()->setValidSegmentCountFactor(200);
        space_information_  = boost::shared_ptr<ompl::base::SpaceInformation> (new ompl::base::SpaceInformation(hydra_space_));
        space_information_->setup();
        space_information_->setStateValidityChecker(boost::bind(&PlanningScene::isStateValid, this, _1));
        space_information_->setStateValidityCheckingResolution(0.03); // 3%
        space_information_->setMotionValidator(ompl::base::MotionValidatorPtr(new ompl::base::DiscreteMotionValidator(space_information_)));
        space_information_->setup();

        ompl::base::ScopedState<> start(hydra_space_);
        ompl::base::ScopedState<> goal(hydra_space_);

        start->as<ompl::base::SE2StateSpace::StateType>()->setXY(start_state[0], start_state[1]);
        start->as<ompl::base::SE2StateSpace::StateType>()->setYaw(start_state[2]);
        goal->as<ompl::base::SE2StateSpace::StateType>()->setXY(goal_state[0], goal_state[1]);
        goal->as<ompl::base::SE2StateSpace::StateType>()->setYaw(goal_state[2]);



        ompl::base::ProblemDefinitionPtr pdef(new ompl::base::ProblemDefinition(space_information_));
        pdef->setStartAndGoalStates(start, goal);

        ompl::base::PlannerPtr planner;
        if(ompl_mode_ == RRT_START_MODE)
          {
            pdef->setOptimizationObjective(getPathLengthObjective(space_information_));
            planner = ompl::base::PlannerPtr(new ompl::geometric::RRTstar(space_information_));
          }
        else if(ompl_mode_ == LBKPIECE1_MODE)
          {
            planner = ompl::base::PlannerPtr(new ompl::geometric::LBKPIECE1(space_information_));
          }
        else
          {
            planner = ompl::base::PlannerPtr(new ompl::geometric::SBL(space_information_));
          }
        planner->setProblemDefinition(pdef);
        planner->setup();
        space_information_->printSettings(std::cout);


        solved_ = false;
        ompl::base::PlannerStatus solved = planner->solve(3600.0);
        if (solved)
          {
            solved_ = true;
            path_ = pdef->getSolutionPath();
            std::cout << "Found solution:" << std::endl;
            path_->print(std::cout);

            //visualztion
            plan_states_ = new ompl::base::StateStorage(hydra_space_);
            int index = (int)(boost::static_pointer_cast<ompl::geometric::PathGeometric>(path_)->getStateCount());
            for(int i = 0; i < index; i++)
              {
                ompl::base::State *state1;
                ompl::base::State *state2;
            
                if(i == 0)
                  state1 = start.get();
                else 
                  state1 = boost::static_pointer_cast<ompl::geometric::PathGeometric>(path_)->getState(i - 1);
        
                state2 = boost::static_pointer_cast<ompl::geometric::PathGeometric>(path_)->getState(i);

                plan_states_->addState(state1);
                int nd = hydra_space_->validSegmentCount(state1, state2);
                if (nd > 1)
                  {
                    ompl::base::State *interpolated_state = space_information_->allocState();
                    for (int j = 1 ; j < nd ; ++j)
                      {
                        hydra_space_->interpolate(state1, state2, (double)j / (double)nd, interpolated_state);
                        plan_states_->addState(interpolated_state);
                      }
                  }
                plan_states_->addState(state2);
              }
          }
        else
          std::cout << "No solution found" << std::endl;

        return true;
      }
    else if(length_of_gap < d)
      {
        ROS_WARN("can not traverse");
        return false;
      }

    //centroid
    int l_r_flag = 0;
    tf::Vector3 shorter_corner;
    float test_corner_x;
    float test_corner_y_down;
    float test_corner_y_top;
    if(left_corner.getZ() <= right_corner.getZ()) 
      {
        shorter_corner = left_corner;
        test_corner_x = right_corner.getX() - left_corner.getX();
        test_corner_y_down = right_corner.getY() - (left_corner.getY() + left_corner.getZ()/2);
        test_corner_y_top = right_corner.getY() - (left_corner.getY() + left_corner.getZ()/2) + right_corner.getZ();

        l_r_flag = 0; //left
        ROS_INFO("left shorter, test_corner_x:%f, test_corner_y_down:%f, test_corner_y_top:%f",test_corner_x, test_corner_y_down, test_corner_y_top);
      }
    else 
      {
        shorter_corner = right_corner;
        test_corner_x = left_corner.getX() - right_corner.getX();
        test_corner_y_down = left_corner.getY() - (right_corner.getY() + right_corner.getZ()/2);
        test_corner_y_top = left_corner.getY() - (right_corner.getY() + right_corner.getZ()/2) + left_corner.getZ();
        l_r_flag = 1; //right
        ROS_INFO("right shorter, test_corner_x:%f, test_corner_y_down:%f, test_corner_y_top:%f",test_corner_x, test_corner_y_down, test_corner_y_top);

      }

    float h = shorter_corner.getZ();
    bool collision_flag = false;
    int centroid_type = 0; //0: ellipse, -1: under_parabola, 1: top_parabola

    float a = h / 2 + (sqrt(2)  -1 ) * d;
    float b = (h + 2 * d) / 2;
    

    if(l_r_flag == 0)//left
      {
        //*** visualizatio of the centroid
        //ellipse
        for(float theta = -M_PI/2; theta < M_PI/2; theta += M_PI /100)
          {
            float ellipse_x = cos(theta) * a + d + left_corner.getX();
            float ellipse_y = sin(theta) * b + left_corner.getY() + left_corner.getZ()/2;
            gap2WorldCoord(tf::Vector3(ellipse_x,ellipse_y,0)).getY();
            geometry_msgs::Pose pose;
            pose.position.x = gap2WorldCoord(tf::Vector3(ellipse_x,ellipse_y,0)).getX();
            pose.position.y = gap2WorldCoord(tf::Vector3(ellipse_x,ellipse_y,0)).getY();
            pose.position.z = 0.0;
            pose.orientation.w = 1.0;
            shape_msgs::SolidPrimitive primitive;
            primitive.type = primitive.BOX;
            primitive.dimensions.resize(3);
            primitive.dimensions[0] = 0.01;
            primitive.dimensions[1] = 0.01;
            primitive.dimensions[2] = 1;
            collision_object.primitives.push_back(primitive);
            collision_object.primitive_poses.push_back(pose);
          }
        //parabola
        for(float theta =  0; theta < M_PI/4; theta += M_PI /100)
          {
            float parabola_y_top = (2*l - d/2) * theta + d *( theta + 1) + h / 2;
            float parabola_x =  - (2*l - d/2) * theta * theta + d * (1 -theta) + left_corner.getX();
            float parabola_y_down = -parabola_y_top + left_corner.getY() + left_corner.getZ()/2;
            parabola_y_top = parabola_y_top + left_corner.getY() + left_corner.getZ()/2;

            geometry_msgs::Pose pose1, pose2;
            pose1.position.x = gap2WorldCoord(tf::Vector3(parabola_x,parabola_y_top,0)).getX();
            pose1.position.y = gap2WorldCoord(tf::Vector3(parabola_x,parabola_y_top,0)).getY();
            pose1.position.z = 0.0;
            pose1.orientation.w = 1.0;
            pose2.position.x = gap2WorldCoord(tf::Vector3(parabola_x,parabola_y_down,0)).getX();
            pose2.position.y = gap2WorldCoord(tf::Vector3(parabola_x,parabola_y_down,0)).getY();
            pose2.position.z = 0.0;
            pose2.orientation.w = 1.0;

            shape_msgs::SolidPrimitive primitive;
            primitive.type = primitive.BOX;
            primitive.dimensions.resize(3);
            primitive.dimensions[0] = 0.01;
            primitive.dimensions[1] = 0.01;
            primitive.dimensions[2] = 1;
            collision_object.primitives.push_back(primitive);
            collision_object.primitives.push_back(primitive);
            collision_object.primitive_poses.push_back(pose1);
            collision_object.primitive_poses.push_back(pose2);

            geometry_msgs::Pose pose3, pose4;
            parabola_y_top = (2*l - d/2) * sin(theta) + sqrt(2) * d * sin(M_PI/4 + theta) + h / 2;
            parabola_x =  - (2*l - d/2) * sin(theta) * tan(theta) + sqrt(2) * d * cos(M_PI/4 + theta) + left_corner.getX();
            parabola_y_down = -parabola_y_top + left_corner.getY() + left_corner.getZ()/2;
            parabola_y_top = parabola_y_top + left_corner.getY() + left_corner.getZ()/2;

            pose3.position.x = gap2WorldCoord(tf::Vector3(parabola_x,parabola_y_top,0)).getX();
            pose3.position.y = gap2WorldCoord(tf::Vector3(parabola_x,parabola_y_top,0)).getY();
            pose3.position.z = 0.0;
            pose3.orientation.w = 1.0;
            pose4.position.x = gap2WorldCoord(tf::Vector3(parabola_x,parabola_y_down,0)).getX();
            pose4.position.y = gap2WorldCoord(tf::Vector3(parabola_x,parabola_y_down,0)).getY();
            pose4.position.z = 0.0;
            pose4.orientation.w = 1.0;

            primitive.type = primitive.BOX;
            primitive.dimensions.resize(3);
            primitive.dimensions[0] = 0.01;
            primitive.dimensions[1] = 0.01;
            primitive.dimensions[2] = 1;
            collision_object.primitives.push_back(primitive);
            collision_object.primitives.push_back(primitive);
            collision_object.primitive_poses.push_back(pose3);
            collision_object.primitive_poses.push_back(pose4);

          }


#if 0
        collision_object.operation = collision_object.ADD;
        planning_scene->processCollisionObjectMsg(collision_object);
        moveit_msgs::PlanningScene planning_scene_msg;
        planning_scene->getPlanningSceneMsg(planning_scene_msg);
        planning_scene_msg.is_diff = true;
        planning_scene_diff_pub_.publish(planning_scene_msg);
#endif

        if(test_corner_x < h/2 + sqrt(2) * d)
          {//else is ellipse
            ROS_INFO("left fit some centroid curve");
            if(test_corner_x >= d)
              {//ellipse
                ROS_INFO("left fit ellipse centroid curve");

                // y^2 = b^2 - b^2 / a^2 * (x -d )^2

                float ellipse_y1 = sqrt(b*b*(1- (test_corner_x - d) * (test_corner_x - d) / (a * a)));
                float ellipse_y2 = -ellipse_y1;

                if(! ((test_corner_y_top > ellipse_y1 && test_corner_y_down > ellipse_y1) || (test_corner_y_top < ellipse_y2 && test_corner_y_down < ellipse_y2)))
                  {
                    ROS_WARN("traverse with two step of transform");
                    collision_flag = true;
                  }
                ROS_INFO("ellipse path, ellipse_y1:%f, a:%f, b:%f",ellipse_y1, a, b);
                centroid_type = 0; //ellipse
              }
            else
              { // parabola
                float parabola_y1 = d + (2*l - d/2 + d) * (sqrt(d*d/(4*(2*l - d/2)*(2*l - d/2)) + (d-test_corner_x)/(2*l - d/2)) - d/2/(2*l - d/2)) + h/2;
                float parabola_y2 = -parabola_y1;

                ROS_INFO("parabola path, parabola_y1:%f",parabola_y1);
                if(test_corner_y_top > parabola_y1 && test_corner_y_down > parabola_y1)
                  {//top parabola
                    centroid_type = 1; //top parabola
                    ROS_INFO("top parabola");
                  }
                else if(test_corner_y_top < parabola_y2 && test_corner_y_down < parabola_y2)
                  {
                    //under parabola
                    ROS_INFO("under parabola");
                    centroid_type = -1; //under parabola
                  }
                else
                  {
                    ROS_WARN("traverse with two step of transform");
                    collision_flag = true;
                  }
              }
          }
        else
          {
            ROS_INFO("left,x out of all centroid, no collision, fit ellipse path");
          }
      }
    else//right
      {
        if(test_corner_x  > -(h/2 + sqrt(2) * d))
          {//else is ellipse
            ROS_INFO("right fit some centroid curve");
            if(test_corner_x <= -d)
              {//ellipse
                ROS_INFO("right fit ellipse centroid curve");
                //float a = h / 2 + (sqrt(2)  -1 ) * d;
                //float b = (h + 2 * d) / 2;
                // y^2 = b^2 - b^2 / a^2 * (x -d )^2
                float ellipse_y1 = sqrt(b*b*(1- (test_corner_x + d) * (test_corner_x + d) / (a * a)));
                float ellipse_y2 = -ellipse_y1;

                if(! ((test_corner_y_top > ellipse_y1 && test_corner_y_down > ellipse_y1) && (test_corner_y_top < ellipse_y2 && test_corner_y_down < ellipse_y2)))
                  {
                    ROS_WARN("traverse with two step of transform");
                    collision_flag = true;
                  }
                ROS_INFO("ellipse path, ellipse_y1:%f, a:%f, b:%f",ellipse_y1, a, b);
                centroid_type = 0; //ellipse
              }
            else
              { // parabola
                float parabola_y1 = d + (2*l - d/2 + d) * (sqrt(d*d/(4*(2*l - d/2)*(2*l - d/2)) + (d+test_corner_x)/(2*l - d/2)) - d/2/(2*l - d/2)) + h/2;
                float parabola_y2 = -parabola_y1;
                ROS_INFO("parabola path, parabola_y1:%f",parabola_y1);
                if(test_corner_y_top > parabola_y1 && test_corner_y_down > parabola_y1)
                  {//top parabola
                    centroid_type = 1; //top parabola
                    ROS_INFO("top parabola");
                  }
                else if(test_corner_y_top < parabola_y2 && test_corner_y_down < parabola_y2)
                  {
                    //under parabola
                    ROS_INFO("under parabola");
                    centroid_type = -1; //under parabola
                  }
                else
                  {
                    ROS_WARN("traverse with two step of transform");
                    collision_flag = true;
                  }
              }
          }
        else
          {
            ROS_INFO("right,x out of all centroid, no collision, fit ellipse path");
          }
      }

    if(collision_flag)
      {//two step of transform
        float middle_point_x, middle_point_y;
        float d_up, d_down;
        float x1,x2,x3,x4;
        float y1,y2,y3,y4;
        int transform_type = 0; 
        float direction = 0; //top shape(sgn), down shape(odd/even)

        ROS_INFO("two step traverse");

        if(overlap_type == NO_OVERLAP)
          {//NO Overlap

            middle_point_x = (left_corner.getX() + right_corner.getX())/2;
             middle_point_y = (gap_inclination > 0)?(left_corner.getY() + right_corner.getY() + left_corner.getZ())/2: (left_corner.getY() + right_corner.getY() + right_corner.getZ())/2;

             d_up = (gap_inclination > 0)?right_corner.getZ() * cos(gap_inclination):left_corner.getZ() * cos(gap_inclination);

             d_down = (gap_inclination > 0)?left_corner.getZ() * cos(gap_inclination):right_corner.getZ() * cos(gap_inclination);

            if((gap_inclination > 0 && gap_inclination <= M_PI/4)||
               (gap_inclination > -M_PI && gap_inclination <= -M_PI/4))
              transform_type = -2; //"2" model
            else
              transform_type = 1; // "s" model


             x1 = middle_point_x + d_down * sin(gap_inclination);
             y1 = middle_point_y - d_down * cos(gap_inclination);
             x2 = middle_point_x - 2 * l * sin(gap_inclination);
             y2 = middle_point_y + 2 * l * cos(gap_inclination);
             x3 = middle_point_x - l * sin(gap_inclination) + l * abs(transform_type)/transform_type * cos(gap_inclination);
             y3 = middle_point_y + l * cos(gap_inclination) + l * abs(transform_type)/transform_type * sin(gap_inclination) ;
             x4 = middle_point_x - (3*l + d_up) * sin(gap_inclination) + l * abs(transform_type)/transform_type * cos(gap_inclination);
             y4 = middle_point_y + (3*l + d_up) * cos(gap_inclination) + l * abs(transform_type)/transform_type * sin(gap_inclination) ;

            direction = gap_inclination;
          }
        else if (overlap_type == X_AXIS_OVERLAP)
          {
            if(left_corner.getX() - right_corner.getX() >= (2*l-d))
              {
                ROS_WARN("can not traverse");
                return false;
              }
            else
              {
                if(left_corner.getY() + left_corner.getZ() < right_corner.getY())
                  {//left down
                    middle_point_x = (left_corner.getX() + right_corner.getX())/2;
                    middle_point_y = (left_corner.getY() + left_corner.getZ() + right_corner.getY())/2;
                    transform_type = 1; // "s" model
                    x1 = left_corner.getX() + 0.1; //+ 0.1mm margin
                    y1 = middle_point_y;
                    x2 = middle_point_x - 2*l;
                    y2 = middle_point_y;
                    x3 = middle_point_x - l;
                    y3 = middle_point_y + l;
                    x4 = right_corner.getX() - 3*l;
                    y4 = middle_point_y + l;
                    direction = M_PI/2;
                  }
                else
                  {//right down
                    middle_point_x = (left_corner.getX() + right_corner.getX())/2;
                    middle_point_y = (left_corner.getY() + right_corner.getZ() + right_corner.getY())/2;
                    transform_type = -2; // "2" model
                    x1 = right_corner.getX() - 0.1; //+ 0.1mm margin
                    y1 = middle_point_y;
                    x2 = middle_point_x + 2*l;
                    y2 = middle_point_y;
                    x3 = middle_point_x + l;
                    y3 = middle_point_y + l;
                    x4 = left_corner.getX() + 3*l;
                    y4 = middle_point_y + l;
                    direction = -M_PI/2;
                  }
              }
          }
        else if (overlap_type == Y_AXIS_OVERLAP)
          {
            direction = 0;
            if(left_corner.getY() + left_corner.getZ() < right_corner.getY() + right_corner.getZ())
              {
                if(left_corner.getY() < right_corner.getY())
                  {//l_up + r_down
                    ROS_INFO("l up and r down");
                    if(left_corner.getY() + left_corner.getZ() - right_corner.getY() >= (2*l-d))
                      {
                        ROS_WARN("can not traverse");
                        return false;
                      }
                    middle_point_x = (left_corner.getX() + right_corner.getX())/2;
                    middle_point_y = (left_corner.getY() + left_corner.getZ() + right_corner.getY())/2;
                    transform_type = -2; // "2" model
                    x1 = middle_point_x;
                    y1 = left_corner.getY();
                    x2 = middle_point_x;
                    y2 = middle_point_y + 2*l;
                    x3 = middle_point_x - l;
                    y3 = middle_point_y + l;
                    x4 = middle_point_x - l;
                    y4 = right_corner.getY() + right_corner.getZ() + 3*l + 0.1; //0.2m padding

                  }
                else
                  {//l_up + l_down
                    ROS_INFO("l up and l down");
                    if(left_corner.getZ() >= (2*l-d))
                      {
                        ROS_WARN("can not traverse");
                        return false;
                      }
                    middle_point_x = (left_corner.getX() + right_corner.getX())/2;
                    middle_point_y = (left_corner.getY() + left_corner.getZ() + left_corner.getY())/2;
                    transform_type = -1; // "inv c" model
                    x1 = middle_point_x;
                    y1 = left_corner.getY();
                    x2 = middle_point_x;
                    y2 = middle_point_y + 2*l;
                    x3 = middle_point_x - l;
                    y3 = middle_point_y + l;
                    x4 = middle_point_x - l;
                    y4 = right_corner.getY() + right_corner.getZ() + 3*l + 0.1; //0.1m padding
                  }
              }
            else
              {
                if(left_corner.getY() > right_corner.getY())
                  {//l_down + r_up
                    ROS_INFO("l down and r up");
                    if(right_corner.getY() + right_corner.getZ() - left_corner.getY() >= (2*l-d))
                      {
                        ROS_WARN("can not traverse");
                        return false;
                      }
                    middle_point_x = (left_corner.getX() + right_corner.getX())/2;
                    middle_point_y = (left_corner.getY() + right_corner.getZ() + right_corner.getY())/2;

                    transform_type = 1; // "s" model
                    x1 = middle_point_x;
                    y1 = right_corner.getY();
                    x2 = middle_point_x;
                    y2 = middle_point_y + 2*l;
                    x3 = middle_point_x + l;
                    y3 = middle_point_y + l;
                    x4 = middle_point_x + l;
                    y4 = left_corner.getY() + left_corner.getZ() + 3*l + 0.1; //0.2m padding
                  }
                else
                  {//r_up + r_down
                    ROS_INFO("r down and r up");
                    if(right_corner.getZ() >= (2*l-d))
                      {
                        ROS_WARN("can not traverse");
                        return false;
                      }
                    middle_point_x = (left_corner.getX() + right_corner.getX())/2;
                    middle_point_y = (right_corner.getY() + right_corner.getZ() + right_corner.getY())/2;

                    transform_type = 2; // "c" model
                    x1 = middle_point_x;
                    y1 = right_corner.getY();
                    x2 = middle_point_x;
                    y2 = middle_point_y + 2*l;
                    x3 = middle_point_x + l;
                    y3 = middle_point_y + l;
                    x4 = middle_point_x + l;
                    y4 = left_corner.getY() + left_corner.getZ() + 3*l + 0.1; //0.2m padding
                  }
              }
          }

        float delta_x, delta_y;
        float joint1_delta, joint2_delta, joint3_delta, theta_delta;

        tf::Vector3 xy1_world_coord = gap2WorldCoord(tf::Vector3(x1, y1, 0));
        tf::Vector3 xy2_world_coord = gap2WorldCoord(tf::Vector3(x2, y2, 0));
        tf::Vector3 xy3_world_coord = gap2WorldCoord(tf::Vector3(x3, y3, 0));
        tf::Vector3 xy4_world_coord = gap2WorldCoord(tf::Vector3(x4, y4, 0));
        tf::Vector3 middle_point_world_coord = gap2WorldCoord(tf::Vector3(middle_point_x, middle_point_y, 0));


        //*** step 1 transform
        theta_delta  = direction - start_state[2];
        if(theta_delta > M_PI)  theta_delta = -2 * M_PI + theta_delta; 
        else if(theta_delta < -M_PI)  theta_delta = 2 * M_PI + theta_delta;
        theta_delta /= 100;

        joint1_delta = (0 - start_state[3]) /100;
        joint2_delta = (0 - start_state[4]) /100;
        joint3_delta = (M_PI /2 * ((transform_type)%2?-1:1) - start_state[5]) /100;



        for(int i = 0; i < 100; i++)
          {//  bad trnasform TODO
            conf_values new_state;
            new_state.x = start_state[0];
            new_state.y = start_state[1];
            new_state.theta =  start_state[2] + i * theta_delta;
            new_state.joint1 = start_state[3] + i * joint1_delta;
            new_state.joint2 = start_state[4] + i * joint2_delta;
            new_state.joint3 = start_state[5] + i * joint3_delta;
            original_path.push_back(new_state);
          }


        //*** step2: access to first pint
        delta_x = (xy1_world_coord.getX() - start_state[0])/100;
        delta_y = (xy1_world_coord.getY() - start_state[1])/100;

        for(int i = 0; i < 100; i++)
          {// bad access TODO
            conf_values new_state;
            new_state.x = start_state[0] + delta_x * i;
            new_state.y = start_state[1] + delta_y * i;
            new_state.theta =  direction;
            new_state.joint1 = 0;
            new_state.joint2 = 0;
            new_state.joint3 = M_PI /2 * ((transform_type)%2?-1:1);
            original_path.push_back(new_state);
          }


        //step 3: first insert
        delta_x = (xy2_world_coord.getX() - xy1_world_coord.getX())/100;
        delta_y = (xy2_world_coord.getY() - xy1_world_coord.getY())/100;
        for(int i = 0; i < 100; i++)
          {
            conf_values new_state;
            new_state.x = xy1_world_coord.getX() + delta_x * i;
            new_state.y = xy1_world_coord.getY() + delta_y * i;
            new_state.theta =  direction;
            new_state.joint1 = 0;
            new_state.joint2 = 0;
            new_state.joint3 = M_PI /2 * ((transform_type)%2?-1:1);
            original_path.push_back(new_state);
          }



        //*** step4: transform in the gap
        joint1_delta = M_PI/2 * abs(transform_type)/transform_type/100;
        joint3_delta = (0 - M_PI /2 * ((transform_type)%2?-1:1))/100;

        float x_pivot = (xy2_world_coord.getX() + middle_point_world_coord.getX())/2;
        float y_pivot = (xy2_world_coord.getY() + middle_point_world_coord.getY())/2;
        float changed_direction = direction - fabs(joint1_delta)/joint1_delta * M_PI/2;
        float changed_joint1 = fabs(joint1_delta)/joint1_delta * M_PI / 2;
        for(int i = 0; i < 100; i++)
          {
            conf_values new_state;
            new_state.x = x_pivot + l * cos(direction - joint1_delta * i);
            new_state.y = y_pivot + l * sin(direction - joint1_delta * i);
            new_state.theta =  direction - joint1_delta * i;
            new_state.joint1 = joint1_delta * i;
            new_state.joint2 = 0;
            new_state.joint3 = M_PI /2 * ((transform_type)%2?-1:1);
            original_path.push_back(new_state);
          }

        for(int i = 0; i < 100; i++)
          {
            conf_values new_state;
            new_state.x = xy3_world_coord.getX();
            new_state.y = xy3_world_coord.getY();
            new_state.theta =  changed_direction;
            new_state.joint1 = changed_joint1;
            new_state.joint2 = 0;
            new_state.joint3 = M_PI /2 * ((transform_type)%2?-1:1) + joint3_delta *i;
            original_path.push_back(new_state);
          }


        //*** step6: escape
        delta_x = (xy4_world_coord.getX() - xy3_world_coord.getX())/100;
        delta_y = (xy4_world_coord.getY() - xy3_world_coord.getY())/100;
        for(int i = 0; i < 100; i++)
          {
            conf_values new_state;
            new_state.x = xy3_world_coord.getX() + delta_x * i;
            new_state.y = xy3_world_coord.getY() + delta_y * i;
            new_state.theta =  changed_direction;;
            new_state.joint1 = changed_joint1;
            new_state.joint2 = 0;
            new_state.joint3 = 0;
            original_path.push_back(new_state);
          }


        //*** step 7: transform
        joint1_delta = (goal_state[3] - fabs(joint1_delta)/joint1_delta * M_PI/2) /100;
        joint2_delta = goal_state[4] /100;
        joint3_delta = goal_state[5] /100;
        for(int i = 0; i < 100; i++)
          {
            conf_values new_state;
            new_state.x = xy4_world_coord.getX();
            new_state.y = xy4_world_coord.getY();
            new_state.theta =  changed_direction;
            new_state.joint1 = changed_joint1 + joint1_delta * i;
            new_state.joint2 = joint2_delta * i;
            new_state.joint3 = joint3_delta * i;
            original_path.push_back(new_state);
          }

        delta_x = (goal_state[0] - xy4_world_coord.getX())/100;
        delta_y = (goal_state[1] - xy4_world_coord.getY())/100;
        theta_delta = goal_state[2] - changed_direction;
        if(theta_delta > M_PI)  theta_delta = -2 * M_PI + theta_delta; 
        else if(theta_delta < -M_PI)  theta_delta = 2 * M_PI + theta_delta;
        theta_delta /= 100;


        //*** step7: to goal 
        for(int i = 0; i < 100; i++)
          {//bad TODO
            conf_values new_state;
            new_state.x = xy4_world_coord.getX() + delta_x * i;
            new_state.y = xy4_world_coord.getY() + delta_y * i;
            new_state.theta =  changed_direction + theta_delta * i;
            new_state.joint1 = goal_state[3];
            new_state.joint2 = goal_state[4];
            new_state.joint3 = goal_state[5];
            original_path.push_back(new_state);
          }

        return true;
      }
    else
      {//ku model traverse
        float delta_x, delta_y;
        float joint1_delta, joint2_delta, joint3_delta, theta_delta;

        if(centroid_type == 0)
          {//ellipse path
            if(l_r_flag == 0)
              {//left

                //*** step1: transform
                theta_delta  = 0 - start_state[2];
                if(theta_delta > M_PI)  theta_delta = -2 * M_PI + theta_delta; 
                else if(theta_delta < -M_PI)  theta_delta = 2 * M_PI + theta_delta;
                theta_delta /= 100;

                joint1_delta = (0 - start_state[3]) /100;
                joint2_delta = (-M_PI/2 - start_state[4]) /100;
                joint3_delta = (0 - start_state[5]) /100;
                for(int i = 0; i <= 100; i++)
                  {//TODO: bad transform
                    conf_values new_state;
                    new_state.x = start_state[0];
                    new_state.y = start_state[1];
                    new_state.theta =  start_state[2] + i * theta_delta;
                    new_state.joint1 = start_state[3] + i * joint1_delta;
                    new_state.joint2 = start_state[4] + i * joint2_delta;
                    new_state.joint3 = start_state[5] + i * joint3_delta;
                    original_path.push_back(new_state);
                  }

                //*** step2: access1
                tf::Vector3 access_point_world_coord 
                  = gap2WorldCoord(tf::Vector3(left_corner.getX() + d/2, left_corner.getY() ,0));
                delta_x = (access_point_world_coord.getX() - start_state[0]) /100;
                delta_y = (access_point_world_coord.getY() - start_state[1]) /100;
                for(int i = 0; i < 100; i++)
                  {//TODO: bad aces => best path planning(inclination)
                    conf_values new_state;
                    new_state.x = i* delta_x + start_state[0];
                    new_state.y = i* delta_y + start_state[1];
                    new_state.theta =  0;
                    new_state.joint1 = 0;
                    new_state.joint2 = -M_PI/2;
                    new_state.joint3 = 0;
                    original_path.push_back(new_state);
                  }
                delta_y = (2*l - d/2) / 100;
                for(int i = 0; i < 100; i++)
                  {
                    tf::Vector3 moving_world_coord 
                      = gap2WorldCoord(tf::Vector3(left_corner.getX() + d/2, left_corner.getY() + delta_y * i ,0));
                    conf_values new_state;
                    new_state.x = moving_world_coord.getX();
                    new_state.y = moving_world_coord.getY();
                    new_state.theta =  0;
                    new_state.joint1 = 0;
                    new_state.joint2 = -M_PI/2;
                    new_state.joint3 = 0;
                    original_path.push_back(new_state);
                  }

                //***step 3: traverse
                float x_ellipse, y_ellipse;
                for(float theta = M_PI; theta >=0; theta -= M_PI/100)
                  {
                    float x_theta = h/2 * sin(theta) + sqrt(2) *d * sin(M_PI/4 + theta/2);
                    float y_theta = h/2 * cos(theta) + sqrt(2) *d * cos(M_PI/4 + theta/2);
                    x_ellipse = x_theta - (2*l+d/2) * sin(M_PI/2 - theta/2) - d/2*cos(M_PI/2 - theta/2) + left_corner.getX();
                    y_ellipse = y_theta + (2*l+d/2) * cos(M_PI/2 - theta/2) - d/2*sin(M_PI/2 - theta/2) + (left_corner.getY() + left_corner.getZ()/2);

                    conf_values new_state;
                    new_state.x = gap2WorldCoord(tf::Vector3(x_ellipse,y_ellipse,0)).getX();
                    new_state.y = gap2WorldCoord(tf::Vector3(x_ellipse,y_ellipse,0)).getY();
                    new_state.theta = M_PI/2 - theta/2;
                    new_state.joint1 = 0;
                    new_state.joint2 = -M_PI/2;
                    new_state.joint3 = 0;
                    original_path.push_back(new_state);
                  }
                ROS_INFO("x:%f,y:%f", x_ellipse, y_ellipse);
                //*** step4: escape
                float x_escape = x_ellipse;
                float y_escape = left_corner.getY() + left_corner.getZ() + 2*l + 0.1; //0.1m margin
                float x_escape_world = gap2WorldCoord(tf::Vector3(x_escape, y_escape, 0)).getX();
                float y_escape_world = gap2WorldCoord(tf::Vector3(x_escape, y_escape, 0)).getY();
                float y_tmp = y_ellipse;
                while(1)
                  {
                    if(y_tmp >= y_escape) break;
                    y_tmp += 0.05;// 0.05 cm 
                    conf_values new_state;
                    new_state.x = gap2WorldCoord(tf::Vector3(x_escape, y_tmp, 0)).getX();
                    new_state.y = gap2WorldCoord(tf::Vector3(x_escape, y_tmp, 0)).getY();
                    new_state.theta =  M_PI /2;
                    new_state.joint1 = 0;
                    new_state.joint2 = -M_PI/2;
                    new_state.joint3 = 0;
                    original_path.push_back(new_state);
                  }

                //*** step5: transform
                joint1_delta = goal_state[3] /100;
                joint2_delta = (goal_state[4] - (-M_PI/2)) /100;
                joint3_delta = goal_state[5] /100;
                for(int i = 0; i < 100; ++i)
                  {
                    conf_values new_state;
                    new_state.x = x_escape_world;
                    new_state.y = y_escape_world;
                    new_state.theta =  M_PI/2;
                    new_state.joint1 = joint1_delta * i;
                    new_state.joint2 = -M_PI/2 + joint2_delta * i;
                    new_state.joint3 = joint3_delta * i;
                    original_path.push_back(new_state);
                  }

                //*** step6: to goal
                delta_x = (goal_state[0] - x_escape_world)/100;
                delta_y = (goal_state[1] - y_escape_world)/100;
                theta_delta = goal_state[2] - M_PI/2;
                if(theta_delta > M_PI)  theta_delta = -2 * M_PI + theta_delta; 
                else if(theta_delta < -M_PI)  theta_delta = 2 * M_PI + theta_delta;
                theta_delta /= 100;
                for(int i = 0; i < 100; ++i)
                  {//bad TODO
                    conf_values new_state;
                    new_state.x = x_escape_world + delta_x * i;
                    new_state.y = y_escape_world + delta_y * i;
                    new_state.theta =  M_PI/2 + theta_delta * i;
                    new_state.joint1 = goal_state[3];
                    new_state.joint2 = goal_state[4];
                    new_state.joint3 = goal_state[5];
                    original_path.push_back(new_state);
                  }
              }
            else
              {//right  ** mocap(link39 base planning!! **
                /*
                  start_state[0]: mocap_x
                  start_state[1]: mocap_y
                  start_state[2]: mocap_psi
                  start_state[3]: joint1
                  start_state[4]: joint2
                  start_state[5]: joint2
 
                  mocap center to link center, according to link coord 
                  (mocap_center_to_link_center_x_, mocap_center_to_link_center_y_)
                */
                conf_values new_state_link1, new_state_link3;

                //*** step1: transform
                theta_delta  = M_PI/2 - start_state[2];
                if(theta_delta > M_PI)  theta_delta = -2 * M_PI + theta_delta; 
                else if(theta_delta < -M_PI)  theta_delta = 2 * M_PI + theta_delta;
                theta_delta /= 100;

                joint1_delta = (0 - start_state[3]) /100;
                joint2_delta = (M_PI/2 - start_state[4]) /100;
                joint3_delta = (0 - start_state[5]) /100;
                for(int i = 0; i <= 100; i++)
                  {//TODO: bad transform
                    //for link1 visualization
                    new_state_link1.x = start_state[0] + (mocap_center_to_link_center_x_ + l/2) * cos(new_state_link1.theta) - mocap_center_to_link_center_y_ * sin(new_state_link1.theta) + l * cos(new_state_link1.theta - new_state_link1.joint2) + l * cos(new_state_link1.theta - new_state_link1.joint1 - new_state_link1.joint2);
                    new_state_link1.y = start_state[1] + (mocap_center_to_link_center_x_ + l\2) * sin(new_state_link1.theta) + mocap_center_to_link_center_y_ * cos(new_state_link1.theta) + l * sin(new_state_link1.theta - new_state_link1.joint2) + l * sin(new_state_link1.theta - new_state_link1.joint1 - new_state_link1.joint2);
                    new_state_link1.theta =  start_state[2] + i * (theta_delta) - new_state_link1.joint1 - new_state_link1.joint2;
                    new_state_link1.joint1 = start_state[3] + i * joint1_delta;
                    new_state_link1.joint2 = start_state[4] + i * joint2_delta;
                    new_state_link1.joint3 = start_state[5] + i * joint3_delta;
                    original_path.push_back(new_state_link1);
                    //for link which contains mocap , that is, the link3 in this system
                    new_state_link3.x = start_state[0] + mocap_center_to_link_center_x_ * cos(start_state[2]) - mocap_center_to_link_center_y_ * sin(start_state[2]);
                    new_state_link3.y = start_state[1] + mocap_center_to_link_center_x_ * sin(start_state[2]) + mocap_center_to_link_center_y_ * cos(start_state[2]);
                    new_state_link3.theta =  start_state[2] + i * theta_delta;
                    new_state_link3.joint1 = start_state[3] + i * joint1_delta;
                    new_state_link3.joint2 = start_state[4] + i * joint2_delta;
                    new_state_link3.joint3 = start_state[5] + i * joint3_delta;
                    real_robot_path_.push_back(new_state_link3);
                  }
                //**** for real robot move
                //TODO: trasnformation();


                //*** step2: access1
                float init_link1_x = new_state_link1.x;
                float init_link1_y = new_state_link1.y;
                float init_link3_x = new_state_link3.x;
                float init_link3_y = new_state_link3.y;
                tf::Vector3 access_point_world_coord 
                  = gap2WorldCoord(tf::Vector3(right_corner.getX() - d/2, right_corner.getY() ,0));
                delta_x = (access_point_world_coord.getX() - init_link1_x) /100;
                delta_y = (access_point_world_coord.getY() - init_link1_y) /100;
                for(int i = 0; i < 100; i++)
                  {
                    //for link1 visualization
                    new_state_link1.x = i* delta_x + init_link1_x;
                    new_state_link1.y = i* delta_y + init_link1_y;
                    new_state_link1.theta =  0;
                    new_state_link1.joint1 = 0;
                    new_state_link1.joint2 = M_PI/2;
                    new_state_link1.joint3 = 0;
                    original_path.push_back(new_state_link1);
                    //for link3 visualization
                    new_state_link3.x = i* delta_x + init_link3_x;
                    new_state_link3.y = i* delta_y + init_link3_y;
                    new_state_link3.theta =  M_PI/2;
                    new_state_link3.joint1 = 0;
                    new_state_link3.joint2 = M_PI/2;
                    new_state_link3.joint3 = 0;
                    real_robot_path_.push_back(new_state_link3);
                  }

                init_link1_x = new_state_link1.x;
                init_link1_y = new_state_link1.y;
                init_link3_x = new_state_link3.x;
                init_link3_y = new_state_link3.y;
                access_point_world_coord = gap2WorldCoord(tf::Vector3(right_corner.getX() - d/2, right_corner.getY() + 2*l -d/2, 0));
                delta_x = (access_point_world_coord.getX() - init_link1_x) /100;
                delta_y = (access_point_world_coord.getY() - init_link1_y) /100;
                for(int i = 0; i < 100; i++)
                  {
                    //for link1 visualization
                    new_state_link1.x = i* delta_x + init_link1_x;
                    new_state_link1.y = i* delta_y + init_link1_y;
                    new_state_link1.theta =  0;
                    new_state_link1.joint1 = 0;
                    new_state_link1.joint2 = M_PI/2;
                    new_state_link1.joint3 = 0;
                    original_path.push_back(new_state_link1);
                    //for link3 visualization
                    new_state_link3.x = i* delta_x + init_link3_x;
                    new_state_link3.y = i* delta_y + init_link3_y;
                    new_state_link3.theta =  M_PI/2;
                    new_state_link3.joint1 = 0;
                    new_state_link3.joint2 = M_PI/2;
                    new_state_link3.joint3 = 0;
                    real_robot_path_.push_back(new_state_link3);
                  }


                //***step 3: traverse
                float x_link1, y_link1, x_link3, y_link3;
                for(float theta = M_PI; theta >=0; theta -= M_PI/100)
                  {
                    float x_theta = -h/2 * sin(theta);
                    float y_theta = h/2 * cos(theta);
                    //for link1 visualization
                    x_link1 = x_theta +  (2*l-d/2) * cos(theta/2) - d/2*sin(theta/2) + right_corner.getX();
                    y_link1 = y_theta + (2*l-d/2) * sin(theta/2) + d/2*cos(theta/2) + right_corner.getY() + right_corner.getZ()/2;

                    new_state_link1.x = gap2WorldCoord(tf::Vector3(x_link1,y_link1,0)).getX();
                    new_state_link1.y = gap2WorldCoord(tf::Vector3(x_link1,y_link1,0)).getY();
                    new_state_link1.theta = -M_PI/2 + theta/2;
                    new_state_link1.joint1 = 0;
                    new_state_link1.joint2 = M_PI/2;
                    new_state_link1.joint3 = 0;
                    original_path.push_back(new_state_link1);
                    //for link3 visualization
                    new_state_link3.x = new_state_link1.x + fdfdsf(bakui, mocap link)
                    new_state_link3.y = 
                    new_state_link3.theta = theta/2;
                    new_state_link3.joint1 = 0;
                    new_state_link3.joint2 = M_PI/2;
                    new_state_link3.joint3 = 0;
                    real_robot_path_.push_back(new_state_link3);
                  }

                //*** step4: escape
                float x_escape = x_ellipse;
                float y_escape = right_corner.getY() + right_corner.getZ() + 2*l + 0.1; //0.1m margin
                float x_escape_world = gap2WorldCoord(tf::Vector3(x_escape, y_escape, 0)).getX();
                float y_escape_world = gap2WorldCoord(tf::Vector3(x_escape, y_escape, 0)).getY();
                float y_tmp = y_ellipse;
                while(1)
                  {
                    if(y_tmp >= y_escape) break;
                    y_tmp += 0.05;// 0.05 cm 
                    conf_values new_state;
                    new_state.x = gap2WorldCoord(tf::Vector3(x_escape, y_tmp, 0)).getX();
                    new_state.y = gap2WorldCoord(tf::Vector3(x_escape, y_tmp, 0)).getY();
                    new_state.theta =  -M_PI /2;
                    new_state.joint1 = 0;
                    new_state.joint2 = M_PI/2;
                    new_state.joint3 = 0;
                    original_path.push_back(new_state);
                  }

                //*** step5: transform
                joint1_delta = goal_state[3] /100;
                joint2_delta = (goal_state[4] - M_PI/2) /100;
                joint3_delta = goal_state[5] /100;
                for(int i = 0; i < 100; ++i)
                  {
                    conf_values new_state;
                    new_state.x = x_escape_world;
                    new_state.y = y_escape_world;
                    new_state.theta =  -M_PI/2;
                    new_state.joint1 = joint1_delta * i;
                    new_state.joint2 = M_PI/2 + joint2_delta * i;
                    new_state.joint3 = joint3_delta * i;
                    original_path.push_back(new_state);
                  }

                //*** step6: to goal
                delta_x = (goal_state[0] - x_escape_world)/100;
                delta_y = (goal_state[1] - y_escape_world)/100;
                theta_delta = goal_state[2] - ( - M_PI/2);
                if(theta_delta > M_PI)  theta_delta = -2 * M_PI + theta_delta; 
                else if(theta_delta < -M_PI)  theta_delta = 2 * M_PI + theta_delta;
                theta_delta /= 100;
                for(int i = 0; i < 100; ++i)
                  {//bad TODO
                    conf_values new_state;
                    new_state.x = x_escape_world + delta_x * i;
                    new_state.y = y_escape_world + delta_y * i;
                    new_state.theta =  -M_PI/2 + theta_delta * i;
                    new_state.joint1 = goal_state[3];
                    new_state.joint2 = goal_state[4];
                    new_state.joint3 = goal_state[5];
                    original_path.push_back(new_state);
                  }
              }
          }
        else if(centroid_type == -1)
          {//under parabola
            if(l_r_flag == 0)
              {//left
                //*** step1: transform
                theta_delta  = -M_PI/4 - start_state[2];
                if(theta_delta > M_PI)  theta_delta = -2 * M_PI + theta_delta; 
                else if(theta_delta < -M_PI)  theta_delta = 2 * M_PI + theta_delta;
                theta_delta /= 100;

                joint1_delta = (0 - start_state[3]) /100;
                joint2_delta = (-M_PI/2 - start_state[4]) /100;
                joint3_delta = (0 - start_state[5]) /100;

                for(int i = 0; i <= 100; i++)
                  {//TODO: bad transform
                    conf_values new_state;
                    new_state.x = start_state[0];
                    new_state.y = start_state[1];
                    new_state.theta =  start_state[2] + i * theta_delta;
                    new_state.joint1 = start_state[3] + i * joint1_delta;
                    new_state.joint2 = start_state[4] + i * joint2_delta;
                    new_state.joint3 = start_state[5] + i * joint3_delta;
                    original_path.push_back(new_state);
                  }

                //step 2: access
                tf::Vector3 access_coord = gap2WorldCoord(tf::Vector3(left_corner.getX() + d/2*sqrt(2)/2, left_corner.getY() - d/2*sqrt(2)/2,0));
                float delta_x = (access_coord.getX() - start_state[0]) /100;
                float delta_y = (access_coord.getY() - start_state[1]) /100;
                for(int i = 0; i < 100; i++)
                  {//TODO: best path planning(inclination)
                    conf_values new_state;
                    new_state.x = i*delta_x + start_state[0];
                    new_state.y = i*delta_y + start_state[1];
                    new_state.theta =  -M_PI/4;
                    new_state.joint1 = 0;
                    new_state.joint2 = -M_PI/2;
                    new_state.joint3 = 0;
                    original_path.push_back(new_state);
                  }

                //step 3: traverse
                float x_parabola, y_parabola;
                for(float theta = M_PI/4; theta >=0; theta -=M_PI/400)
                  {
                    float y_theta = (2*l-d/2) * sin(theta) + sqrt(2)*d* sin(M_PI/4 + theta) + h/2 ;
                    float x_theta = -(2*l-d/2) * sin(theta) * tan(theta) + sqrt(2)* d * cos(M_PI/4 + theta);
                    x_parabola = x_theta + (2*l+d/2) * sin(theta) - d/2*cos(theta) + left_corner.getX();
                    y_parabola = -y_theta + (2*l+d/2) * cos(theta) + d/2*sin(theta) + (left_corner.getY() + left_corner.getZ()/2);

                    tf::Vector3 parabola_coord = gap2WorldCoord(tf::Vector3(x_parabola, y_parabola, 0));
                    conf_values new_state;
                    new_state.x = parabola_coord.getX();
                    new_state.y = parabola_coord.getY();
                    new_state.theta = -theta;
                    new_state.joint1 = 0;
                    new_state.joint2 = -M_PI/2;
                    new_state.joint3 = 0;
                    original_path.push_back(new_state);
                  }

                //step 4: escape
                float x_escape = left_corner.getX() + 2 * l + 0.1; //0.1m margin
                float y_escape = y_parabola;
                float x_escape_world = gap2WorldCoord(tf::Vector3(x_escape, y_escape, 0)).getX();
                float y_escape_world = gap2WorldCoord(tf::Vector3(x_escape, y_escape, 0)).getY();
                float x_tmp = x_parabola;
                while(1)
                  {
                    if(x_tmp >= x_escape) break;
                    x_tmp += 0.05;// 0.05 cm
                    tf::Vector3 world_coord = gap2WorldCoord(tf::Vector3(x_tmp, y_escape, 0));
                    conf_values new_state;
                    new_state.x = world_coord.getX();
                    new_state.y = world_coord.getY();
                    new_state.theta =  0;
                    new_state.joint1 = 0;
                    new_state.joint2 = -M_PI/2;
                    new_state.joint3 = 0;
                    original_path.push_back(new_state);
                  }

                //*** step5: transform
                joint1_delta = goal_state[3] /100;
                joint2_delta = (goal_state[4] - (-M_PI/2)) /100;
                joint3_delta = goal_state[5] /100;
                
                for(int i = 0; i < 100; ++i)
                  {
                    conf_values new_state;
                    new_state.x = x_escape_world;
                    new_state.y = y_escape_world;
                    new_state.theta =  0;
                    new_state.joint1 = joint1_delta * i;
                    new_state.joint2 = -M_PI/2 + joint2_delta * i;
                    new_state.joint3 = joint3_delta * i;
                    original_path.push_back(new_state);
                  }

                //*** step6: to goal
                delta_x = (goal_state[0] - x_escape_world)/100;
                delta_y = (goal_state[1] - y_escape_world)/100;
                theta_delta = goal_state[2];
                if(theta_delta > M_PI)  theta_delta = -2 * M_PI + theta_delta; 
                else if(theta_delta < -M_PI)  theta_delta = 2 * M_PI + theta_delta;
                theta_delta /= 100;
                for(int i = 0; i < 100; ++i)
                  {//bad TODO
                    conf_values new_state;
                    new_state.x = x_escape_world + delta_x * i;
                    new_state.y = y_escape_world + delta_y * i;
                    new_state.theta =  theta_delta * i;
                    new_state.joint1 = goal_state[3];
                    new_state.joint2 = goal_state[4];
                    new_state.joint3 = goal_state[5];
                    original_path.push_back(new_state);
                  }
              }
            else
              {//right 
                //TODO

              }
          }
        else if(centroid_type == 1)
          {//top parabola
            if(l_r_flag == 0)
              {//left
                //*** step1: transform
                theta_delta  = M_PI/2 - start_state[2];
                if(theta_delta > M_PI)  theta_delta = -2 * M_PI + theta_delta; 
                else if(theta_delta < -M_PI)  theta_delta = 2 * M_PI + theta_delta;
                theta_delta /= 100;

                joint1_delta = (0 - start_state[3]) /100;
                joint2_delta = (-M_PI/2 - start_state[4]) /100;
                joint3_delta = (0 - start_state[5]) /100;

                for(int i = 0; i <= 100; i++)
                  {//TODO: bad transform
                    conf_values new_state;
                    new_state.x = start_state[0];
                    new_state.y = start_state[1];
                    new_state.theta =  start_state[2] + i * theta_delta;
                    new_state.joint1 = start_state[3] + i * joint1_delta;
                    new_state.joint2 = start_state[4] + i * joint2_delta;
                    new_state.joint3 = start_state[5] + i * joint3_delta;
                    original_path.push_back(new_state);
                  }

                //step 2: access
                tf::Vector3 access_point_world_coord = gap2WorldCoord(tf::Vector3(left_corner.getX(), left_corner.getY() + left_corner.getZ()+ d/2,0));
                delta_x = (access_point_world_coord.getX() - start_state[0]) /100;
                delta_y = (access_point_world_coord.getY() - start_state[1]) /100;
                for(int i = 0; i < 100; ++i)
                  {//TODO: best path planning(inclination)
                    conf_values new_state;
                    new_state.x = i*delta_x + start_state[0];
                    new_state.y = i*delta_y + start_state[1];
                    new_state.theta =  M_PI/2;
                    new_state.joint1 = 0;
                    new_state.joint2 = -M_PI/2;
                    new_state.joint3 = 0;
                    original_path.push_back(new_state);
                  }
                delta_x = -(2*l -d) / 100;
                for(int i = 0; i < 100; ++i)
                  {//access2
                    tf::Vector3 moving_world_coord 
                      = gap2WorldCoord(tf::Vector3(left_corner.getX() + delta_x * i, left_corner.getY() + left_corner.getZ()+ d/2,0));

                    conf_values new_state;
                    new_state.x = moving_world_coord.getX();
                    new_state.y = moving_world_coord.getY();
                    new_state.theta =  M_PI/2;
                    new_state.joint1 = 0;
                    new_state.joint2 = -M_PI/2;
                    new_state.joint3 = 0;
                    original_path.push_back(new_state);
                  }
                //*** step3: traverse
                float x_parabola, y_parabola;
                for(float theta = 0; theta <=M_PI/4; theta +=M_PI/400)
                  {

                    float y_theta = (2*l-d/2) * sin(theta) + sqrt(2)*d* sin(M_PI/4 + theta) + h/2;
                    float x_theta = -(2*l-d/2) * sin(theta) * tan(theta) + sqrt(2)* d * cos(M_PI/4 + theta);
                    x_parabola = x_theta - (2*l+d/2) * cos(theta) + d/2*sin(theta) + left_corner.getX();
                    y_parabola = y_theta - (2*l+d/2) * sin(theta) - d/2*cos(theta) + (left_corner.getY() + left_corner.getZ()/2);

                    tf::Vector3 world_coord = gap2WorldCoord(tf::Vector3(x_parabola, y_parabola, 0));
                    conf_values new_state;
                    new_state.x = world_coord.getX();
                    new_state.y = world_coord.getY();
                    new_state.theta = theta + M_PI/2;
                    new_state.joint1 = 0;
                    new_state.joint2 = -M_PI/2;
                    new_state.joint3 = 0;
                    original_path.push_back(new_state);
                  }

                //*** step4: escape
                float x_escape = right_corner.getX() - 2 *sqrt(2)* l - 0.1; //0.1m margin
                float y_escape = y_parabola;
                float x_escape_world = gap2WorldCoord(tf::Vector3(x_escape, y_escape, 0)).getX();
                float y_escape_world = gap2WorldCoord(tf::Vector3(x_escape, y_escape, 0)).getY();

                float x_tmp = x_parabola;
                float y_tmp = y_parabola;
                while(1)
                  {
                    if(x_tmp <= x_escape) break;
                    x_tmp -= 0.05;// 0.05 cm
                    tf::Vector3 world_coord = gap2WorldCoord(tf::Vector3(x_tmp, y_escape, 0));
                    conf_values new_state;
                    new_state.x = world_coord.getX();
                    new_state.y = world_coord.getY();
                    new_state.theta =  3* M_PI /4;
                    new_state.joint1 = 0;
                    new_state.joint2 = -M_PI/2;
                    new_state.joint3 = 0;
                    original_path.push_back(new_state);
                  }
                y_escape = y_parabola + 0.6; //0.6m
                x_escape_world = gap2WorldCoord(tf::Vector3(x_escape, y_escape, 0)).getX();
                y_escape_world = gap2WorldCoord(tf::Vector3(x_escape, y_escape, 0)).getY();
                while(1)
                  {
                    if(y_tmp >= y_escape) break;
                    y_tmp += 0.05;// 0.05 cm
                    tf::Vector3 world_coord = gap2WorldCoord(tf::Vector3(x_escape, y_tmp, 0));
                    conf_values new_state;
                    new_state.x = world_coord.getX();
                    new_state.y = world_coord.getY();
                    new_state.theta =  3* M_PI /4;
                    new_state.joint1 = 0;
                    new_state.joint2 = -M_PI/2;
                    new_state.joint3 = 0;
                    original_path.push_back(new_state);
                  }


                //*** step5: transform
                joint1_delta = goal_state[3] /100;
                joint2_delta = (goal_state[4] - (-3* M_PI /4)) /100;
                joint3_delta = goal_state[5] /100;
                for(int i = 0; i < 100; ++i)
                  {
                    conf_values new_state;
                    new_state.x = x_escape_world;
                    new_state.y = y_escape_world;
                    new_state.theta =  3* M_PI /4;
                    new_state.joint1 = joint1_delta * i;
                    new_state.joint2 = -M_PI/2 + joint2_delta * i;
                    new_state.joint3 = joint3_delta * i;
                    original_path.push_back(new_state);
                  }

                //*** step6: to goal
                delta_x = (goal_state[0] - x_escape_world)/100;
                delta_y = (goal_state[1] - y_escape_world)/100;
                theta_delta = goal_state[2] - 3* M_PI /4;
                if(theta_delta > M_PI)  theta_delta = -2 * M_PI + theta_delta; 
                else if(theta_delta < -M_PI)  theta_delta = 2 * M_PI + theta_delta;
                theta_delta /= 100;
                for(int i = 0; i < 100; ++i)
                  {//bad TODO
                    conf_values new_state;
                    new_state.x = x_escape_world + delta_x * i;
                    new_state.y = y_escape_world + delta_y * i;
                    new_state.theta =  3* M_PI /4 + theta_delta * i;
                    new_state.joint1 = goal_state[3];
                    new_state.joint2 = goal_state[4];
                    new_state.joint3 = goal_state[5];
                    original_path.push_back(new_state);
                  }

                //TODO: to the goal!!
              }
            else
              {//right 
                //TODO
              }
          }
          return true;
      }
    ROS_ERROR("bad dectection");
    return false;
  }
  

  ~PlanningScene()
  {
    delete robot_model_loader_;
    delete planning_scene_;
    delete plan_states_;
    delete transform_controller_;
  }

};

int main(int argc, char **argv)
{
  ros::init (argc, argv, "hydra_collision_detection");
  //ros::AsyncSpinner spinner(1);
  //spinner.start();

  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  
  PlanningScene *planning_scene = new PlanningScene(nh,nhp);
  ros::spin();
  ROS_INFO("ok6");

  ros::shutdown(); 
  delete planning_scene;


 return 0;
}
