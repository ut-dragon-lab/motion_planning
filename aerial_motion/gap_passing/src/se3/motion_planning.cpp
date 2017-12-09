// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* TODO: implemtn SE(2) + alt or R^3 x SO(2), the former one is better */
/* may be not need: the desire tilt angle of cog(baselink) can be validated in the state validation method */


#include <gap_passing/se3/motion_control.h>
#include <gap_passing/se3/motion_planning.h>

/* the StabilityObjective does not work!!!!! */
namespace se3
{
  MotionPlanning::MotionPlanning(ros::NodeHandle nh, ros::NodeHandle nhp) :nh_(nh), nhp_(nhp)
  {
    transform_controller_ = boost::shared_ptr<DragonTransformController>(new DragonTransformController(nh_, nhp_, false));
    joint_num_ = (transform_controller_->getRotorNum()  - 1) * 2 ;

    rosParamInit();

    motion_control_ = new MotionControl(nh, nhp, transform_controller_);

    planning_scene_diff_pub_ = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

    if(planning_mode_ != gap_passing::PlanningMode::ONLY_JOINTS_MODE)
      {
        while(planning_scene_diff_pub_.getNumSubscribers() < 1)
          {
            ros::WallDuration sleep_t(0.5);
            sleep_t.sleep();
          }
        ros::Duration sleep_time(1.0);
        sleep_time.sleep();

        robot_model_loader_ = new robot_model_loader::RobotModelLoader("robot_description");
        kinematic_model_ = robot_model_loader_->getModel();

        /* for collision check */
        planning_scene_ = new planning_scene::PlanningScene(kinematic_model_);
        acm_ = planning_scene_->getAllowedCollisionMatrix();

        gapEnvInit();

      }

    calculation_time_ = 0;
    best_cost_ = -1;

    if(!play_log_path_) Planning();
    motion_sequence_timer_ = nhp_.createTimer(ros::Duration(1.0 / motion_sequence_rate_), &MotionPlanning::motionSequenceFunc, this);

  }

  MotionPlanning::~MotionPlanning()
  {
    if(planning_mode_ != gap_passing::PlanningMode::ONLY_JOINTS_MODE)
      {
        delete robot_model_loader_;
        delete planning_scene_;
      }

    delete plan_states_;
    delete rrt_start_planner_;
    delete path_length_opt_objective_;
    delete motion_control_;
  }

  void MotionPlanning::motionSequenceFunc(const ros::TimerEvent &e)
  {
    static bool first_flag = true;

    static int state_index = 0;
    static std::vector<conf_values> planning_path;

    if(solved_ && ros::ok())
      {
        if(first_flag)
          {
            ROS_WARN("plan size is %d, planning time is %f, motion cost is %f", motion_control_->getPathSize(), motion_control_->getPlanningTime(), motion_control_->getMotionCost());
            ROS_WARN("min var is %f in state %d; max force is %f in state %d",
                     motion_control_->getMinVar(),
                     motion_control_->getMinVarStateIndex(),
                     motion_control_->getMaxForce(),
                     motion_control_->getMaxForceStateIndex());

            planning_path.resize(0);
            motion_control_->getPlanningPath(planning_path);

            first_flag = false;
          }
        else
          {
            if(planning_mode_ == gap_passing::PlanningMode::ONLY_JOINTS_MODE)
              {
                sensor_msgs::JointState joint_state_msg;
                joint_state_msg.header.stamp = ros::Time::now();
                if(planning_path[state_index].state_values.size() != 7 + joint_num_)
                  ROS_ERROR("wrong size of planning joint state and this state");
                joint_state_msg.position.resize(0);
                joint_state_msg.name.resize(0);
                for(int i = 0; i < joint_num_; i ++)
                  {
                    std::stringstream joint_no2;
                    joint_no2 << i + 1;
                    joint_state_msg.name.push_back(std::string("joint") + joint_no2.str());
                    joint_state_msg.position.push_back(planning_path[state_index].state_values[7 + i]);
                  }
                motion_control_->joint_cmd_pub_.publish(joint_state_msg);
              }
            else
              {
                setRobotState2Moveit(motion_control_->getState(state_index).state_values);
                planning_scene_->getPlanningSceneMsg(planning_scene_msg_);
                planning_scene_msg_.is_diff = true;
                planning_scene_diff_pub_.publish(planning_scene_msg_);
              }

            state_index ++;
            if(state_index ==  motion_control_->getPathSize()) state_index = 0;
          }

      }
  }

  bool  MotionPlanning::isStateValid(const ompl::base::State *state)
  {
    std::vector<double> current_state(7 + joint_num_, 0); // SE3 + R^n
    current_state[6] = 1.0; //q.w();

    if(planning_mode_ == gap_passing::PlanningMode::ONLY_JOINTS_MODE)
      {
        for(int i = 0; i < joint_num_; i++)
          current_state[7 + i] = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i];
      }
    else if(planning_mode_ == gap_passing::PlanningMode::ONLY_BASE_MODE)
      {
        if(locomotion_mode_ == gap_passing::PlanningMode::SE2)
          {
            current_state[0] = state->as<ompl::base::SE2StateSpace::StateType>()->getX();
            current_state[1] = state->as<ompl::base::SE2StateSpace::StateType>()->getY();
            current_state[2] = 0; //Z

            /* we have to convert from se2 to se3 */
            //current_state[2] = state->as<ompl::base::SE3StateSpace::StateType>()->getYaw();
            tf::Quaternion q;
            q.setRPY(0, 0, state->as<ompl::base::SE2StateSpace::StateType>()->getYaw());
            current_state[3] = q.x();
            current_state[4] = q.y();
            current_state[5] = q.z();
            current_state[6] = q.w();
          }

        if(locomotion_mode_ == gap_passing::PlanningMode::SE3)
          {
            current_state[0] = state->as<ompl::base::SE3StateSpace::StateType>()->getX();
            current_state[1] = state->as<ompl::base::SE3StateSpace::StateType>()->getY();
            current_state[2] = state->as<ompl::base::SE3StateSpace::StateType>()->getZ();
            current_state[3] = state->as<ompl::base::SE3StateSpace::StateType>()->rotation().x;
            current_state[4] = state->as<ompl::base::SE3StateSpace::StateType>()->rotation().y;
            current_state[5] = state->as<ompl::base::SE3StateSpace::StateType>()->rotation().z;
            current_state[6] = state->as<ompl::base::SE3StateSpace::StateType>()->rotation().w;
          }

        for(int i = 0; i < joint_num_ ; i++)
          current_state[7 + i] = start_state_[7 + i];

      }
    else if(planning_mode_ == gap_passing::PlanningMode::JOINTS_AND_BASE_MODE)
      {
        const ompl::base::CompoundState* state_tmp = dynamic_cast<const ompl::base::CompoundState*>(state);
        if(locomotion_mode_ == gap_passing::PlanningMode::SE2)
          {
            current_state[0] = state_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->getX();
            current_state[1] = state_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->getY();
            current_state[2] = 0; //Z

            /* we have to convert from se2 to se3 */
            //current_state[2] = state_tmp->as<ompl::base::SE3StateSpace::StateType>()->getYaw();
            tf::Quaternion q;
            q.setRPY(0, 0, state_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->getYaw());
            current_state[3] = q.x();
            current_state[4] = q.y();
            current_state[5] = q.z();
            current_state[6] = q.w();
          }

        if(locomotion_mode_ == gap_passing::PlanningMode::SE3)
          {
            current_state[0] = state_tmp->as<ompl::base::SE3StateSpace::StateType>(0)->getX();
            current_state[1] = state_tmp->as<ompl::base::SE3StateSpace::StateType>(0)->getY();
            current_state[2] = state_tmp->as<ompl::base::SE3StateSpace::StateType>(0)->getZ();
            current_state[3] = state_tmp->as<ompl::base::SE3StateSpace::StateType>(0)->rotation().x;
            current_state[4] = state_tmp->as<ompl::base::SE3StateSpace::StateType>(0)->rotation().y;
            current_state[5] = state_tmp->as<ompl::base::SE3StateSpace::StateType>(0)->rotation().z;
            current_state[6] = state_tmp->as<ompl::base::SE3StateSpace::StateType>(0)->rotation().w;
          }
        for(int i = 0; i < joint_num_ ; i++)
          current_state[7 + i] = state_tmp->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[i];
      }

    //check distance thresold
    if(stability_opt_weight_ == 0)
      {//only path length opt, we have to judge the form whethe valid or not
        sensor_msgs::JointState joint_state;
        /* special for the dragon kinematics */
        for(int i = 0; i < joint_num_ / 2; i++)
          {
            std::stringstream ss;
            ss << i + 1;
            joint_state.name.push_back(std::string("joint") + ss.str() + std::string("_pitch"));
            joint_state.position.push_back(current_state[7 + 2 * i]);
            joint_state.name.push_back(std::string("joint") + ss.str() + std::string("_yaw"));
            joint_state.position.push_back(current_state[7 + 2 * i + 1]);
          }
        /* gimbal */
        for(int j = 0; j <= joint_num_ / 2; j++)
          {
            std::stringstream ss;
            ss << j + 1;
            joint_state.name.push_back(std::string("gimbal") + ss.str() + std::string("_roll"));
            joint_state.position.push_back(0);
            joint_state.name.push_back(std::string("gimbal") + ss.str() + std::string("_pitch"));
            joint_state.position.push_back(0);
          }

        /* need to set the cog coodrinate for dragon */
        transform_controller_->setCogDesireOrientation(KDL::Rotation::Quaternion(current_state[3],
                                                                                 current_state[4],
                                                                                 current_state[5],
                                                                                 current_state[6]));
        transform_controller_->kinematics(joint_state);
        double dist_thre_check = transform_controller_->distThreCheck();

        if(dist_thre_check == 0)
          {
            return false;
          }

        if(!transform_controller_->overlapCheck())
          {
            return false;
          }

        if(!transform_controller_->modelling())
          {
            return false;
          }
      }

    //set the nominal gimbal angles for moveit collision check
    std::vector<double> gimbal_nominal_angles = transform_controller_->getGimbalNominalAngles();
    for(auto itr = gimbal_nominal_angles.begin(); itr != gimbal_nominal_angles.end(); ++itr)
      current_state.push_back(*itr); // roll -> pitch -> roll ->pitch

    //check collision
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;

    robot_state::RobotState robot_state = setRobotState2Moveit(current_state);
    planning_scene_->checkCollision(collision_request, collision_result, robot_state, acm_);

    if(collision_result.collision)
      {
        return false;
      }
    return true;
  }

  void MotionPlanning::gapEnvInit()
  {
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = "world";
    collision_object.id = "box";

    /* gap type 1: vertical gap */
    if(gap_type_ == HORIZONTAL_GAP)
      {
        double gap_left_x;
        double gap_x_offset;
        double gap_y_offset;
        double gap_left_width;
        double gap_right_width;

        nhp_.param("gap_left_x", gap_left_x, 0.0);
        nhp_.param("gap_x_offset", gap_x_offset, 0.0); //minus: overlap
        nhp_.param("gap_y_offset", gap_y_offset, 0.6); //minus: overlap
        nhp_.param("gap_left_width", gap_left_width, 0.3); //minus: bandwidth
        nhp_.param("gap_right_width", gap_right_width, 0.3); //minus: bandwidth

        ROS_ERROR("gap_y_offset: %f", gap_y_offset);
        geometry_msgs::Pose pose1, pose2,pose3,pose4;
        pose1.position.x = gap_left_x + gap_left_width /2;
        pose1.position.y =  (2.5 + gap_y_offset) /2;
        pose1.position.z = 0.0;
        pose1.orientation.w = 1.0;

        pose2.position.x = gap_left_x + gap_right_width /2;
        pose2.position.y = - (2.5 + gap_y_offset) /2;
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

        primitive1.dimensions[0] = gap_left_width;
        primitive1.dimensions[1] = 2.5;
        primitive1.dimensions[2] = 10;
        collision_object.primitives.push_back(primitive1);
        collision_object.primitive_poses.push_back(pose1);
        primitive2.type = primitive2.BOX;
        primitive2.dimensions.resize(3);
        primitive2.dimensions[0] = gap_right_width;
        primitive2.dimensions[1] = 2.5;
        primitive2.dimensions[2] = 10;
        collision_object.primitives.push_back(primitive2);
        collision_object.primitive_poses.push_back(pose2);
        primitive3.type = primitive3.BOX;
        primitive3.dimensions.resize(3);
        primitive3.dimensions[0] = 8;
        primitive3.dimensions[1] = 0.6;
        primitive3.dimensions[2] = 10;
        collision_object.primitives.push_back(primitive3);
        collision_object.primitive_poses.push_back(pose3);
        primitive4.type = primitive4.BOX;
        primitive4.dimensions.resize(3);
        primitive4.dimensions[0] = 8;
        primitive4.dimensions[1] = 0.6;
        primitive4.dimensions[2] = 10;
        collision_object.primitives.push_back(primitive4);
        collision_object.primitive_poses.push_back(pose4);
      }

    if(gap_type_ == VERTICAL_GAP)
      {
        double gap_x_width, gap_y_width, gap_height;
        double wall_length = 10;

        nhp_.param("gap_x_width", gap_x_width, 1.0);
        nhp_.param("gap_y_width", gap_y_width, 1.0);
        nhp_.param("gap_height", gap_height, 0.3);

        geometry_msgs::Pose pose1, pose2,pose3,pose4;
        pose1.position.x = gap_x_width / 2 + wall_length / 2;
        pose1.position.y = 0;
        pose1.position.z = gap_height;
        pose1.orientation.w = 1.0;

        pose2.position.x = 0;
        pose2.position.y = gap_y_width / 2 + wall_length / 2;
        pose2.position.z = gap_height;
        pose2.orientation.w = 1.0;

        pose3.position.x = -gap_x_width / 2 - wall_length / 2;
        pose3.position.y = 0;
        pose3.position.z = gap_height;
        pose3.orientation.w = 1.0;

        pose4.position.x = 0;
        pose4.position.y = -gap_y_width / 2 - wall_length / 2;
        pose4.position.z = gap_height;
        pose4.orientation.w = 1.0;


        shape_msgs::SolidPrimitive primitive1, primitive2, primitive3, primitive4;
        primitive1.type = primitive1.BOX;
        primitive1.dimensions.resize(3);

        primitive1.dimensions[0] = wall_length;
        primitive1.dimensions[1] = wall_length;
        primitive1.dimensions[2] = 0.05;
        collision_object.primitives.push_back(primitive1);
        collision_object.primitive_poses.push_back(pose1);
        primitive2.type = primitive2.BOX;
        primitive2.dimensions.resize(3);
        primitive2.dimensions[0] = wall_length;
        primitive2.dimensions[1] = wall_length;
        primitive2.dimensions[2] = 0.05;
        collision_object.primitives.push_back(primitive2);
        collision_object.primitive_poses.push_back(pose2);
        primitive3.type = primitive2.BOX;
        primitive3.dimensions.resize(3);
        primitive3.dimensions[0] = wall_length;
        primitive3.dimensions[1] = wall_length;
        primitive3.dimensions[2] = 0.05;
        collision_object.primitives.push_back(primitive3);
        collision_object.primitive_poses.push_back(pose3);
        primitive4.type = primitive2.BOX;
        primitive4.dimensions.resize(3);
        primitive4.dimensions[0] = wall_length;
        primitive4.dimensions[1] = wall_length;
        primitive4.dimensions[2] = 0.05;
        collision_object.primitives.push_back(primitive4);
        collision_object.primitive_poses.push_back(pose4);
      }

    collision_object.operation = collision_object.ADD;
    planning_scene_->processCollisionObjectMsg(collision_object);


    /* kinemaitcs */
    sensor_msgs::JointState joint_state;
    /* special for the dragon kinematics */
    for(int i = 0; i < joint_num_ / 2; i++)
      {
        std::stringstream ss;
        ss << i + 1;
        joint_state.name.push_back(std::string("joint") + ss.str() + std::string("_pitch"));
        joint_state.position.push_back(start_state_[7 + 2 * i]);
        joint_state.name.push_back(std::string("joint") + ss.str() + std::string("_yaw"));
        joint_state.position.push_back(start_state_[7 + 2 * i + 1]);
      }
    /* gimbal */
    for(int j = 0; j <= joint_num_ / 2; j++)
      {
        std::stringstream ss;
        ss << j + 1;
        joint_state.name.push_back(std::string("gimbal") + ss.str() + std::string("_roll"));
        joint_state.position.push_back(0);
        joint_state.name.push_back(std::string("gimbal") + ss.str() + std::string("_pitch"));
        joint_state.position.push_back(0);
      }
    /* need to set the cog coodrinate for dragon */
    transform_controller_->setCogDesireOrientation(KDL::Rotation::Quaternion(start_state_[3],
                                                                             start_state_[4],
                                                                             start_state_[5],
                                                                             start_state_[6]));
    transform_controller_->kinematics(joint_state);
    std::vector<double> gimbal_nominal_angles = transform_controller_->getGimbalNominalAngles();
    for(auto itr = gimbal_nominal_angles.begin(); itr != gimbal_nominal_angles.end(); ++itr)
      start_state_.push_back(*itr); // roll -> pitch -> roll ->pitch

    robot_state::RobotState robot_state = setRobotState2Moveit(start_state_);

    planning_scene_->getPlanningSceneMsg(planning_scene_msg_);
    planning_scene_msg_.is_diff = true;
    planning_scene_diff_pub_.publish(planning_scene_msg_);

    //check collision with init state
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    planning_scene_->checkCollision(collision_request, collision_result, robot_state, acm_);

    if(collision_result.collision) ROS_WARN("collsion with init state");
  }

  void MotionPlanning::Planning()
  {
    //planning
    //x, y
    ompl::base::StateSpacePtr r_base;
    if(locomotion_mode_ == gap_passing::PlanningMode::SE2)
      {
        r_base = std::shared_ptr<ompl::base::SE2StateSpace>(new ompl::base::SE2StateSpace());

        ompl::base::RealVectorBounds motion_bounds(2);
        motion_bounds.low[0] = x_low_bound_;
        motion_bounds.low[1] = y_low_bound_;
        motion_bounds.high[0] = x_high_bound_;
        motion_bounds.high[1] = y_high_bound_;
        r_base->as<ompl::base::SE2StateSpace>()->setBounds(motion_bounds);
      }

    if(locomotion_mode_ == gap_passing::PlanningMode::SE3)
      {
        r_base = std::shared_ptr<ompl::base::SE3StateSpace>(new ompl::base::SE3StateSpace());

        ompl::base::RealVectorBounds motion_bounds(3);
        motion_bounds.low[0] = x_low_bound_;
        motion_bounds.low[1] = y_low_bound_;
        motion_bounds.low[2] = z_low_bound_;
        motion_bounds.high[0] = x_high_bound_;
        motion_bounds.high[1] = y_high_bound_;
        motion_bounds.high[2] = z_high_bound_;
        r_base->as<ompl::base::SE3StateSpace>()->setBounds(motion_bounds);
      }

    //joints
    ompl::base::StateSpacePtr r_joints(new ompl::base::RealVectorStateSpace(joint_num_));
    ompl::base::RealVectorBounds joint_bounds(joint_num_);
    /* special for dragon */
    for(int j = 0; j < joint_num_ / 2; j++ )
      {
        joint_bounds.low[2 * j] = pitch_joint_low_bound_;
        joint_bounds.high[2 * j] = pitch_joint_high_bound_;
        joint_bounds.low[2 * j + 1] = yaw_joint_low_bound_;
        joint_bounds.high[2 * j + 1] = yaw_joint_high_bound_;

        // joint_bounds.setLow(-1.58);
        // joint_bounds.setHigh(1.58);
      }
    r_joints->as<ompl::base::RealVectorStateSpace>()->setBounds(joint_bounds);

    if(planning_mode_ == gap_passing::PlanningMode::ONLY_JOINTS_MODE)
      {
        config_space_ = r_joints;
        config_space_->as<ompl::base::RealVectorStateSpace>()->setValidSegmentCountFactor(valid_segment_count_factor_);
        space_information_  = ompl::base::SpaceInformationPtr(new ompl::base::SpaceInformation(config_space_));
        space_information_->setup();
        space_information_->setStateValidityChecker(boost::bind(&MotionPlanning::isStateValid, this, _1));
        space_information_->setStateValidityCheckingResolution(state_validity_check_res_); 
        space_information_->setMotionValidator(ompl::base::MotionValidatorPtr(new ompl::base::DiscreteMotionValidator(space_information_)));
        space_information_->setup();

      }
    else if(planning_mode_ == gap_passing::PlanningMode::ONLY_BASE_MODE)
      {
        config_space_ = r_base;
        config_space_->as<ompl::base::SE3StateSpace>()->setValidSegmentCountFactor(valid_segment_count_factor_);
        space_information_  = ompl::base::SpaceInformationPtr(new ompl::base::SpaceInformation(config_space_));
        space_information_->setup();
        space_information_->setStateValidityChecker(boost::bind(&MotionPlanning::isStateValid, this, _1));
        space_information_->setStateValidityCheckingResolution(state_validity_check_res_);
        space_information_->setMotionValidator(ompl::base::MotionValidatorPtr(new ompl::base::DiscreteMotionValidator(space_information_)));
        space_information_->setup();
      }
    else if(planning_mode_ == gap_passing::PlanningMode::JOINTS_AND_BASE_MODE)
      {
        config_space_ = r_base + r_joints;
        //config_space_->as<ompl::base::CompoundStateSpace>()->setSubspaceWeight(1, 0.1);
        space_information_  = ompl::base::SpaceInformationPtr(new ompl::base::SpaceInformation(config_space_));
        space_information_->setStateValidityChecker(boost::bind(&MotionPlanning::isStateValid, this, _1));
      }

    /* init state */
    ompl::base::ScopedState<> start(config_space_);
    ompl::base::ScopedState<> goal(config_space_);
    if(planning_mode_ == gap_passing::PlanningMode::ONLY_JOINTS_MODE)
      {
        for(int i = 0; i < joint_num_; i ++)
          {
            start->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = start_state_[7 + i];
            goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = goal_state_[7 + i];
          }
      }
    else if(planning_mode_ == gap_passing::PlanningMode::ONLY_BASE_MODE)
      {
        if(locomotion_mode_ == gap_passing::PlanningMode::SE2)
          {
            start->as<ompl::base::SE2StateSpace::StateType>()->setXY(start_state_[0], start_state_[1]);
            start->as<ompl::base::SE2StateSpace::StateType>()->setYaw(tf::getYaw(tf::Quaternion(start_state_[3], start_state_[4], start_state_[5], start_state_[6])));
            goal->as<ompl::base::SE2StateSpace::StateType>()->setXY(goal_state_[0], goal_state_[1]);
            goal->as<ompl::base::SE2StateSpace::StateType>()->setYaw(tf::getYaw(tf::Quaternion(start_state_[3], start_state_[4], start_state_[5], start_state_[6])));
          }
        if(locomotion_mode_ == gap_passing::PlanningMode::SE3)
          {
            start->as<ompl::base::SE3StateSpace::StateType>()->setXYZ(start_state_[0], start_state_[1], start_state_[2]);
            start->as<ompl::base::SE3StateSpace::StateType>()->rotation().x = start_state_[3];
            start->as<ompl::base::SE3StateSpace::StateType>()->rotation().y = start_state_[4];
            start->as<ompl::base::SE3StateSpace::StateType>()->rotation().z = start_state_[5];
            start->as<ompl::base::SE3StateSpace::StateType>()->rotation().w = start_state_[6];

            goal->as<ompl::base::SE3StateSpace::StateType>()->setXYZ(goal_state_[0], goal_state_[1],goal_state_[2]);
            goal->as<ompl::base::SE3StateSpace::StateType>()->rotation().x = goal_state_[3];
            goal->as<ompl::base::SE3StateSpace::StateType>()->rotation().y = goal_state_[4];
            goal->as<ompl::base::SE3StateSpace::StateType>()->rotation().z = goal_state_[5];
            goal->as<ompl::base::SE3StateSpace::StateType>()->rotation().w = goal_state_[6];
          }
      }

    else if(planning_mode_ == gap_passing::PlanningMode::JOINTS_AND_BASE_MODE)
      {
        ompl::base::CompoundState* start_tmp = dynamic_cast<ompl::base::CompoundState*> (start.get());
        ompl::base::CompoundState* goal_tmp = dynamic_cast<ompl::base::CompoundState*> (goal.get());

        if(locomotion_mode_ == gap_passing::PlanningMode::SE2)
          {
            start_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->setXY(start_state_[0], start_state_[1]);
            start_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->setYaw(tf::getYaw(tf::Quaternion(start_state_[3], start_state_[4], start_state_[5], start_state_[6])));
            goal_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->setXY(goal_state_[0], goal_state_[1]);
            goal_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->setYaw(tf::getYaw(tf::Quaternion(goal_state_[3], goal_state_[4], goal_state_[5], goal_state_[6])));
          }

        if(locomotion_mode_ == gap_passing::PlanningMode::SE3)
          {
            start_tmp->as<ompl::base::SE3StateSpace::StateType>(0)->setXYZ(start_state_[0], start_state_[1], start_state_[2]);
            start_tmp->as<ompl::base::SE3StateSpace::StateType>(0)->rotation().x = start_state_[3];
            start_tmp->as<ompl::base::SE3StateSpace::StateType>(0)->rotation().y = start_state_[4];
            start_tmp->as<ompl::base::SE3StateSpace::StateType>(0)->rotation().z = start_state_[5];
            start_tmp->as<ompl::base::SE3StateSpace::StateType>(0)->rotation().w = start_state_[6];

            goal_tmp->as<ompl::base::SE3StateSpace::StateType>(0)->setXYZ(goal_state_[0], goal_state_[1],goal_state_[2]);
            goal_tmp->as<ompl::base::SE3StateSpace::StateType>(0)->rotation().x = goal_state_[3];
            goal_tmp->as<ompl::base::SE3StateSpace::StateType>(0)->rotation().y = goal_state_[4];
            goal_tmp->as<ompl::base::SE3StateSpace::StateType>(0)->rotation().z = goal_state_[5];
            goal_tmp->as<ompl::base::SE3StateSpace::StateType>(0)->rotation().w = goal_state_[6];
          }

        for(int i = 0; i < joint_num_; i ++)
          {
            start_tmp->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[i] = start_state_[7 + i];
            goal_tmp->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[i] = goal_state_[7 + i];
          }

      }

    ompl::base::ProblemDefinitionPtr pdef(new ompl::base::ProblemDefinition(space_information_));
    pdef->setStartAndGoalStates(start, goal);

    //sampler
    space_information_->setValidStateSamplerAllocator(boost::bind(&MotionPlanning::allocValidStateSampler, this, _1));

    //optimation objective
    path_length_opt_objective_ = new ompl::base::PathLengthOptimizationObjective(space_information_);
    path_length_opt_objective_->setCostThreshold(onlyJointPathLimit());
    std::cout << "path length opt cost thre is "<<  path_length_opt_objective_->getCostThreshold() << std::endl;
    ompl::base::OptimizationObjectivePtr lengthObj(path_length_opt_objective_);

    ompl::base::PlannerPtr planner;
    if(ompl_mode_ == RRT_START_MODE)
      {
        pdef->setOptimizationObjective(lengthObj);
        rrt_start_planner_ = new ompl::geometric::RRTstar(space_information_);
        planner = ompl::base::PlannerPtr(rrt_start_planner_);
      }

    planner->setProblemDefinition(pdef);
    planner->setup();
    space_information_->printSettings(std::cout);

    solved_ = false;
    ros::Time start_time = ros::Time::now();

    ompl::base::PlannerStatus solved = planner->solve(solving_time_limit_);

    if (solved)
      {
        calculation_time_ = ros::Time::now().toSec() - start_time.toSec();
        path_ = pdef->getSolutionPath();
        std::cout << "Found solution:" << std::endl;
        path_->print(std::cout);

        if(ompl_mode_ == RRT_START_MODE)
          {
            std::cout << "iteration is "<< rrt_start_planner_->numIterations() << "best cost is " << rrt_start_planner_->bestCost()  << std::endl;
            std::stringstream ss;
            ss << rrt_start_planner_->bestCost();
            ss >> best_cost_;
          }

        //visualztion
        plan_states_ = new ompl::base::StateStorage(config_space_);
        int index = (int)(std::static_pointer_cast<ompl::geometric::PathGeometric>(path_)->getStateCount());
        ROS_ERROR("index: %d", index);
        for(int i = 0; i < index; i++)
          {
            ompl::base::State *state1;
            ompl::base::State *state2;

            if(i == 0)
              state1 = start.get();
            else
              state1 = std::static_pointer_cast<ompl::geometric::PathGeometric>(path_)->getState(i - 1);

            state2 = std::static_pointer_cast<ompl::geometric::PathGeometric>(path_)->getState(i);


            plan_states_->addState(state1);
            int nd = config_space_->validSegmentCount(state1, state2);
            if (nd > 1)
              {
                ompl::base::State *interpolated_state = space_information_->allocState();
                for (int j = 1 ; j < nd ; ++j)
                  {
                    config_space_->interpolate(state1, state2, (double)j / (double)nd, interpolated_state);
                    plan_states_->addState(interpolated_state);

                  }
              }
            plan_states_->addState(state2);
          }

        motion_control_->planStoring(plan_states_, planning_mode_, locomotion_mode_, start_state_, goal_state_, best_cost_, calculation_time_);

        solved_ = true;
      }
    else
      std::cout << "No solution found" << std::endl;
  }

  void MotionPlanning::rosParamInit()
  {
    nhp_.param("gap_type", gap_type_, 0); //Type0: vertical; Type1: horizontal

    //nhp_.param("coefficient_rate", coefficient_rate_, 0.035);
    nhp_.param("ompl_mode", ompl_mode_, 0); //RRT_START_MODE
    nhp_.param("planning_mode", planning_mode_, 1); //ONLY_BASE_MODE
    nhp_.param("locomotion_mode", locomotion_mode_, 0); //SE2
    nhp_.param("motion_sequence_rate", motion_sequence_rate_, 10.0);

    /* x, y, z, joint bound */
    nhp_.param("x_low_bound", x_low_bound_, -2.0);
    nhp_.param("x_high_bound", x_high_bound_, 5.0);
    nhp_.param("y_low_bound", y_low_bound_, -2.2);
    nhp_.param("y_high_bound", y_high_bound_, 2.2);
    nhp_.param("z_low_bound", z_low_bound_, -2.2);
    nhp_.param("z_high_bound", z_high_bound_, 2.2);
    nhp_.param("pitch_joint_low_bound", pitch_joint_low_bound_, -1.58);
    nhp_.param("pitch_joint_high_bound", pitch_joint_high_bound_, 1.58);
    nhp_.param("yaw_joint_low_bound", yaw_joint_low_bound_, -1.58);
    nhp_.param("yaw_joint_high_bound", yaw_joint_high_bound_, 1.58);

    start_state_.resize(7 + joint_num_);
    nhp_.param("start_state_x", start_state_[0], 0.0);
    nhp_.param("start_state_y", start_state_[1], 0.5);
    nhp_.param("start_state_z", start_state_[2], 0.0);
    double r, p, y;
    nhp_.param("start_state_roll", r, 0.0);
    nhp_.param("start_state_pitch", p, 0.0);
    nhp_.param("start_state_yaw", y, 0.0);
    tf::Quaternion q; q.setRPY(r, p, y);
    start_state_[3] = q.x();
    start_state_[4] = q.y();
    start_state_[5] = q.z();
    start_state_[6] = q.w();

    goal_state_.resize(7 + joint_num_);
    nhp_.param("goal_state_x", goal_state_[0], 3.0);
    nhp_.param("goal_state_y", goal_state_[1], 0.5);
    nhp_.param("goal_state_z", goal_state_[2], 0.0);
    nhp_.param("goal_state_roll", r, 0.0);
    nhp_.param("goal_state_pitch", p, 0.0);
    nhp_.param("goal_state_yaw", y, 0.0);
    q.setRPY(r, p, y);
    goal_state_[3] = q.x();
    goal_state_[4] = q.y();
    goal_state_[5] = q.z();
    goal_state_[6] = q.w();

    for(int i = 0; i < joint_num_; i++)
      {
        std::stringstream joint_no;
        joint_no << i + 1;
        nhp_.param(std::string("start_state_joint") + joint_no.str(), start_state_[7 + i], 0.0);
        nhp_.param(std::string("goal_state_joint") + joint_no.str(), goal_state_[7 + i], 0.0);
      }

    nhp_.param("state_validity_check_res", state_validity_check_res_, 0.03);
    nhp_.param("valid_segment_count_factor", valid_segment_count_factor_,20);

    nhp_.param("solving_time_limit", solving_time_limit_, 3600.0);

    nhp_.param("length_opt_weight", length_opt_weight_, 1.0);
    nhp_.param("stability_opt_weight", stability_opt_weight_, 0.0);

    nhp_.param("stability_cost_thre", stability_cost_thre_, 100000000.0);
    nhp_.param("length_cost_thre", length_cost_thre_, 0.0);

    nhp_.param("play_log_path", play_log_path_, false);
    if(play_log_path_) solved_ = true;
  }

  ompl::base::Cost MotionPlanning::onlyJointPathLimit()
  {
    if(planning_mode_ == gap_passing::PlanningMode::ONLY_JOINTS_MODE ||
       planning_mode_ == gap_passing::PlanningMode::JOINTS_AND_BASE_MODE)
      {
        if(planning_mode_ == gap_passing::PlanningMode::ONLY_JOINTS_MODE) length_cost_thre_ = 0.1;
        for(int i = 0; i < joint_num_; i ++)
          length_cost_thre_ += fabs(start_state_[7 + i] - goal_state_[7 + i]);
      }

    return ompl::base::Cost(length_cost_thre_);
  }

  robot_state::RobotState MotionPlanning::setRobotState2Moveit(std::vector<double> state)
  {
    // if(state.size() !=7 + joint_num_ + joint_num_ + 2)
    //   ROS_INFO("state size: %d", state.size());
    assert(state.size() == 7 + joint_num_ + joint_num_ + 2);

    robot_state::RobotState& robot_state = planning_scene_->getCurrentStateNonConst();
    robot_state.setVariablePosition(std::string("base/trans_x"), state[0]);
    robot_state.setVariablePosition(std::string("base/trans_y"), state[1]);
    robot_state.setVariablePosition(std::string("base/trans_z"), state[2]);
    robot_state.setVariablePosition(std::string("base/rot_x"),   state[3]);
    robot_state.setVariablePosition(std::string("base/rot_y"),   state[4]);
    robot_state.setVariablePosition(std::string("base/rot_z"),   state[5]);
    robot_state.setVariablePosition(std::string("base/rot_w"),   state[6]);
    for(int i = 0; i < joint_num_ / 2; i++)
      {
        std::stringstream ss;
        ss << i + 1;
        robot_state.setVariablePosition(std::string("joint") + ss.str() + std::string("_pitch"),
                                        state[7 + 2 * i]);
        robot_state.setVariablePosition(std::string("joint") + ss.str() + std::string("_yaw"),
                                        state[7 + 2 * i + 1]);
      }

    for(int i = 0; i <= joint_num_ / 2; i++)
      {
        std::stringstream ss;
        ss << i + 1;
        robot_state.setVariablePosition(std::string("gimbal") + ss.str() + std::string("_roll"),
                                        state[7 + joint_num_ + 2 * i]);
        robot_state.setVariablePosition(std::string("gimbal") + ss.str() + std::string("_pitch"),
                                        state[7 + joint_num_ + 2 * i + 1]);
      }

    return robot_state;
  }

}
