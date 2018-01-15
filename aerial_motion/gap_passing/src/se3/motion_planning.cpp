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


#include <gap_passing/se3/motion_planning.h>

/* the StabilityObjective does not work!!!!! */
namespace se3
{
  MotionPlanning::MotionPlanning(ros::NodeHandle nh, ros::NodeHandle nhp):
    se2::MotionPlanning(nh, nhp), max_force_(0), max_force_state_(0)
  {
  }

  void MotionPlanning::robotInit()
  {
    root_motion_dof_ = 6;
    transform_controller_ = boost::shared_ptr<DragonTransformController>(new DragonTransformController(nh_, nhp_, false));
    joint_num_ = (transform_controller_->getRotorNum() - 1) * 2 ;

    start_state_.joint_states.resize(2 * joint_num_ + 2, 0);
    goal_state_.joint_states.resize(2 * joint_num_ + 2, 0);

    for(int i = 0; i < joint_num_ / 2; i++)
      {
        std::stringstream ss;
        ss << i + 1;
        start_state_.joint_names.push_back(std::string("joint") + ss.str() + std::string("_pitch"));
        start_state_.joint_names.push_back(std::string("joint") + ss.str() + std::string("_yaw"));
      }
    /* gimbal */
    for(int j = 0; j <= joint_num_ / 2; j++)
      {
        std::stringstream ss;
        ss << j + 1;
        start_state_.joint_names.push_back(std::string("gimbal") + ss.str() + std::string("_roll"));
        start_state_.joint_names.push_back(std::string("gimbal") + ss.str() + std::string("_pitch"));
      }
  }

  bool MotionPlanning::isStateValid(const ompl::base::State *state)
  {
    State current_state = start_state_;

    if(planning_mode_ == gap_passing::PlanningMode::ONLY_JOINTS_MODE)
      {
        for(int i = 0; i < joint_num_; i++)
          current_state.joint_states.at(i) = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i];
      }
    else if(planning_mode_ == gap_passing::PlanningMode::ONLY_BASE_MODE)
      {
        if(locomotion_mode_ == gap_passing::PlanningMode::SE2)
          {
            current_state.root_state.at(0) = state->as<ompl::base::SE2StateSpace::StateType>()->getX();
            current_state.root_state.at(1) = state->as<ompl::base::SE2StateSpace::StateType>()->getY();
            tf::Quaternion q = tf::createQuaternionFromYaw(state->as<ompl::base::SE2StateSpace::StateType>()->getYaw());
            current_state.root_state.at(3) = q.x();
            current_state.root_state.at(4) = q.y();
            current_state.root_state.at(5) = q.z();
            current_state.root_state.at(6) = q.w();
          }

        if(locomotion_mode_ == gap_passing::PlanningMode::SE3)
          {
            current_state.root_state.at(0) = state->as<ompl::base::SE3StateSpace::StateType>()->getX();
            current_state.root_state.at(1) = state->as<ompl::base::SE3StateSpace::StateType>()->getY();
            current_state.root_state.at(2) = state->as<ompl::base::SE3StateSpace::StateType>()->getZ();
            current_state.root_state.at(3) = state->as<ompl::base::SE3StateSpace::StateType>()->rotation().x;
            current_state.root_state.at(4) = state->as<ompl::base::SE3StateSpace::StateType>()->rotation().y;
            current_state.root_state.at(5) = state->as<ompl::base::SE3StateSpace::StateType>()->rotation().z;
            current_state.root_state.at(6) = state->as<ompl::base::SE3StateSpace::StateType>()->rotation().w;
          }
      }
    else if(planning_mode_ == gap_passing::PlanningMode::JOINTS_AND_BASE_MODE)
      {
        const ompl::base::CompoundState* state_tmp = dynamic_cast<const ompl::base::CompoundState*>(state);
        if(locomotion_mode_ == gap_passing::PlanningMode::SE2)
          {
            current_state.root_state.at(0) = state_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->getX();
            current_state.root_state.at(1) = state_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->getY();
            tf::Quaternion q = tf::createQuaternionFromYaw(state_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->getYaw());
            current_state.root_state.at(3) = q.x();
            current_state.root_state.at(4) = q.y();
            current_state.root_state.at(5) = q.z();
            current_state.root_state.at(6) = q.w();
          }
        if(locomotion_mode_ == gap_passing::PlanningMode::SE3)
          {
            current_state.root_state.at(0) = state_tmp->as<ompl::base::SE3StateSpace::StateType>(0)->getX();
            current_state.root_state.at(1) = state_tmp->as<ompl::base::SE3StateSpace::StateType>(0)->getY();
            current_state.root_state.at(2) = state_tmp->as<ompl::base::SE3StateSpace::StateType>(0)->getZ();
            current_state.root_state.at(3) = state_tmp->as<ompl::base::SE3StateSpace::StateType>(0)->rotation().x;
            current_state.root_state.at(4) = state_tmp->as<ompl::base::SE3StateSpace::StateType>(0)->rotation().y;
            current_state.root_state.at(5) = state_tmp->as<ompl::base::SE3StateSpace::StateType>(0)->rotation().z;
            current_state.root_state.at(6) = state_tmp->as<ompl::base::SE3StateSpace::StateType>(0)->rotation().w;
          }
        for(int i = 0; i < joint_num_ ; i++)
          current_state.joint_states.at(i) = state_tmp->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[i];
      }

    /* special for the dragon kinematics */
    sensor_msgs::JointState joint_state;
    joint_state.name = current_state.joint_names;
    joint_state.position = current_state.joint_states;
    /* consider rootlink (link1) as baselink */
    transform_controller_->setCogDesireOrientation(KDL::Rotation::Quaternion(current_state.root_state.at(3),
                                                                             current_state.root_state.at(4),
                                                                             current_state.root_state.at(5),
                                                                             current_state.root_state.at(6)));

    transform_controller_->kinematics(joint_state);
    double dist_thre_check = transform_controller_->distThreCheck();

    if(dist_thre_check == 0) return false;
    if(!transform_controller_->overlapCheck()) return false;
    if(!transform_controller_->modelling()) return false;

    //set the nominal gimbal angles for moveit collision check
    std::vector<double> gimbal_nominal_angles = transform_controller_->getGimbalNominalAngles();
    for(int i = 0; i < gimbal_nominal_angles.size(); i++)
      {
        current_state.joint_states.at(joint_num_ + i) = gimbal_nominal_angles.at(i); // roll -> pitch -> roll ->pitch

        /*  the gimbal tilt angle */
        /* roll */
        if (i / 2 == 0 && fabs(gimbal_nominal_angles.at(i)) > gimbal_roll_thresh_) return false;
        /* pitch */
        if (i / 2 == 1 && fabs(gimbal_nominal_angles.at(i)) > gimbal_pitch_thresh_) return false;
      }

    //check collision
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;

    robot_state::RobotState robot_state = setRobotState2Moveit(current_state, planning_scene_);
    planning_scene_->checkCollision(collision_request, collision_result, robot_state, acm_);

    if(collision_result.collision) return false;

    return true;
  }

  void MotionPlanning::gapEnvInit()
  {
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = "world";
    collision_object.id = "box";

    geometry_msgs::Pose wall_pose;
    wall_pose.orientation.w = 1.0;
    wall_pose.position.z = 0.0;
    shape_msgs::SolidPrimitive wall_primitive;
    wall_primitive.type = wall_primitive.BOX;
    wall_primitive.dimensions.resize(3);

    /* gap type 1: vertical gap */
    if(gap_type_ == HORIZONTAL_GAP)
      {
        double gap_left_x;
        double gap_x_offset;
        double gap_y_offset;
        double gap_left_width;
        double gap_right_width;

        nhp_.param("gap_left_x", gap_left_x, 0.0);
        nhp_.param("gap_y_offset", gap_y_offset, 0.6); //minus: overlap
        nhp_.param("gap_left_width", gap_left_width, 0.3); //minus: bandwidth
        nhp_.param("gap_right_width", gap_right_width, 0.3); //minus: bandwidth

        wall_primitive.dimensions[2] = 10;

        wall_pose.position.x = gap_left_x + gap_left_width /2;
        wall_pose.position.y =  (2.5 + gap_y_offset) /2;
        wall_primitive.dimensions[0] = gap_left_width;
        wall_primitive.dimensions[1] = 2.5;
        collision_object.primitives.push_back(wall_primitive);
        collision_object.primitive_poses.push_back(wall_pose);

        wall_pose.position.x = gap_left_x + gap_right_width /2;
        wall_pose.position.y = - (2.5 + gap_y_offset) /2;
        collision_object.primitives.push_back(wall_primitive);
        collision_object.primitive_poses.push_back(wall_pose);

        wall_pose.position.x = 1.0;
        wall_pose.position.y = 2.5;
        wall_primitive.dimensions[0] = 8;
        wall_primitive.dimensions[1] = 0.6;
        collision_object.primitives.push_back(wall_primitive);
        collision_object.primitive_poses.push_back(wall_pose);

        wall_pose.position.y = -2.5;
        collision_object.primitives.push_back(wall_primitive);
        collision_object.primitive_poses.push_back(wall_pose);
      }

    if(gap_type_ == VERTICAL_GAP)
      {
        double gap_x_width, gap_y_width, gap_height;
        double wall_length = 10;

        nhp_.param("gap_x_width", gap_x_width, 1.0);
        nhp_.param("gap_y_width", gap_y_width, 1.0);
        nhp_.param("gap_height", gap_height, 0.3);

        /* gap */
        wall_pose.position.x = gap_x_width / 2 + wall_length / 2;
        wall_pose.position.y = 0;
        wall_pose.position.z = gap_height;
        wall_primitive.dimensions[0] = wall_length;
        wall_primitive.dimensions[1] = wall_length;
        wall_primitive.dimensions[2] = 0.05;
        collision_object.primitives.push_back(wall_primitive);
        collision_object.primitive_poses.push_back(wall_pose);

        wall_pose.position.x = 0;
        wall_pose.position.y = gap_y_width / 2 + wall_length / 2;
        collision_object.primitives.push_back(wall_primitive);
        collision_object.primitive_poses.push_back(wall_pose);

        wall_pose.position.x = -gap_x_width / 2 - wall_length / 2;
        wall_pose.position.y = 0;
        collision_object.primitives.push_back(wall_primitive);
        collision_object.primitive_poses.push_back(wall_pose);

        wall_pose.position.x = 0;
        wall_pose.position.y = -gap_y_width / 2 - wall_length / 2;
        collision_object.primitives.push_back(wall_primitive);
        collision_object.primitive_poses.push_back(wall_pose);

        /* ceil */
        wall_pose.position.x = 0;
        wall_pose.position.y = 0;
        wall_pose.position.z = z_high_bound_;
        wall_primitive.dimensions[0] = wall_length;
        wall_primitive.dimensions[1] = wall_length;
        wall_primitive.dimensions[2] = 0.05;
        collision_object.primitives.push_back(wall_primitive);
        collision_object.primitive_poses.push_back(wall_pose);

        /* ground */
        wall_pose.position.z = 0;
        collision_object.primitives.push_back(wall_primitive);
        collision_object.primitive_poses.push_back(wall_pose);

        /* side wall */
        wall_pose.position.x = x_high_bound_;
        wall_pose.position.y = 0;
        //wall_pose.position.z = gap_height;
        wall_pose.position.z = gap_height/ 2;
        wall_primitive.dimensions[0] = 0.05;
        wall_primitive.dimensions[1] = y_high_bound_ - y_low_bound_;
        //wall_primitive.dimensions[2] = 2 * gap_height;
        wall_primitive.dimensions[2] = gap_height;
        collision_object.primitives.push_back(wall_primitive);
        collision_object.primitive_poses.push_back(wall_pose);

        wall_pose.position.x = x_low_bound_;
        collision_object.primitives.push_back(wall_primitive);
        collision_object.primitive_poses.push_back(wall_pose);

        wall_pose.position.x = 0;
        wall_pose.position.y = y_high_bound_;
        wall_primitive.dimensions[0] = x_high_bound_ - x_low_bound_;
        wall_primitive.dimensions[1] = 0.05;
        collision_object.primitives.push_back(wall_primitive);
        collision_object.primitive_poses.push_back(wall_pose);

        wall_pose.position.x = 0;
        wall_pose.position.y = y_low_bound_;
        collision_object.primitives.push_back(wall_primitive);
        collision_object.primitive_poses.push_back(wall_pose);
      }

    collision_object.operation = collision_object.ADD;
    planning_scene_->processCollisionObjectMsg(collision_object);

    /* special for the dragon kinematics */
    transform_controller_->setBaselink(std::string("link1"));
    sensor_msgs::JointState joint_state;
    joint_state.name = start_state_.joint_names;
    joint_state.position = start_state_.joint_states;
    /* need to set the cog coodrinate for dragon */
    transform_controller_->setCogDesireOrientation(KDL::Rotation::Quaternion(start_state_.root_state.at(3),
                                                                             start_state_.root_state.at(4),
                                                                             start_state_.root_state.at(5),
                                                                             start_state_.root_state.at(6)));

    transform_controller_->kinematics(joint_state);
    //set the nominal gimbal angles for moveit collision check
    std::vector<double> gimbal_nominal_angles = transform_controller_->getGimbalNominalAngles();
    for(int i = 0; i < gimbal_nominal_angles.size(); i++)
      start_state_.joint_states.at(joint_num_ + i) = gimbal_nominal_angles.at(i); // roll -> pitch -> roll ->pitch
    transform_controller_->setBaselink(base_link_);
  }

  void MotionPlanning::planInit()
  {
    //set root link as the baselink for the planning
    transform_controller_->setBaselink(std::string("link1"));

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
        config_space_->as<ompl::base::CompoundStateSpace>()->setSubspaceWeight(1, 0.001);
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
            start->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = start_state_.joint_states.at(i);
            goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = goal_state_.joint_states.at(i);
          }
      }
    else if(planning_mode_ == gap_passing::PlanningMode::ONLY_BASE_MODE)
      {
        if(locomotion_mode_ == gap_passing::PlanningMode::SE2)
          {
            start->as<ompl::base::SE2StateSpace::StateType>()->setXY(start_state_.root_state.at(0), start_state_.root_state.at(1));
            start->as<ompl::base::SE2StateSpace::StateType>()->setYaw(tf::getYaw(tf::Quaternion(start_state_.root_state.at(3), start_state_.root_state.at(4), start_state_.root_state.at(5), start_state_.root_state.at(6))));
            goal->as<ompl::base::SE2StateSpace::StateType>()->setXY(goal_state_.root_state.at(0), goal_state_.root_state.at(1));
            goal->as<ompl::base::SE2StateSpace::StateType>()->setYaw(tf::getYaw(tf::Quaternion(goal_state_.root_state.at(3), goal_state_.root_state.at(4), goal_state_.root_state.at(5), goal_state_.root_state.at(6))));
          }
        if(locomotion_mode_ == gap_passing::PlanningMode::SE3)
          {
            start->as<ompl::base::SE3StateSpace::StateType>()->setXYZ(start_state_.root_state.at(0), start_state_.root_state.at(1), start_state_.root_state.at(2));
            start->as<ompl::base::SE3StateSpace::StateType>()->rotation().x = start_state_.root_state.at(3);
            start->as<ompl::base::SE3StateSpace::StateType>()->rotation().y = start_state_.root_state.at(4);
            start->as<ompl::base::SE3StateSpace::StateType>()->rotation().z = start_state_.root_state.at(5);
            start->as<ompl::base::SE3StateSpace::StateType>()->rotation().w = start_state_.root_state.at(6);
            goal->as<ompl::base::SE3StateSpace::StateType>()->setXYZ(goal_state_.root_state.at(0), goal_state_.root_state.at(1), goal_state_.root_state.at(2));
            goal->as<ompl::base::SE3StateSpace::StateType>()->rotation().x = goal_state_.root_state.at(3);
            goal->as<ompl::base::SE3StateSpace::StateType>()->rotation().y = goal_state_.root_state.at(4);
            goal->as<ompl::base::SE3StateSpace::StateType>()->rotation().z = goal_state_.root_state.at(5);
            goal->as<ompl::base::SE3StateSpace::StateType>()->rotation().w = goal_state_.root_state.at(6);
          }
      }
    else if(planning_mode_ == gap_passing::PlanningMode::JOINTS_AND_BASE_MODE)
      {
        ompl::base::CompoundState* start_tmp = dynamic_cast<ompl::base::CompoundState*> (start.get());
        ompl::base::CompoundState* goal_tmp = dynamic_cast<ompl::base::CompoundState*> (goal.get());

        if(locomotion_mode_ == gap_passing::PlanningMode::SE2)
          {
            start_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->setXY(start_state_.root_state.at(0), start_state_.root_state.at(1));
            start_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->setYaw(tf::getYaw(tf::Quaternion(start_state_.root_state.at(3), start_state_.root_state.at(4), start_state_.root_state.at(5), start_state_.root_state.at(6))));
            goal_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->setXY(goal_state_.root_state.at(0), goal_state_.root_state.at(1));
            goal_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->setYaw(tf::getYaw(tf::Quaternion(goal_state_.root_state.at(3), goal_state_.root_state.at(4), goal_state_.root_state.at(5), goal_state_.root_state.at(6))));
          }
        if(locomotion_mode_ == gap_passing::PlanningMode::SE3)
          {
            start_tmp->as<ompl::base::SE3StateSpace::StateType>(0)->setXYZ(start_state_.root_state.at(0), start_state_.root_state.at(1), start_state_.root_state.at(2));
            start_tmp->as<ompl::base::SE3StateSpace::StateType>(0)->rotation().x = start_state_.root_state.at(3);
            start_tmp->as<ompl::base::SE3StateSpace::StateType>(0)->rotation().y = start_state_.root_state.at(4);
            start_tmp->as<ompl::base::SE3StateSpace::StateType>(0)->rotation().z = start_state_.root_state.at(5);
            start_tmp->as<ompl::base::SE3StateSpace::StateType>(0)->rotation().w = start_state_.root_state.at(6);
            goal_tmp->as<ompl::base::SE3StateSpace::StateType>(0)->setXYZ(goal_state_.root_state.at(0), goal_state_.root_state.at(1), goal_state_.root_state.at(2));
            goal_tmp->as<ompl::base::SE3StateSpace::StateType>(0)->rotation().x = goal_state_.root_state.at(3);
            goal_tmp->as<ompl::base::SE3StateSpace::StateType>(0)->rotation().y = goal_state_.root_state.at(4);
            goal_tmp->as<ompl::base::SE3StateSpace::StateType>(0)->rotation().z = goal_state_.root_state.at(5);
            goal_tmp->as<ompl::base::SE3StateSpace::StateType>(0)->rotation().w = goal_state_.root_state.at(6);
          }
        for(int i = 0; i < joint_num_; i ++)
          {
            start_tmp->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[i] = start_state_.joint_states.at(i);
            goal_tmp->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[i] = goal_state_.joint_states.at(i);
          }
      }

    pdef_ = ompl::base::ProblemDefinitionPtr(new ompl::base::ProblemDefinition(space_information_));
    pdef_->setStartAndGoalStates(start, goal);
  }

  void MotionPlanning::rosParamInit()
  {
    se2::MotionPlanning::rosParamInit();

    nhp_.param("gap_type", gap_type_, 0); //Type0: vertical; Type1: horizontal
    nhp_.param("locomotion_mode", locomotion_mode_, 0);

    /* z, roll, pitch, joint bound */
    nhp_.param("z_low_bound", z_low_bound_, -2.2);
    nhp_.param("z_high_bound", z_high_bound_, 2.2);
    nhp_.param("pitch_joint_low_bound", pitch_joint_low_bound_, -1.58);
    nhp_.param("pitch_joint_high_bound", pitch_joint_high_bound_, 1.58);
    nhp_.param("yaw_joint_low_bound", yaw_joint_low_bound_, -1.58);
    nhp_.param("yaw_joint_high_bound", yaw_joint_high_bound_, 1.58);

    nhp_.param("start_state_z", start_state_.root_state.at(2), 0.0);
    double r, p, y;
    nhp_.param("start_state_roll", r, 0.0);
    nhp_.param("start_state_pitch", p, 0.0);
    nhp_.param("start_state_yaw", y, 0.0);
    start_state_.setRootRPY(r, p, y);

    nhp_.param("goal_state_z", goal_state_.root_state.at(2), 0.0);
    nhp_.param("goal_state_roll", r, 0.0);
    nhp_.param("goal_state_pitch", p, 0.0);
    nhp_.param("goal_state_yaw", y, 0.0);
    goal_state_.setRootRPY(r, p, y);

    nhp_.param("gimbal_roll_thresh", gimbal_roll_thresh_, 1.6);
    nhp_.param("gimbal_pitch_thresh", gimbal_pitch_thresh_, 1.1);
  }

  void MotionPlanning::addState(ompl::base::State *ompl_state)
  {
    State new_state = start_state_;

    if(planning_mode_ == gap_passing::PlanningMode::ONLY_BASE_MODE)
      {
        if(locomotion_mode_ == gap_passing::PlanningMode::SE2)
          {
            new_state.root_state.at(0) = ompl_state->as<ompl::base::SE2StateSpace::StateType>()->getX();
            new_state.root_state.at(1) = ompl_state->as<ompl::base::SE2StateSpace::StateType>()->getY();
            tf::Quaternion q = tf::createQuaternionFromYaw(ompl_state->as<ompl::base::SE2StateSpace::StateType>()->getYaw());
            new_state.root_state.at(3) = q.x();
            new_state.root_state.at(4) = q.y();
            new_state.root_state.at(5) = q.z();
            new_state.root_state.at(6) = q.w();
          }
        if(locomotion_mode_ == gap_passing::PlanningMode::SE3)
          {
            new_state.root_state.at(0) = ompl_state->as<ompl::base::SE3StateSpace::StateType>()->getX();
            new_state.root_state.at(1) = ompl_state->as<ompl::base::SE3StateSpace::StateType>()->getY();
            new_state.root_state.at(2) = ompl_state->as<ompl::base::SE3StateSpace::StateType>()->getZ();
            new_state.root_state.at(3) = ompl_state->as<ompl::base::SE3StateSpace::StateType>()->rotation().x;
            new_state.root_state.at(4) = ompl_state->as<ompl::base::SE3StateSpace::StateType>()->rotation().y;
            new_state.root_state.at(5) = ompl_state->as<ompl::base::SE3StateSpace::StateType>()->rotation().z;
            new_state.root_state.at(6) = ompl_state->as<ompl::base::SE3StateSpace::StateType>()->rotation().w;
          }
      }
    else if(planning_mode_ == gap_passing::PlanningMode::JOINTS_AND_BASE_MODE)
      {
        const ompl::base::CompoundState* state_tmp = dynamic_cast<const ompl::base::CompoundState*>(ompl_state);

        /* SE2 */
        if(locomotion_mode_ == gap_passing::PlanningMode::SE2)
          {
            new_state.root_state.at(0) = state_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->getX();
            new_state.root_state.at(1) = state_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->getY();
            tf::Quaternion q = tf::createQuaternionFromYaw(state_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->getYaw());
            new_state.root_state.at(3) = q.x();
            new_state.root_state.at(4) = q.y();
            new_state.root_state.at(5) = q.z();
            new_state.root_state.at(6) = q.w();
          }

        /* SE3 */
        if(locomotion_mode_ == gap_passing::PlanningMode::SE3)
          {
            new_state.root_state.at(0) = state_tmp->as<ompl::base::SE3StateSpace::StateType>(0)->getX();
            new_state.root_state.at(1) = state_tmp->as<ompl::base::SE3StateSpace::StateType>(0)->getY();
            new_state.root_state.at(2) = state_tmp->as<ompl::base::SE3StateSpace::StateType>(0)->getZ();
            new_state.root_state.at(3) = state_tmp->as<ompl::base::SE3StateSpace::StateType>(0)->rotation().x;
            new_state.root_state.at(4) = state_tmp->as<ompl::base::SE3StateSpace::StateType>(0)->rotation().y;
            new_state.root_state.at(5) = state_tmp->as<ompl::base::SE3StateSpace::StateType>(0)->rotation().z;
            new_state.root_state.at(6) = state_tmp->as<ompl::base::SE3StateSpace::StateType>(0)->rotation().w;
          }

        for(int j = 0; j < joint_num_; j++)
          new_state.joint_states.at(j) = state_tmp->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[j];
      }

    /* special for the dragon kinematics */
    sensor_msgs::JointState joint_state;
    joint_state.name = start_state_.joint_names;
    joint_state.position = new_state.joint_states;

    /* consider rootlink (link1) as baselink */
    transform_controller_->setCogDesireOrientation(KDL::Rotation::Quaternion(new_state.root_state.at(3),
                                                                             new_state.root_state.at(4),
                                                                             new_state.root_state.at(5),
                                                                             new_state.root_state.at(6)));

    transform_controller_->kinematics(joint_state);
    if(transform_controller_->distThreCheck() < min_var_)
      {
        min_var_ = transform_controller_->distThreCheck();
        min_var_state_ = path_.size();
      }

    transform_controller_->modelling();
    if(transform_controller_->getStableState().maxCoeff() > max_force_)
      {
        max_force_ = transform_controller_->getStableState().maxCoeff();
        max_force_state_ = path_.size();
      }

    //set the nominal gimbal angles for moveit collision check
    std::vector<double> gimbal_nominal_angles = transform_controller_->getGimbalNominalAngles();
    for(int i = 0; i < gimbal_nominal_angles.size(); i++)
        new_state.joint_states.at(joint_num_ + i) = gimbal_nominal_angles.at(i); // roll -> pitch -> roll ->pitch

    /* log out */
    double r, p, y; new_state.getRootRPY(r, p, y);
    ROS_INFO("index: %d, dist_var: %f, max_force: %f, base pose: [%f, %f, %f] att: [%f, %f, %f]", (int)path_.size(), transform_controller_->distThreCheck(), transform_controller_->getStableState().maxCoeff(), new_state.root_state.at(0), new_state.root_state.at(1), new_state.root_state.at(2), r, p, y);

    se2::MotionPlanning::addState(new_state);
  }

  /* keypose size: 7 + joint_state_size */
  State MotionPlanning::root2cog(const std::vector<double> &keypose)
  {
    static double max_pitch_tilt = 0;
    static double max_roll_tilt = 0;
    State state = start_state_;

    /* root */
    tf::Transform root_world(tf::Quaternion(keypose[3], keypose[4], keypose[5], keypose[6]),
                             tf::Vector3(keypose[0], keypose[1], keypose[2]));

    for(int i = 0; i < 7; i++) state.root_state.at(i) = keypose.at(i);

    /* joint state */
    sensor_msgs::JointState joint_state;
    joint_state.name = start_state_.joint_names;
    for(int i = 0; i < joint_state.name.size(); i++)
      state.joint_states.at(i) = keypose.at(7 + i);
    joint_state.position = state.joint_states;

    /* cog */
    tf::Quaternion baselink_q = root_world * transform_controller_->getRoot2Link(base_link_, joint_state).getRotation();
    state.cog_state.at(3) = baselink_q.x();
    state.cog_state.at(4) = baselink_q.y();
    state.cog_state.at(5) = baselink_q.z();
    state.cog_state.at(6) = baselink_q.w();

    KDL::Rotation kdl_q;
    tf::quaternionTFToKDL(baselink_q, kdl_q);
    transform_controller_->setCogDesireOrientation(kdl_q);

    transform_controller_->kinematics(joint_state);
    tf::Vector3 cog_world_pos = root_world * transform_controller_->getCog().getOrigin();

    std::vector<double> keypose_cog;
    state.cog_state.at(0) = cog_world_pos.getX();
    state.cog_state.at(1) = cog_world_pos.getY();
    state.cog_state.at(2) = cog_world_pos.getZ();

    /* gimbal angle check */
    for (int i = 0; i < (joint_num_ + 2) / 2; i ++)
      {
        if (fabs(state.joint_states.at(joint_num_ + 2 * i)) > max_roll_tilt )
          {
            max_roll_tilt = fabs(state.joint_states.at(joint_num_ + 2 * i));
            //ROS_WARN("max roll tilt is %f", max_roll_tilt);
          }

        if (fabs(state.joint_states.at(joint_num_ + 2 * i + 1)) > max_pitch_tilt )
          {
            max_pitch_tilt = fabs(state.joint_states.at(joint_num_ + 2 * i + 1));
            //ROS_WARN("max pitch tilt is %f", max_pitch_tilt);
          }
      }
    return state;
  }

}
