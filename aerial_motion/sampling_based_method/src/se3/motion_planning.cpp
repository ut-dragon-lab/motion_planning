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


#include <sampling_based_method/se3/motion_planning.h>

/* the StabilityObjective does not work!!!!! */
namespace se3
{
  MotionPlanning::MotionPlanning(ros::NodeHandle nh, ros::NodeHandle nhp, boost::shared_ptr<TransformController> transform_controller):
    se2::MotionPlanning(nh, nhp, transform_controller), max_force_(0), max_force_state_(0)
  {
  }

#if 0
  void MotionPlanning::robotInit()
  {
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
#endif

  bool MotionPlanning::isStateValid(const ompl::base::State *state)
  {
    MultilinkState current_state = start_state_;
    geometry_msgs::Pose root_pose;
    if(planning_mode_ == sampling_based_method::PlanningMode::ONLY_JOINTS_MODE)
      {
        int index = 0;
        for(auto itr : transform_controller_->getActuatorJointMap())
          current_state.setActuatorState(itr, state->as<ompl::base::RealVectorStateSpace::StateType>()->values[index++]);
      }
    else if(planning_mode_ == sampling_based_method::PlanningMode::ONLY_BASE_MODE)
      {
        if(motion_type_ == sampling_based_method::PlanningMode::SE2)
          {
            root_pose.position.x = state->as<ompl::base::SE2StateSpace::StateType>()->getX();
            root_pose.position.y = state->as<ompl::base::SE2StateSpace::StateType>()->getY();
            root_pose.orientation = tf::createQuaternionMsgFromYaw(state->as<ompl::base::SE2StateSpace::StateType>()->getYaw());
          }

        if(motion_type_ == sampling_based_method::PlanningMode::SE3)
          {
            root_pose.position.x = state->as<ompl::base::SE3StateSpace::StateType>()->getX();
            root_pose.position.y = state->as<ompl::base::SE3StateSpace::StateType>()->getY();
            root_pose.position.z = state->as<ompl::base::SE3StateSpace::StateType>()->getZ();
            root_pose.orientation.x = state->as<ompl::base::SE3StateSpace::StateType>()->rotation().x;
            root_pose.orientation.y = state->as<ompl::base::SE3StateSpace::StateType>()->rotation().y;
            root_pose.orientation.z = state->as<ompl::base::SE3StateSpace::StateType>()->rotation().z;
            root_pose.orientation.w = state->as<ompl::base::SE3StateSpace::StateType>()->rotation().w;
          }
      }
    else if(planning_mode_ == sampling_based_method::PlanningMode::JOINTS_AND_BASE_MODE)
      {
        const ompl::base::CompoundState* state_tmp = dynamic_cast<const ompl::base::CompoundState*>(state);
        if(motion_type_ == sampling_based_method::PlanningMode::SE2)
          {
            root_pose.position.x = state_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->getX();
            root_pose.position.y = state_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->getY();
            root_pose.orientation = tf::createQuaternionMsgFromYaw( state_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->getYaw());
          }
        if(motion_type_ == sampling_based_method::PlanningMode::SE3)
          {
            root_pose.position.x = state_tmp->as<ompl::base::SE3StateSpace::StateType>(0)->getX();
            root_pose.position.y = state_tmp->as<ompl::base::SE3StateSpace::StateType>(0)->getY();
            root_pose.position.z = state_tmp->as<ompl::base::SE3StateSpace::StateType>(0)->getZ();
            root_pose.orientation.x = state_tmp->as<ompl::base::SE3StateSpace::StateType>(0)->rotation().x;
            root_pose.orientation.y = state_tmp->as<ompl::base::SE3StateSpace::StateType>(0)->rotation().y;
            root_pose.orientation.z = state_tmp->as<ompl::base::SE3StateSpace::StateType>(0)->rotation().z;
            root_pose.orientation.w = state_tmp->as<ompl::base::SE3StateSpace::StateType>(0)->rotation().w;
          }

        int index = 0;
        for(auto itr : transform_controller_->getActuatorJointMap())
          current_state.setActuatorState(itr, state_tmp->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[index++]);
      }
    current_state.setRootPose(root_pose);

    /* special for the dragon kinematics */
    /* consider rootlink (link1) as baselink */
    KDL::Rotation q;
    tf::quaternionMsgToKDL(root_pose.orientation, q);
    transform_controller_->setCogDesireOrientation(q);
    transform_controller_->forwardKinematics(current_state.getActuatorStateNonConst());
    if(!transform_controller_->stabilityMarginCheck()) return false;
    if(!transform_controller_->overlapCheck()) return false;
    if(!transform_controller_->modelling()) return false;

#if 0
    //set the nominal gimbal angles for moveit collision check
    std::vector<double> gimbal_nominal_angles = boost::dynamic_pointer_cast<DragonTransformController>(transform_controller_)->getGimbalNominalAngles();
    for(int i = 0; i < gimbal_nominal_angles.size(); i++)
      {
        current_state.joint_states.at(joint_num_ + i) = gimbal_nominal_angles.at(i); // roll -> pitch -> roll ->pitch

        /*  the gimbal tilt angle */
        /* roll */
        if (i / 2 == 0 && fabs(gimbal_nominal_angles.at(i)) > gimbal_roll_thresh_) return false;
        /* pitch */
        if (i / 2 == 1 && fabs(gimbal_nominal_angles.at(i)) > gimbal_pitch_thresh_) return false;
      }
#endif

    //check collision
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;

    robot_state::RobotState robot_state = setRobotState2Moveit(current_state);
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

        if(!load_path_flag_)
          {
            /* planning mode */
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
      }

    collision_object.operation = collision_object.ADD;
    planning_scene_->processCollisionObjectMsg(collision_object);
    ROS_ERROR("set env init");
    /* CAUTION: important */
    /* special for the dragon kinematics to get start state correct joint*/
    transform_controller_->setBaselink(std::string("link1"));
    KDL::Rotation q;
    tf::quaternionMsgToKDL(start_state_.getRootPoseConst().orientation, q);
    transform_controller_->setCogDesireOrientation(q);
    transform_controller_->forwardKinematics(start_state_.getActuatorStateNonConst());
    tf::quaternionMsgToKDL(goal_state_.getRootPoseConst().orientation, q);
    transform_controller_->setCogDesireOrientation(q);
    transform_controller_->forwardKinematics(goal_state_.getActuatorStateNonConst());
    /* set the correct base link ( which is not root_link = link1), to be suitable for the control system */
    transform_controller_->setBaselink(base_link_);
  }

  void MotionPlanning::planInit()
  {
    //set root link as the baselink for the planning
    transform_controller_->setBaselink(std::string("link1"));

    //planning
    //x, y
    ompl::base::StateSpacePtr r_base;
    if(motion_type_ == sampling_based_method::PlanningMode::SE2)
      {
        r_base = std::shared_ptr<ompl::base::SE2StateSpace>(new ompl::base::SE2StateSpace());

        ompl::base::RealVectorBounds motion_bounds(2);
        motion_bounds.low[0] = x_low_bound_;
        motion_bounds.low[1] = y_low_bound_;
        motion_bounds.high[0] = x_high_bound_;
        motion_bounds.high[1] = y_high_bound_;
        r_base->as<ompl::base::SE2StateSpace>()->setBounds(motion_bounds);
      }

    if(motion_type_ == sampling_based_method::PlanningMode::SE3)
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

    if(planning_mode_ == sampling_based_method::PlanningMode::ONLY_JOINTS_MODE)
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
    else if(planning_mode_ == sampling_based_method::PlanningMode::ONLY_BASE_MODE)
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
    else if(planning_mode_ == sampling_based_method::PlanningMode::JOINTS_AND_BASE_MODE)
      {
        config_space_ = r_base + r_joints;
        config_space_->as<ompl::base::CompoundStateSpace>()->setSubspaceWeight(1, 0.001);
        space_information_  = ompl::base::SpaceInformationPtr(new ompl::base::SpaceInformation(config_space_));
        space_information_->setStateValidityChecker(boost::bind(&MotionPlanning::isStateValid, this, _1));
      }

    /* init state */
    ompl::base::ScopedState<> start(config_space_);
    ompl::base::ScopedState<> goal(config_space_);
    if(planning_mode_ == sampling_based_method::PlanningMode::ONLY_JOINTS_MODE)
      {
        for(int i = 0; i < joint_num_; i ++)
          {
            start->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = start_state_.getActuatorStateConst().position.at(transform_controller_->getActuatorJointMap().at(i));
            goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = goal_state_.getActuatorStateConst().position.at(transform_controller_->getActuatorJointMap().at(i));
          }
      }
    else if(planning_mode_ == sampling_based_method::PlanningMode::ONLY_BASE_MODE)
      {
        if(motion_type_ == sampling_based_method::PlanningMode::SE2)
          {
            start->as<ompl::base::SE2StateSpace::StateType>()->setXY(start_state_.getRootPoseConst().position.x, start_state_.getRootPoseConst().position.y);
            start->as<ompl::base::SE2StateSpace::StateType>()->setYaw(tf::getYaw(start_state_.getRootPoseConst().orientation));
            goal->as<ompl::base::SE2StateSpace::StateType>()->setXY(goal_state_.getRootPoseConst().position.x, goal_state_.getRootPoseConst().position.y);
            goal->as<ompl::base::SE2StateSpace::StateType>()->setYaw(tf::getYaw(goal_state_.getRootPoseConst().orientation));
          }
        if(motion_type_ == sampling_based_method::PlanningMode::SE3)
          {
            start->as<ompl::base::SE3StateSpace::StateType>()->setXYZ(start_state_.getRootPoseConst().position.x, start_state_.getRootPoseConst().position.y,start_state_.getRootPoseConst().position.z);
            start->as<ompl::base::SE3StateSpace::StateType>()->rotation().x = start_state_.getRootPoseConst().orientation.x;
            start->as<ompl::base::SE3StateSpace::StateType>()->rotation().y = start_state_.getRootPoseConst().orientation.y;
            start->as<ompl::base::SE3StateSpace::StateType>()->rotation().z = start_state_.getRootPoseConst().orientation.z;
            start->as<ompl::base::SE3StateSpace::StateType>()->rotation().w = start_state_.getRootPoseConst().orientation.w;

            goal->as<ompl::base::SE3StateSpace::StateType>()->setXYZ(goal_state_.getRootPoseConst().position.x, goal_state_.getRootPoseConst().position.y, goal_state_.getRootPoseConst().position.z);
            goal->as<ompl::base::SE3StateSpace::StateType>()->rotation().x = goal_state_.getRootPoseConst().orientation.x;
            goal->as<ompl::base::SE3StateSpace::StateType>()->rotation().y = goal_state_.getRootPoseConst().orientation.y;
            goal->as<ompl::base::SE3StateSpace::StateType>()->rotation().z = goal_state_.getRootPoseConst().orientation.z;
            goal->as<ompl::base::SE3StateSpace::StateType>()->rotation().w = goal_state_.getRootPoseConst().orientation.w;
          }
      }
    else if(planning_mode_ == sampling_based_method::PlanningMode::JOINTS_AND_BASE_MODE)
      {
        ompl::base::CompoundState* start_tmp = dynamic_cast<ompl::base::CompoundState*> (start.get());
        ompl::base::CompoundState* goal_tmp = dynamic_cast<ompl::base::CompoundState*> (goal.get());

        if(motion_type_ == sampling_based_method::PlanningMode::SE2)
          {
            start_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->setXY(start_state_.getRootPoseConst().position.x, start_state_.getRootPoseConst().position.y);
            start_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->setYaw(tf::getYaw(start_state_.getRootPoseConst().orientation));
            ompl::base::CompoundState* goal_tmp = dynamic_cast<ompl::base::CompoundState*> (goal.get());
            goal_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->setXY(goal_state_.getRootPoseConst().position.x, goal_state_.getRootPoseConst().position.y);
            goal_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->setYaw(tf::getYaw(goal_state_.getRootPoseConst().orientation));
          }
        if(motion_type_ == sampling_based_method::PlanningMode::SE3)
          {
            start_tmp->as<ompl::base::SE3StateSpace::StateType>(0)->setXYZ(start_state_.getRootPoseConst().position.x, start_state_.getRootPoseConst().position.y, start_state_.getRootPoseConst().position.z);
            start_tmp->as<ompl::base::SE3StateSpace::StateType>(0)->rotation().x = start_state_.getRootPoseConst().orientation.x;
            start_tmp->as<ompl::base::SE3StateSpace::StateType>(0)->rotation().y = start_state_.getRootPoseConst().orientation.y;
            start_tmp->as<ompl::base::SE3StateSpace::StateType>(0)->rotation().z = start_state_.getRootPoseConst().orientation.z;
            start_tmp->as<ompl::base::SE3StateSpace::StateType>(0)->rotation().w = start_state_.getRootPoseConst().orientation.w;

            goal_tmp->as<ompl::base::SE3StateSpace::StateType>(0)->setXYZ(goal_state_.getRootPoseConst().position.x, goal_state_.getRootPoseConst().position.y, goal_state_.getRootPoseConst().position.z);
            goal_tmp->as<ompl::base::SE3StateSpace::StateType>(0)->rotation().x = goal_state_.getRootPoseConst().orientation.x;
            goal_tmp->as<ompl::base::SE3StateSpace::StateType>(0)->rotation().y = goal_state_.getRootPoseConst().orientation.y;
            goal_tmp->as<ompl::base::SE3StateSpace::StateType>(0)->rotation().z = goal_state_.getRootPoseConst().orientation.z;
            goal_tmp->as<ompl::base::SE3StateSpace::StateType>(0)->rotation().w = goal_state_.getRootPoseConst().orientation.w;
          }
        for(int i = 0; i < joint_num_; i ++)
          {
            start_tmp->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[i] = start_state_.getActuatorStateConst().position.at(transform_controller_->getActuatorJointMap().at(i));
            goal_tmp->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[i] = goal_state_.getActuatorStateConst().position.at(transform_controller_->getActuatorJointMap().at(i));
          }
      }

    pdef_ = ompl::base::ProblemDefinitionPtr(new ompl::base::ProblemDefinition(space_information_));
    pdef_->setStartAndGoalStates(start, goal);
  }

  void MotionPlanning::rosParamInit()
  {
    se2::MotionPlanning::rosParamInit();

    nhp_.param("gap_type", gap_type_, 0); //Type0: vertical; Type1: horizontal

    /* z, roll, pitch, joint bound */
    nhp_.param("z_low_bound", z_low_bound_, -2.2);
    nhp_.param("z_high_bound", z_high_bound_, 2.2);
    nhp_.param("pitch_joint_low_bound", pitch_joint_low_bound_, -1.58);
    nhp_.param("pitch_joint_high_bound", pitch_joint_high_bound_, 1.58);
    nhp_.param("yaw_joint_low_bound", yaw_joint_low_bound_, -1.58);
    nhp_.param("yaw_joint_high_bound", yaw_joint_high_bound_, 1.58);

    double r, p, y, z;
    nhp_.param("start_state_z", start_state_.getRootPoseNonConst().position.z, 0.0);
    nhp_.param("start_state_roll", r, 0.0);
    nhp_.param("start_state_pitch", p, 0.0);
    nhp_.param("start_state_yaw", y, 0.0);
    start_state_.setRootOrientation(tf::createQuaternionMsgFromRollPitchYaw(r,p,y));

    nhp_.param("goal_state_z", goal_state_.getRootPoseNonConst().position.z, 0.0);
    nhp_.param("goal_state_roll", r, 0.0);
    nhp_.param("goal_state_pitch", p, 0.0);
    nhp_.param("goal_state_yaw", y, 0.0);
    goal_state_.setRootOrientation(tf::createQuaternionMsgFromRollPitchYaw(r,p,y));

    if(nhp_.hasParam("goal_state_qx"))
      {
        ROS_WARN("get quaternion goal rotation");
        geometry_msgs::Quaternion q;
        nhp_.getParam("goal_state_qx", goal_state_.getRootPoseNonConst().orientation.x);
        nhp_.getParam("goal_state_qy", goal_state_.getRootPoseNonConst().orientation.y);
        nhp_.getParam("goal_state_qz", goal_state_.getRootPoseNonConst().orientation.z);
        nhp_.getParam("goal_state_qw", goal_state_.getRootPoseNonConst().orientation.w);
      }

    nhp_.param("gimbal_roll_thresh", gimbal_roll_thresh_, 1.6);
    nhp_.param("gimbal_pitch_thresh", gimbal_pitch_thresh_, 1.1);

    /* test
    ROS_WARN("start pos: [%f, %f, %f], goal pos: [%f, %f, %f]",
             start_state_.getRootPoseNonConst().position.x,
             start_state_.getRootPoseNonConst().position.y,
             start_state_.getRootPoseNonConst().position.z,
             goal_state_.getRootPoseNonConst().position.x,
             goal_state_.getRootPoseNonConst().position.y,
             goal_state_.getRootPoseNonConst().position.z);
    */
  }

  void MotionPlanning::addState(ompl::base::State *ompl_state)
  {
    MultilinkState new_state = start_state_;
    geometry_msgs::Pose root_pose;

    if(planning_mode_ == sampling_based_method::PlanningMode::ONLY_BASE_MODE)
      {
        if(motion_type_ == sampling_based_method::PlanningMode::SE2)
          {
            root_pose.position.x = ompl_state->as<ompl::base::SE2StateSpace::StateType>()->getX();
            root_pose.position.y = ompl_state->as<ompl::base::SE2StateSpace::StateType>()->getY();
            root_pose.orientation = tf::createQuaternionMsgFromYaw(ompl_state->as<ompl::base::SE2StateSpace::StateType>()->getYaw());
          }
        if(motion_type_ == sampling_based_method::PlanningMode::SE3)
          {
            root_pose.position.x = ompl_state->as<ompl::base::SE3StateSpace::StateType>()->getX();
            root_pose.position.y = ompl_state->as<ompl::base::SE3StateSpace::StateType>()->getY();
            root_pose.position.z = ompl_state->as<ompl::base::SE3StateSpace::StateType>()->getZ();
            root_pose.orientation.x = ompl_state->as<ompl::base::SE3StateSpace::StateType>()->rotation().x;
            root_pose.orientation.y = ompl_state->as<ompl::base::SE3StateSpace::StateType>()->rotation().y;
            root_pose.orientation.z = ompl_state->as<ompl::base::SE3StateSpace::StateType>()->rotation().z;
            root_pose.orientation.w = ompl_state->as<ompl::base::SE3StateSpace::StateType>()->rotation().w;
          }
      }
    else if(planning_mode_ == sampling_based_method::PlanningMode::JOINTS_AND_BASE_MODE)
      {
        const ompl::base::CompoundState* state_tmp = dynamic_cast<const ompl::base::CompoundState*>(ompl_state);

        /* SE2 */
        if(motion_type_ == sampling_based_method::PlanningMode::SE2)
          {
            root_pose.position.x = state_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->getX();
            root_pose.position.y = state_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->getY();
            root_pose.orientation = tf::createQuaternionMsgFromYaw( state_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->getYaw());
          }

        /* SE3 */
        if(motion_type_ == sampling_based_method::PlanningMode::SE3)
          {
            root_pose.position.x = state_tmp->as<ompl::base::SE3StateSpace::StateType>(0)->getX();
            root_pose.position.y = state_tmp->as<ompl::base::SE3StateSpace::StateType>(0)->getY();
            root_pose.position.z = state_tmp->as<ompl::base::SE3StateSpace::StateType>(0)->getZ();
            root_pose.orientation.x = state_tmp->as<ompl::base::SE3StateSpace::StateType>(0)->rotation().x;
            root_pose.orientation.y = state_tmp->as<ompl::base::SE3StateSpace::StateType>(0)->rotation().y;
            root_pose.orientation.z = state_tmp->as<ompl::base::SE3StateSpace::StateType>(0)->rotation().z;
            root_pose.orientation.w = state_tmp->as<ompl::base::SE3StateSpace::StateType>(0)->rotation().w;
          }

        int index = 0;
        for(auto itr : transform_controller_->getActuatorJointMap())
          new_state.setActuatorState(itr, state_tmp->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[index++]);
      }

    new_state.setRootPose(root_pose);

    /* special for the dragon kinematics */
    /* consider rootlink (link1) as baselink */
    KDL::Rotation q;
    tf::quaternionMsgToKDL(new_state.getRootPoseConst().orientation, q);
    transform_controller_->setCogDesireOrientation(q);
    transform_controller_->forwardKinematics(new_state.getActuatorStateNonConst());

    transform_controller_->stabilityMarginCheck();

    if(transform_controller_->getStabilityMargin() < min_var_)
      {
        min_var_ = transform_controller_->getStabilityMargin();
        min_var_state_ = path_.size();
      }

    transform_controller_->modelling();
    if(transform_controller_->getOptimalHoveringThrust().maxCoeff() > max_force_)
      {
        max_force_ = transform_controller_->getOptimalHoveringThrust().maxCoeff();
        max_force_state_ = path_.size();
      }

#if 0
    //set the nominal gimbal angles for moveit collision check
    std::vector<double> gimbal_nominal_angles = boost::dynamic_pointer_cast<DragonTransformController>(transform_controller_)->getGimbalNominalAngles();
    for(int i = 0; i < gimbal_nominal_angles.size(); i++)
        new_state.joint_states.at(joint_num_ + i) = gimbal_nominal_angles.at(i); // roll -> pitch -> roll ->pitch
#endif

    /* log out */
    double r, p, y;
    tf::Quaternion tf_q;
    quaternionMsgToTF(new_state.getRootPoseConst().orientation, tf_q);
    tf::Matrix3x3(tf_q).getRPY(r, p, y);
    ROS_INFO("index: %d, dist_var: %f, max_force: %f, base pose: [%f, %f, %f] att: [%f, %f, %f]", (int)path_.size(), transform_controller_->getStabilityMargin(), transform_controller_->getOptimalHoveringThrust().maxCoeff(), new_state.getRootPoseConst().position.x, new_state.getRootPoseConst().position.y, new_state.getRootPoseConst().position.z, r, p, y);

    se2::MotionPlanning::addState(new_state);
  }

#if 0
  /* keypose size: 7 + joint_state_size (no gimbal) */
  State MotionPlanning::cog2root(const std::vector<double> &keypose)
  {
    State state = start_state_;

    /* cog */
    for(int i = 0; i < 7; i++) state.cog_state.at(i) = keypose.at(i);
    tf::Transform cog_world(tf::Quaternion(keypose[3], keypose[4], keypose[5], keypose[6]),
                            tf::Vector3(keypose[0], keypose[1], keypose[2]));

    /* joint state */
    sensor_msgs::JointState joint_state;
    joint_state.name = start_state_.joint_names;
    joint_state.position.resize(joint_state.name.size(), 0);
    for(int i = 0; i < joint_num_; i++)
      {
        state.joint_states.at(i) = keypose.at(7 + i);
        joint_state.position.at(i) = keypose.at(7 + i);
      }

    KDL::Rotation kdl_q;
    tf::quaternionTFToKDL(baselink_desired_att_, kdl_q);
    transform_controller_->setCogDesireOrientation(kdl_q);
    transform_controller_->forwardKinematics(joint_state);
    tf::Transform cog_root = transform_controller_->getCog(); // cog in root frame
    std::vector<double> gimbal_nominal_angles = boost::dynamic_pointer_cast<DragonTransformController>(transform_controller_)->getGimbalNominalAngles();
    for(int i = 0; i < gimbal_nominal_angles.size(); i++)
      {
        // ROS_INFO("%s: %f vs %f",
        //          start_state_.joint_names.at(joint_num_ + i).c_str(),
        //          joint_state.position.at(joint_num_ + i), gimbal_nominal_angles.at(i));
        state.joint_states.at(joint_num_ + i) = gimbal_nominal_angles.at(i);
      }

    /* root */
    tf::Transform root_world = cog_world * transform_controller_->getCog().inverse();
    state.root_state.at(0) = root_world.getOrigin().getX();
    state.root_state.at(1) = root_world.getOrigin().getY();
    state.root_state.at(2) = root_world.getOrigin().getZ();

    tf::Quaternion q = root_world.getRotation();
    state.root_state.at(3) = q.x();
    state.root_state.at(4) = q.y();
    state.root_state.at(5) = q.z();
    state.root_state.at(6) = q.w();

    return state;
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

    /* cog: true baselink (e.g. fc) */
    tf::Quaternion baselink_q = root_world * transform_controller_->getRoot2Link(base_link_, joint_state).getRotation();
    state.cog_state.at(3) = baselink_q.x();
    state.cog_state.at(4) = baselink_q.y();
    state.cog_state.at(5) = baselink_q.z();
    state.cog_state.at(6) = baselink_q.w();

    KDL::Rotation kdl_q;
    tf::quaternionTFToKDL(baselink_q, kdl_q);
    transform_controller_->setCogDesireOrientation(kdl_q);
    transform_controller_->forwardKinematics(joint_state);
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
            if (play_path_flag_)
              ROS_WARN("index: %d, max roll tilt is %f", (int)path_.size(), max_roll_tilt);
          }

        if (fabs(state.joint_states.at(joint_num_ + 2 * i + 1)) > max_pitch_tilt )
          {
            max_pitch_tilt = fabs(state.joint_states.at(joint_num_ + 2 * i + 1));
            if (play_path_flag_)
              ROS_WARN("index: %d, max pitch tilt is %f", (int)path_.size(), max_pitch_tilt);
          }
      }
    return state;
  }
#endif
}
