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

namespace sampling_base
{
  namespace se3
  {
    MotionPlanning::MotionPlanning(ros::NodeHandle nh, ros::NodeHandle nhp, boost::shared_ptr<HydrusRobotModel> robot_model_ptr):
      se2::MotionPlanning(nh, nhp, robot_model_ptr), max_force_(0), max_force_state_(0)
    {
      rosParamInit();
    }

    bool MotionPlanning::isStateValid(const ompl::base::State *state)
    {
      geometry_msgs::Pose root_pose;
      KDL::JntArray actuator_state(robot_model_ptr_->getActuatorMap().size());

      if(planning_mode_ == sampling_based_method::PlanningMode::ONLY_JOINTS_MODE)
        {
          int index = 0;
          for(auto itr : robot_model_ptr_->getLinkJointIndex())
            actuator_state(itr) = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[index++];
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
          for(auto itr : robot_model_ptr_->getLinkJointIndex())
            actuator_state(itr) = state_tmp->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[index++];
        }

      /* TODO: 
         the robot_model is also updated, in the following command,
         which is not good.
      */
      MultilinkState current_state(robot_model_ptr_, root_pose, actuator_state);

#if 0
      /* special for the dragon kinematics */
      /* consider rootlink (link1) as baselink */
      KDL::Rotation q;
      tf::quaternionMsgToKDL(root_pose.orientation, q);
      robot_model_ptr_->setCogDesireOrientation(q);
      robot_model_ptr_->updateRobotModel(actuator_state);
#endif
      if(!robot_model_ptr_->stabilityMarginCheck()) return false;
      if(!robot_model_ptr_->overlapCheck()) return false;
      if(!robot_model_ptr_->modelling()) return false;

      //check collision
      collision_detection::CollisionRequest collision_request;
      collision_detection::CollisionResult collision_result;
      planning_scene_->setCurrentState(current_state.getRootActuatorStateConst<moveit_msgs::RobotState>());
      planning_scene_->checkCollision(collision_request, collision_result, planning_scene_->getCurrentState(), acm_);

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

          bool load_path_flag;
          nhp_.param("load_path_flag", load_path_flag, false);
          if(!load_path_flag)
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
    }

    void MotionPlanning::planInit()
    {
      //set root link as the baselink for the planning
      robot_model_ptr_->setBaselinkName(std::string("link1"));

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
      ompl::base::StateSpacePtr r_joints(new ompl::base::RealVectorStateSpace(robot_model_ptr_->getLinkJointIndex().size()));
      ompl::base::RealVectorBounds joint_bounds(robot_model_ptr_->getLinkJointIndex().size());

      for(int j = 0; j < robot_model_ptr_->getLinkJointIndex().size(); j++ )
        {
          joint_bounds.low[j] = joint_lower_limits_.at(j);
          joint_bounds.high[j] = joint_upper_limits_.at(j);
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
          for(int i = 0; i < robot_model_ptr_->getLinkJointIndex().size(); i ++)
            {
              start->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = start_state_.getActuatorStateConst()(robot_model_ptr_->getLinkJointIndex().at(i));
              goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = goal_state_.getActuatorStateConst()(robot_model_ptr_->getLinkJointIndex().at(i));
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

          for(int i = 0; i < robot_model_ptr_->getLinkJointIndex().size(); i ++)
            {
              start_tmp->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[i] = start_state_.getActuatorStateConst()(robot_model_ptr_->getLinkJointIndex().at(i));
              goal_tmp->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[i] = goal_state_.getActuatorStateConst()(robot_model_ptr_->getLinkJointIndex().at(i));
            }
        }

      pdef_ = ompl::base::ProblemDefinitionPtr(new ompl::base::ProblemDefinition(space_information_));
      pdef_->setStartAndGoalStates(start, goal);
    }

    void MotionPlanning::rosParamInit()
    {

      nhp_.param("gap_type", gap_type_, 0); //Type0: vertical; Type1: horizontal

      /* z, roll, pitch, joint bound */
      nhp_.param("z_low_bound", z_low_bound_, -2.2);
      nhp_.param("z_high_bound", z_high_bound_, 2.2);

      joint_lower_limits_.resize(robot_model_ptr_->getLinkJointNames().size());
      joint_upper_limits_.resize(robot_model_ptr_->getLinkJointNames().size());
      for(int i = 0; i < robot_model_ptr_->getLinkJointNames().size(); i++)
        {
          nhp_.param(robot_model_ptr_->getLinkJointNames().at(i) + std::string("_lower_limit"), joint_lower_limits_.at(i), robot_model_ptr_->getLinkJointLowerLimits().at(i));
          nhp_.param(robot_model_ptr_->getLinkJointNames().at(i) + std::string("_upper_limit"), joint_upper_limits_.at(i), robot_model_ptr_->getLinkJointUpperLimits().at(i));
        }

      /* overwrite */
      geometry_msgs::Pose pose = start_state_.getRootPoseConst();
      auto actuator_state = start_state_.getActuatorStateConst();
      nhp_.param("start_state_z", pose.position.z, 0.0);
      double r,p,y;
      nhp_.param("start_state_roll", r, 0.0);
      nhp_.param("start_state_pitch", p, 0.0);
      nhp_.param("start_state_yaw", y, 0.0);
      pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(r,p,y);
      start_state_.setStatesFromRoot(robot_model_ptr_, pose, actuator_state);

      pose = goal_state_.getRootPoseConst();
      actuator_state = goal_state_.getActuatorStateConst();
      nhp_.param("goal_state_z", pose.position.z, 0.0);
      nhp_.param("goal_state_roll", r, 0.0);
      nhp_.param("goal_state_pitch", p, 0.0);
      nhp_.param("goal_state_yaw", y, 0.0);
      pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(r,p,y);
      if(nhp_.hasParam("goal_state_qx"))
        {
          ROS_WARN("get quaternion goal rotation");
          geometry_msgs::Quaternion q;
          nhp_.getParam("goal_state_qx", pose.orientation.x);
          nhp_.getParam("goal_state_qy", pose.orientation.y);
          nhp_.getParam("goal_state_qz", pose.orientation.z);
          nhp_.getParam("goal_state_qw", pose.orientation.w);
        }
      goal_state_.setStatesFromRoot(robot_model_ptr_, pose, actuator_state);
    }

    void MotionPlanning::addState(ompl::base::State *ompl_state)
    {
      geometry_msgs::Pose root_pose;
      KDL::JntArray actuator_state(robot_model_ptr_->getActuatorMap().size());

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
          for(auto itr : robot_model_ptr_->getLinkJointIndex())
            actuator_state(itr) = state_tmp->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[index++];
        }

      /* special  */
      /* consider rootlink (link1) as baselink */
      KDL::Rotation q;
      tf::quaternionMsgToKDL(root_pose.orientation, q);
      robot_model_ptr_->setCogDesireOrientation(q);
      robot_model_ptr_->updateRobotModel(actuator_state);
      robot_model_ptr_->stabilityMarginCheck();

      if(robot_model_ptr_->getStabilityMargin() < min_var_)
        {
          min_var_ = robot_model_ptr_->getStabilityMargin() ;
          min_var_state_ = path_.size();
        }

      robot_model_ptr_->modelling();
      if(robot_model_ptr_->getOptimalHoveringThrust().maxCoeff() > max_force_)
        {
          max_force_ = robot_model_ptr_->getOptimalHoveringThrust().maxCoeff();
          max_force_state_ = path_.size();
        }

      /* log out */
      double r, p, y;
      tf::Quaternion tf_q;
      quaternionMsgToTF(root_pose.orientation, tf_q);
      tf::Matrix3x3(tf_q).getRPY(r, p, y);
      ROS_INFO("index: %d, dist_var: %f, max_force: %f, base pose: [%f, %f, %f] att: [%f, %f, %f]", (int)path_.size(), robot_model_ptr_->getStabilityMargin(), robot_model_ptr_->getOptimalHoveringThrust().maxCoeff(), root_pose.position.x, root_pose.position.y, root_pose.position.z, r, p, y);

      se2::MotionPlanning::addState(MultilinkState(robot_model_ptr_, root_pose, actuator_state));
    }
  }
}
