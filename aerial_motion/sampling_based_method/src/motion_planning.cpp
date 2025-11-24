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

#include <sampling_based_method/motion_planning.h>

/* TODO: this is an workaround */
#include <hydrus/hydrus_robot_model.h>


namespace
{
  int state_index = 0;
}

namespace sampling_base
{
  MotionPlanning::MotionPlanning(ros::NodeHandle nh, ros::NodeHandle nhp, boost::shared_ptr<aerial_robot_model::transformable::RobotModel> robot_model_ptr):
    nh_(nh), nhp_(nhp), robot_model_ptr_(robot_model_ptr),
    path_(0), calculation_time_(0),
    best_cost_(-1), min_var_(1e6),
    max_force_state_index_(0), min_force_state_index_(1e6),
    planning_mode_(sampling_based_method::PlanningMode::ONLY_JOINTS_MODE),
    motion_type_(sampling_based_method::PlanningMode::SE2)
  {
    /* ros pub/sub and service */
    planning_scene_diff_pub_ = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

    /* init */
    rosParamInit();

    /* set original baselink name */
    baselink_name_ = robot_model_ptr_->getBaselinkName();
  }

  void MotionPlanning::motionSequence()
  {
    /* moveit */
    planning_scene_->setCurrentState(path_.at(state_index).getRootJointStateConst<moveit_msgs::RobotState>());
    planning_scene_->getPlanningSceneMsg(planning_scene_msg_);
    planning_scene_msg_.is_diff = true;
    planning_scene_diff_pub_.publish(planning_scene_msg_);
  }

  bool  MotionPlanning::isStateValid(const ompl::base::State *state)
  {
    geometry_msgs::Pose root_pose;
    KDL::JntArray joint_state(robot_model_ptr_->getJointIndexMap().size());

    if(planning_mode_ == sampling_based_method::PlanningMode::ONLY_JOINTS_MODE)
      {
        int index = 0;
        for(auto itr : robot_model_ptr_->getLinkJointIndices())
          joint_state(itr) = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[index++];
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
        for(auto itr : robot_model_ptr_->getLinkJointIndices())
          joint_state(itr) = state_tmp->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[index++];
      }

    /* TODO: 
       the robot_model is also updated, in the following command,
       which is not good.
    */
    MultilinkState current_state(robot_model_ptr_, root_pose, joint_state);

    //check distance thresold
    if(planning_mode_ == sampling_based_method::PlanningMode::JOINTS_AND_BASE_MODE ||
       planning_mode_ == sampling_based_method::PlanningMode::ONLY_JOINTS_MODE ||
       motion_type_ == sampling_based_method::PlanningMode::SE3)
      {
#if 0
        /* special for the dragon kinematics */
        /* consider rootlink (link1) as baselink */
        KDL::Rotation q;
        tf::quaternionMsgToKDL(root_pose.orientation, q);
        robot_model_ptr_->setCogDesireOrientation(q);
        robot_model_ptr_->updateRobotModel(joint_state);
#endif
        if(!robot_model_ptr_->stabilityCheck()) return false;
      }

    if(planning_mode_ == sampling_based_method::PlanningMode::ONLY_JOINTS_MODE) return true;
    //check collision

    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    planning_scene_->setCurrentState(current_state.getRootJointStateConst<moveit_msgs::RobotState>());
    planning_scene_->checkCollision(collision_request, collision_result, planning_scene_->getCurrentState(), acm_);

    if(collision_result.collision) return false;

    return true;
  }

  /* temporary gap env */
  void MotionPlanning::setScene(const moveit_msgs::CollisionObject& collision_object)
  {
    if(planning_mode_ == sampling_based_method::PlanningMode::ONLY_JOINTS_MODE)
      {
        ROS_ERROR("Cannot set scene in ONLY_JOINTS_MODE");
        return;
      }
    planning_scene_->processCollisionObjectMsg(collision_object);
  }

  bool MotionPlanning::planInit()
  {
    /* reset */
    path_.clear();
    assert(path_.size() == 0);
    calculation_time_ = 0;
    min_var_ = 1e6;
    max_force_state_index_ = 0;
    min_force_state_index_ = 1e6;

    /* set root link as the baselink for the planning */
    robot_model_ptr_->setBaselinkName(std::string("link1"));

    /* planning */
    /* x, y */
    ompl::base::StateSpacePtr r_base;
    if(motion_type_ == sampling_based_method::PlanningMode::SE2)
      {
        r_base = std::shared_ptr<ompl::base::SE2StateSpace>(new ompl::base::SE2StateSpace());

        ompl::base::RealVectorBounds motion_bounds(2);
        motion_bounds.low[0] = position_lower_limits_.at(0);
        motion_bounds.low[1] = position_lower_limits_.at(1);
        motion_bounds.high[0] = position_upper_limits_.at(0);
        motion_bounds.high[1] = position_upper_limits_.at(1);
        r_base->as<ompl::base::SE2StateSpace>()->setBounds(motion_bounds);
      }

    if(motion_type_ == sampling_based_method::PlanningMode::SE3)
      {
        r_base = std::shared_ptr<ompl::base::SE3StateSpace>(new ompl::base::SE3StateSpace());

        ompl::base::RealVectorBounds motion_bounds(3);
        motion_bounds.low[0] = position_lower_limits_.at(0);
        motion_bounds.low[1] = position_lower_limits_.at(1);
        motion_bounds.low[2] = position_lower_limits_.at(2);
        motion_bounds.high[0] = position_upper_limits_.at(0);
        motion_bounds.high[1] = position_upper_limits_.at(1);
        motion_bounds.high[2] = position_upper_limits_.at(2);

        r_base->as<ompl::base::SE3StateSpace>()->setBounds(motion_bounds);
      }

    /* joints */
    ompl::base::StateSpacePtr r_joints(new ompl::base::RealVectorStateSpace(robot_model_ptr_->getLinkJointIndices().size()));
    ompl::base::RealVectorBounds joint_bounds(robot_model_ptr_->getLinkJointIndices().size());
    for(int j = 0; j < robot_model_ptr_->getLinkJointIndices().size(); j++ )
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
        for(int i = 0; i < robot_model_ptr_->getLinkJointIndices().size(); i ++)
          {
            start->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = start_state_.getJointStateConst()(robot_model_ptr_->getLinkJointIndices().at(i));
            goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = goal_state_.getJointStateConst()(robot_model_ptr_->getLinkJointIndices().at(i));
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

        for(int i = 0; i < robot_model_ptr_->getLinkJointIndices().size(); i ++)
          {
            start_tmp->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[i] = start_state_.getJointStateConst()(robot_model_ptr_->getLinkJointIndices().at(i));
            goal_tmp->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[i] = goal_state_.getJointStateConst()(robot_model_ptr_->getLinkJointIndices().at(i));
          }
      }

    pdef_ = ompl::base::ProblemDefinitionPtr(new ompl::base::ProblemDefinition(space_information_));
    pdef_->setStartAndGoalStates(start, goal);


    /* set the moveit setting in rviz */
    double start_time = ros::Time::now().toSec();
    while(planning_scene_diff_pub_.getNumSubscribers() < 1)
      {
        ros::WallDuration sleep_t(0.5);
        sleep_t.sleep();

        ROS_WARN_THROTTLE(1.0, "%s is not available, please set in RVIZ",
                          planning_scene_diff_pub_.getTopic().c_str());

        if(start_time - ros::Time::now().toSec() > 10.0)
          {
            ROS_ERROR("can not find %s in rivz, stop planing",
                      planning_scene_diff_pub_.getTopic().c_str());
            return false;
          }
      }
    ros::Duration sleep_time(1.0);
    sleep_time.sleep();

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    planning_scene_ = boost::shared_ptr<planning_scene::PlanningScene>(new planning_scene::PlanningScene(robot_model_loader.getModel()));
    acm_ = planning_scene_->getAllowedCollisionMatrix();

    planning_scene_->setCurrentState(start_state_.getRootJointStateConst<moveit_msgs::RobotState>());
    planning_scene_->getPlanningSceneMsg(planning_scene_msg_);
    planning_scene_msg_.is_diff = true;
    planning_scene_diff_pub_.publish(planning_scene_msg_);

    /* check collision*/
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    planning_scene_->checkCollision(collision_request, collision_result);

    if(collision_result.collision)
      {
        ROS_ERROR("collsion with init state, stop planning");
        return false;
      }

    return true;
  }

  bool MotionPlanning::plan()
  {
    if(headless_)
      {
        ROS_ERROR("sampling base method: please set headless to false, if do planning");
        return false;
      }

    if(!planInit()) return false;

    /* sampler */
    space_information_->setValidStateSamplerAllocator(boost::bind(&MotionPlanning::allocValidStateSampler, this, _1));

    /* optimation objective */
    ompl::base::OptimizationObjectivePtr length_obj(new ompl::base::PathLengthOptimizationObjective(space_information_));
    length_obj->setCostThreshold(onlyJointPathLimit());
    std::cout << "path length opt cost thre is "<<  length_obj->getCostThreshold() << std::endl;

    ompl::base::PlannerPtr planner;
    if(ompl_mode_ == RRT_START_MODE)
      {
        pdef_->setOptimizationObjective(length_obj);
        planner = ompl::base::PlannerPtr(new ompl::geometric::RRTstar(space_information_));
      }

    planner->setProblemDefinition(pdef_);
    planner->setup();
    space_information_->printSettings(std::cout);

    ros::Time start_time = ros::Time::now();

    if (planner->solve(solving_time_limit_))
      {
        calculation_time_ = ros::Time::now().toSec() - start_time.toSec();
        ompl::base::PathPtr ompl_result = pdef_->getSolutionPath();
        std::cout << "Found solution:" << std::endl;
        ompl_result->print(std::cout);

        if(ompl_mode_ == RRT_START_MODE)
          {
            std::cout << "iteration is "<< std::static_pointer_cast<ompl::geometric::RRTstar>(planner)->numIterations() << "best cost is " << std::static_pointer_cast<ompl::geometric::RRTstar>(planner)->bestCost()  << std::endl;
            std::stringstream ss;
            ss << std::static_pointer_cast<ompl::geometric::RRTstar>(planner)->bestCost();
            ss >> best_cost_;
          }

        ROS_ERROR("state count: %d", (int)(std::static_pointer_cast<ompl::geometric::PathGeometric>(ompl_result)->getStateCount()));
        for(int i = 1; i < (int)(std::static_pointer_cast<ompl::geometric::PathGeometric>(ompl_result)->getStateCount()); i++)
          {
            ompl::base::State *state1;
            ompl::base::State *state2;

            state1 = std::static_pointer_cast<ompl::geometric::PathGeometric>(ompl_result)->getState(i - 1);

            state2 = std::static_pointer_cast<ompl::geometric::PathGeometric>(ompl_result)->getState(i);

            addState(state1);
            int nd = config_space_->validSegmentCount(state1, state2);
            if (nd > 1)
              {
                ompl::base::State *interpolated_state = space_information_->allocState();
                for (int j = 1 ; j < nd ; ++j)
                  {
                    config_space_->interpolate(state1, state2, (double)j / (double)nd, interpolated_state);
                    addState(interpolated_state);

                  }
              }
            addState(state2);
          }

        /* log data */
        ROS_WARN("plan size is %d, planning time is %f, motion cost is %f, min var is %f, min var state index: %d", getPathSize(), getPlanningTime(), getMotionCost(), getMinVar(), getMinVarStateIndex());

        if(save_path_flag_) savePath();

        /* reset the baselink to the original one */
        robot_model_ptr_->setBaselinkName(baselink_name_);

        return true;
      }
    else
      {
        std::cout << "No solution found" << std::endl;
        return false;
      }
  }

  void MotionPlanning::rosParamInit()
  {
    nhp_.param("headless", headless_, true);
    nhp_.param("save_path_flag", save_path_flag_, true);
    nhp_.param("file_name", file_name_, std::string("planning_log.txt"));

    nhp_.param("x_low_bound", position_lower_limits_.at(0), -2.0);
    nhp_.param("x_high_bound", position_upper_limits_.at(0), 5.0);
    nhp_.param("y_low_bound", position_lower_limits_.at(1), -2.2);
    nhp_.param("y_high_bound", position_upper_limits_.at(1), 2.2);
    nhp_.param("z_low_bound", position_lower_limits_.at(2), -2.2);
    nhp_.param("z_high_bound", position_upper_limits_.at(2), 2.2);

    joint_lower_limits_.resize(robot_model_ptr_->getLinkJointNames().size());
    joint_upper_limits_.resize(robot_model_ptr_->getLinkJointNames().size());
    for(int i = 0; i < robot_model_ptr_->getLinkJointNames().size(); i++)
      {
        nhp_.param(robot_model_ptr_->getLinkJointNames().at(i) + std::string("_lower_limit"), joint_lower_limits_.at(i), robot_model_ptr_->getLinkJointLowerLimits().at(i));
        nhp_.param(robot_model_ptr_->getLinkJointNames().at(i) + std::string("_upper_limit"), joint_upper_limits_.at(i), robot_model_ptr_->getLinkJointUpperLimits().at(i));
      }

    nhp_.param("ompl_mode", ompl_mode_, 0); //RRT_START_MODE
    nhp_.param("planning_mode", planning_mode_, 0); //ONLY_JOINTS_MODE
    nhp_.param("motion_type", motion_type_, 0); //SE2

    geometry_msgs::Pose pose;
    nhp_.param("start_state_x", pose.position.x, 0.0);
    nhp_.param("start_state_y", pose.position.y, 0.5);
    double r,p,y;
    nhp_.param("start_state_roll", r, 0.0);
    nhp_.param("start_state_pitch", p, 0.0);
    nhp_.param("start_state_yaw", y, 0.0);
    pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(r,p,y);
    /* getTree().getNrOfJoints() = link_joint + gimbal + rotor */
    KDL::JntArray joint_state(robot_model_ptr_->getTree().getNrOfJoints());
    for(int i = 0; i < robot_model_ptr_->getLinkJointNames().size(); i++)
      nhp_.param(std::string("start_") + robot_model_ptr_->getLinkJointNames().at(i), joint_state(robot_model_ptr_->getLinkJointIndices().at(i)), 0.0);
    start_state_.setStatesFromRoot(robot_model_ptr_, pose, joint_state);

    nhp_.param("goal_state_x", pose.position.x, 0.0);
    nhp_.param("goal_state_y", pose.position.y, 0.5);
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
    for(int i = 0; i < robot_model_ptr_->getLinkJointNames().size(); i++)
      nhp_.param(std::string("goal_") + robot_model_ptr_->getLinkJointNames().at(i), joint_state(robot_model_ptr_->getLinkJointIndices().at(i)), 0.0);
    goal_state_.setStatesFromRoot(robot_model_ptr_, pose, joint_state);

    nhp_.param("file_state_offset_x", file_state_offset_x_, 0.0);
    nhp_.param("file_state_offset_y", file_state_offset_y_, 0.0);
    nhp_.param("file_state_offset_z", file_state_offset_z_, 0.0);

    nhp_.param("state_validity_check_res", state_validity_check_res_, 0.03);
    nhp_.param("valid_segment_count_factor", valid_segment_count_factor_,20);

    nhp_.param("solving_time_limit", solving_time_limit_, 3600.0);
    nhp_.param("length_cost_thre", length_cost_thre_, 0.0);
  }

  ompl::base::Cost MotionPlanning::onlyJointPathLimit()
  {
    if(planning_mode_ == sampling_based_method::PlanningMode::ONLY_JOINTS_MODE ||
       planning_mode_ == sampling_based_method::PlanningMode::JOINTS_AND_BASE_MODE)
      {
        if(planning_mode_ == sampling_based_method::PlanningMode::ONLY_JOINTS_MODE) length_cost_thre_ = 0.1;
        for(auto index: robot_model_ptr_->getLinkJointIndices())
          length_cost_thre_ += fabs(start_state_.getJointStateConst()(index) - goal_state_.getJointStateConst()(index));
      }
    return ompl::base::Cost(length_cost_thre_);
  }

  void MotionPlanning::addState(ompl::base::State *ompl_state)
  {
    geometry_msgs::Pose root_pose;
    KDL::JntArray joint_state(robot_model_ptr_->getJointIndexMap().size());

    if(planning_mode_ == sampling_based_method::PlanningMode::ONLY_JOINTS_MODE)
      {
        int index = 0;
        for(auto itr : robot_model_ptr_->getLinkJointIndices())
          joint_state(itr) = ompl_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[index++];
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
        for(auto itr : robot_model_ptr_->getLinkJointIndices())
          joint_state(itr) = state_tmp->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[index++];
      }
    else if(planning_mode_ == sampling_based_method::PlanningMode::ONLY_BASE_MODE)
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

    /* special  */
    /* consider rootlink (link1) as baselink */
    KDL::Rotation q;
    tf::quaternionMsgToKDL(root_pose.orientation, q);
    robot_model_ptr_->setCogDesireOrientation(q);
    robot_model_ptr_->updateRobotModel(joint_state);
    robot_model_ptr_->stabilityCheck();


    /* workaround for old method wich using roll/pitch position margin */
    auto hydrus_model_ptr = boost::dynamic_pointer_cast<HydrusRobotModel>(robot_model_ptr_);
    if(hydrus_model_ptr->getRollPitchPositionMargin() < min_var_)
      {
        min_var_ = hydrus_model_ptr->getRollPitchPositionMargin() ;
        min_var_state_index_ = path_.size();
      }


    if(robot_model_ptr_->getStaticThrust().maxCoeff() > max_force_)
      {
        max_force_ = robot_model_ptr_->getStaticThrust().maxCoeff();
        max_force_state_index_ = path_.size();
      }
    if(robot_model_ptr_->getStaticThrust().minCoeff() > min_force_)
      {
        min_force_ = robot_model_ptr_->getStaticThrust().minCoeff();
        min_force_state_index_ = path_.size();
      }

    /* log out */
    double r, p, y;
    tf::Quaternion tf_q;
    quaternionMsgToTF(root_pose.orientation, tf_q);
    tf::Matrix3x3(tf_q).getRPY(r, p, y);
    ROS_INFO("index: %d, dist_var: %f, max_force: %f, base pose: [%f, %f, %f] att: [%f, %f, %f]", (int)path_.size(), hydrus_model_ptr->getRollPitchPositionMargin(), robot_model_ptr_->getStaticThrust().maxCoeff(), root_pose.position.x, root_pose.position.y, root_pose.position.z, r, p, y);

    addState(MultilinkState(robot_model_ptr_, root_pose, joint_state));
  }

  void MotionPlanning::savePath()
  {
    std::ofstream ofs;
    ofs.open(file_name_);
    ofs << "start_state_: ";
    /* root pose */
    for (int i = 0; i < 7; i++)
      ofs << " " << start_state_.getRootJointStateConst<std::vector<double> >().at(i);
    /* joint state */
    for (auto index: robot_model_ptr_->getLinkJointIndices())
      ofs << " " << start_state_.getJointStateConst()(index);
    ofs << std::endl;
    ofs << "goal_state_: ";
    /* root pose */
    for (int i = 0; i < 7; i++)
      ofs << " " << goal_state_.getRootJointStateConst<std::vector<double> >().at(i);
    /* joint state */
    for (auto index: robot_model_ptr_->getLinkJointIndices())
      ofs << " " << goal_state_.getJointStateConst()(index);
    ofs << std::endl;

    ofs << "states: " << path_.size()  << std::endl;
    ofs << "planning_mode: " << planning_mode_ << std::endl;
    ofs << "planning_time: " << calculation_time_ << std::endl;
    ofs << "motion_cost: " << best_cost_ << std::endl;
    ofs << "minimum_var: " << min_var_ << std::endl;
    ofs << "minimum_var_state_entry: " << min_var_state_index_  << std::endl;

    for(auto k = 0; k < (int)path_.size();  k++)
      {
        ofs << "state" << k << ": ";
        /* root pose */
        for (int i = 0; i < 7; i++)
          ofs << " " << path_.at(k).getRootJointStateConst<std::vector<double> >().at(i);
        /* joint state */
        for (auto index: robot_model_ptr_->getLinkJointIndices())
          ofs << " " << path_.at(k).getJointStateConst()(index);

        for (int j = 0; j < robot_model_ptr_->getLinkJointIndices().size(); j++)

          ofs << std::endl;
      }
    ofs << "end"  << std::endl;
    ofs.close();
  }

  bool MotionPlanning::loadPath()
  {
    auto convertPoseFromVector = [](std::vector<double> state_vec)
      {
        assert(state_vec.size() == 7);
        geometry_msgs::Pose pose;
        pose.position.x = state_vec.at(0);
        pose.position.y = state_vec.at(1);
        pose.position.z = state_vec.at(2);
        pose.orientation.x = state_vec.at(3);
        pose.orientation.y = state_vec.at(4);
        pose.orientation.z = state_vec.at(5);
        pose.orientation.w = state_vec.at(6);
        return pose;
      };

    std::ifstream ifs(file_name_.c_str());

    if(ifs.fail())
      {
        ROS_ERROR("File do not exist");
        path_.clear();
        return false;
      }

    int state_list;
    std::stringstream ss[11];
    std::string str;
    std::string header;
    std::vector<double> root_pose_vec(7);
    KDL::JntArray joint_state(robot_model_ptr_->getJointIndexMap().size());

    //1 start and goal state
    std::getline(ifs, str);
    ss[0].str(str);
    ss[0] >> header;
    /* root pose */
    for (int i = 0; i < 7; i++) ss[0] >> root_pose_vec.at(i);
    /* joint state */
    for (auto index: robot_model_ptr_->getLinkJointIndices()) ss[0] >> joint_state(index);

    root_pose_vec.at(0) += file_state_offset_x_;
    root_pose_vec.at(1) += file_state_offset_y_;
    root_pose_vec.at(2) += file_state_offset_z_;
    start_state_.setStatesFromRoot(robot_model_ptr_, convertPoseFromVector(root_pose_vec), joint_state);

    std::getline(ifs, str);
    ss[1].str(str);
    ss[1] >> header;
    /* root pose */
    for (int i = 0; i < 7; i++) ss[0] >> root_pose_vec.at(i);
    /* joint state */
    for (auto index: robot_model_ptr_->getLinkJointIndices()) ss[0] >> joint_state(index);

    root_pose_vec.at(0) += file_state_offset_x_;
    root_pose_vec.at(1) += file_state_offset_y_;
    root_pose_vec.at(2) += file_state_offset_z_;
    goal_state_.setStatesFromRoot(robot_model_ptr_, convertPoseFromVector(root_pose_vec), joint_state);

    std::getline(ifs, str);
    ss[2].str(str);
    ss[2] >> header >> state_list;
    std::cout << header << state_list <<std::endl;
    std::getline(ifs, str);
    ss[3].str(str);
    ss[3] >> header >> planning_mode_;
    std::cout << header << planning_mode_ <<std::endl;
    std::getline(ifs, str);
    ss[4].str(str);
    ss[4] >> header >> calculation_time_;
    std::cout << header << calculation_time_ <<std::endl;
    std::getline(ifs, str);
    ss[5].str(str);
    ss[5] >> header >> best_cost_;
    std::cout << header << best_cost_ <<std::endl;
    std::getline(ifs, str);
    ss[6].str(str);
    ss[6] >> header >> min_var_;
    std::cout << header << min_var_ << std::endl;
    std::getline(ifs, str);
    ss[7].str(str);
    ss[7] >> header >> min_var_state_index_;
    std::cout << header << min_var_state_index_ <<std::endl;

    for(int k = 0; k < state_list;  k++)
      {
        std::stringstream ss_tmp;

        std::getline(ifs, str);
        ss_tmp.str(str);
        ss_tmp >> header;
        /* root pose */
        for (int i = 0; i < 7; i++) ss_tmp >> root_pose_vec.at(i);
        /* joint state */
        for (auto index: robot_model_ptr_->getLinkJointIndices()) ss_tmp >> joint_state(index);
        root_pose_vec.at(0) += file_state_offset_x_;
        root_pose_vec.at(1) += file_state_offset_y_;
        root_pose_vec.at(2) += file_state_offset_z_;

        path_.push_back(MultilinkState(robot_model_ptr_, convertPoseFromVector(root_pose_vec), joint_state));

        if(min_force_ > robot_model_ptr_->getStaticThrust().minCoeff())
          min_force_ = robot_model_ptr_->getStaticThrust().minCoeff();
        if(max_force_ < robot_model_ptr_->getStaticThrust().maxCoeff())
          max_force_ = robot_model_ptr_->getStaticThrust().maxCoeff();
      }

    ROS_WARN("plan size is %d, planning time is %f, motion cost is %f, min var is %f, min var state index: %d, min force: %f, max force: %f", getPathSize(), getPlanningTime(), getMotionCost(), min_var_, min_var_state_index_, min_force_, max_force_);

    return true;
  }

  bool MotionPlanning::checkCollision(const MultilinkState& state)
  {
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    planning_scene_->setCurrentState(state.getRootJointStateConst<moveit_msgs::RobotState>());
    planning_scene_->checkCollision(collision_request, collision_result); //TODO acm_
    if(collision_result.collision) ROS_WARN("Robot collision with env");
    return true;
  }
}
