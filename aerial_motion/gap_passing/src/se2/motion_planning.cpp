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

#include <gap_passing/se2/motion_planning.h>

namespace se2
{
  MotionPlanning::MotionPlanning(ros::NodeHandle nh, ros::NodeHandle nhp):
    nh_(nh), nhp_(nhp), solved_(false),
    path_(0), calculation_time_(0), best_cost_(-1), min_var_(1e6),
    planning_mode_(gap_passing::PlanningMode::ONLY_JOINTS_MODE)
  {
    /* ros pub/sub and service */
    planning_scene_diff_pub_ = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    keyposes_server_ = nh_.advertiseService("keyposes_server", &MotionPlanning::getKeyposes, this);
  }

  void MotionPlanning::baseInit()
  {
    robotInit();

    rosParamInit();

    if(load_path_flag_) loadPath();


    if(play_path_flag_)
      {
        sceneInit();

        if(!load_path_flag_) plan();

        if(realtime_path_flag_)
          {
            continous_path_sub_ = nh_.subscribe("/desired_state", 1, &MotionPlanning::continousPathCallback, this);
            real_state_scene_diff_pub_ = nh_.advertise<moveit_msgs::PlanningScene>("real_state_scene", 1);
            robot_cog_odom_sub_ = nh_.subscribe("/uav/cog/odom", 1, &MotionPlanning::robotOdomCallback, this);
            std::string topic_name;
            nhp_.param("joint_state_topic_name", topic_name, std::string("joint_states"));
            robot_joint_states_sub_ = nh_.subscribe(topic_name, 1, &MotionPlanning::robotJointStatesCallback, this);

            baselink_desired_att_.setRPY(0, 0, 0);
            baselink_desired_attitude_sub_ = nh_.subscribe("/desire_coordinate", 1, &MotionPlanning::desireCoordinateCallback, this);

            while(real_state_scene_diff_pub_.getNumSubscribers() < 1)
              {
                ros::WallDuration sleep_t(0.5);
                sleep_t.sleep();
              }
            ros::Duration sleep_time(1.0);
            sleep_time.sleep();

            robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
            real_state_scene_ = boost::shared_ptr<planning_scene::PlanningScene>(new planning_scene::PlanningScene(robot_model_loader.getModel()));

            solved_ = false;
          }

        motion_sequence_timer_ = nhp_.createTimer(ros::Duration(1.0 / motion_sequence_rate_), &MotionPlanning::motionSequenceFunc, this);
      }
  }

  void MotionPlanning::robotInit()
  {
    root_motion_dof_ = 3;
    transform_controller_ = boost::shared_ptr<TransformController>(new TransformController(nh_, nhp_, false));
    joint_num_ = transform_controller_->getRotorNum() - 1;

    start_state_.joint_states.resize(joint_num_);
    goal_state_.joint_states.resize(joint_num_);

    for(int i = 0; i < joint_num_; i++)
      {
        std::stringstream ss;
        ss << i + 1;
        start_state_.joint_names.push_back(std::string("joint") + ss.str());
      }
  }

  void MotionPlanning::motionSequenceFunc(const ros::TimerEvent &e)
  {
    static bool first_flag = true;
    static int state_index = 0;

    robot_state::RobotState& robot_state = planning_scene_->getCurrentStateNonConst();

    if(solved_ && ros::ok())
      {
        if(first_flag)
          {
            ROS_WARN("plan size is %d, planning time is %f, motion cost is %f, min var is %f, min var state index: %d", getPathSize(), getPlanningTime(), getMotionCost(), getMinVar(), getMinVarStateIndex());

            int min_var_state_index = getMinVarStateIndex();
            for(int i = 0; i < joint_num_; i++)
              std::cout << start_state_.joint_names.at(i) << ": " << getState(min_var_state_index).joint_states.at(i) << " ";
            std::cout << std::endl;

            first_flag = false;
          }

        robot_state::RobotState robot_state = setRobotState2Moveit(getState(state_index), planning_scene_);
        planning_scene_->getPlanningSceneMsg(planning_scene_msg_);
        planning_scene_msg_.is_diff = true;
        planning_scene_diff_pub_.publish(planning_scene_msg_);
        state_index ++;
        if(state_index ==  path_.size()) state_index = 0;
      }
  }

  bool MotionPlanning::getKeyposes(gap_passing::Keyposes::Request &req, gap_passing::Keyposes::Response &res)
  {
    int keyposes_num = path_.size();
    if (solved_ && ros::ok() && keyposes_num){
      res.available_flag = true;
      res.states_cnt = keyposes_num;
      res.motion_type = gap_passing::Keyposes::Response::SE2;
      res.dim = 6 + joint_num_;
      res.data.layout.dim.push_back(std_msgs::MultiArrayDimension());
      res.data.layout.dim.push_back(std_msgs::MultiArrayDimension());
      res.data.layout.dim[0].label = "height";
      res.data.layout.dim[1].label = "width";
      res.data.layout.dim[0].size = keyposes_num;
      res.data.layout.dim[1].size = 6 + joint_num_;
      res.data.layout.dim[0].stride = keyposes_num * (6 + joint_num_);
      res.data.layout.dim[1].stride = 6 + joint_num_;
      res.data.layout.data_offset = 0;

      for (int i = 0; i < keyposes_num; ++i){
        /* cog position */
        for (int j = 0; j < 3; ++j)
          res.data.data.push_back(path_.at(i).cog_state.at(j));

        /* convert to euler enagles */
        tf::Matrix3x3 att(tf::Quaternion(path_.at(i).cog_state.at(3),
                                         path_.at(i).cog_state.at(4),
                                         path_.at(i).cog_state.at(5),
                                         path_.at(i).cog_state.at(6)));
        double r, p, y; path_.at(i).getCogRPY(r, p, y);
        res.data.data.push_back(r);
        res.data.data.push_back(p);
        res.data.data.push_back(y);

        /* set joint state */
        for (int j = 0; j < joint_num_; ++j)
          res.data.data.push_back(path_.at(i).joint_states.at(j));
      }
    }
    else{
      res.available_flag = false;
      res.states_cnt = 0;
    }
    return true;
  }

  bool  MotionPlanning::isStateValid(const ompl::base::State *state)
  {
    State current_state = start_state_;

    double yaw = 0;
    if(planning_mode_ == gap_passing::PlanningMode::ONLY_JOINTS_MODE)
      {
        for(int i = 0; i < joint_num_; i++)
          current_state.joint_states.at(i) = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i];
      }
    else if(planning_mode_ == gap_passing::PlanningMode::ONLY_BASE_MODE)
      {
        current_state.root_state.at(0) = state->as<ompl::base::SE2StateSpace::StateType>()->getX();
        current_state.root_state.at(1) = state->as<ompl::base::SE2StateSpace::StateType>()->getY();
        yaw = state->as<ompl::base::SE2StateSpace::StateType>()->getYaw();
      }
    else if(planning_mode_ == gap_passing::PlanningMode::JOINTS_AND_BASE_MODE)
      {
        const ompl::base::CompoundState* state_tmp = dynamic_cast<const ompl::base::CompoundState*>(state);
        current_state.root_state.at(0) = state_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->getX();
        current_state.root_state.at(1) = state_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->getY();
        yaw = state_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->getYaw();
        for(int i = 0; i < joint_num_; i++)
          current_state.joint_states.at(i) = state_tmp->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[i];
      }

    tf::Quaternion q = tf::createQuaternionFromYaw(yaw);
    current_state.root_state.at(3) = q.x();
    current_state.root_state.at(4) = q.y();
    current_state.root_state.at(5) = q.z();
    current_state.root_state.at(6) = q.w();

    //check distance thresold
    if(planning_mode_ == gap_passing::PlanningMode::JOINTS_AND_BASE_MODE || planning_mode_ == gap_passing::PlanningMode::ONLY_JOINTS_MODE)
      {
        sensor_msgs::JointState joint_state;
        joint_state.name = start_state_.joint_names;
        joint_state.position = current_state.joint_states;
        transform_controller_->kinematics(joint_state);
        double dist_thre_check = transform_controller_->distThreCheck();

        if(dist_thre_check == 0) return false;
        if(!transform_controller_->modelling()) return false;

        if(planning_mode_ == gap_passing::PlanningMode::ONLY_JOINTS_MODE) return true;
      }

    //check collision
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;

    robot_state::RobotState robot_state = setRobotState2Moveit(current_state, planning_scene_);
    planning_scene_->checkCollision(collision_request, collision_result, robot_state, acm_);

    if(collision_result.collision) return false;

    return true;
  }

  void MotionPlanning::sceneInit()
  {
    while(planning_scene_diff_pub_.getNumSubscribers() < 1)
      {
        ros::WallDuration sleep_t(0.5);
        sleep_t.sleep();
      }
    ros::Duration sleep_time(1.0);
    sleep_time.sleep();

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    planning_scene_ = boost::shared_ptr<planning_scene::PlanningScene>(new planning_scene::PlanningScene(robot_model_loader.getModel()));
    acm_ = planning_scene_->getAllowedCollisionMatrix();

    gapEnvInit();

    //ROS_WARN("moveit robot state number: %d",  (int)planning_scene_->getCurrentStateNonConst().getVariableCount());

    robot_state::RobotState robot_state = setRobotState2Moveit(start_state_, planning_scene_);

    planning_scene_->getPlanningSceneMsg(planning_scene_msg_);
    planning_scene_msg_.is_diff = true;
    planning_scene_diff_pub_.publish(planning_scene_msg_);

    //check collision
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    planning_scene_->checkCollision(collision_request, collision_result, robot_state, acm_);

    if(collision_result.collision) ROS_ERROR("collsion with init state");
  }

  void MotionPlanning::gapEnvInit()
  {
    if(planning_mode_ == gap_passing::PlanningMode::ONLY_JOINTS_MODE) return;

    //pub the collision object => gap env
    left_half_corner = tf::Vector3(gap_left_x_, gap_left_y_ , gap_left_width_);
    right_half_corner = tf::Vector3(gap_left_x_ + gap_x_offset_, gap_left_y_ + gap_x_offset_ , gap_right_width_);

    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = "world";
    collision_object.id = "box";
    geometry_msgs::Pose wall_pose;
    wall_pose.orientation.w = 1.0;
    shape_msgs::SolidPrimitive wall_primitive;
    wall_primitive.type = wall_primitive.BOX;
    wall_primitive.dimensions.resize(3);

    wall_pose.position.x = left_half_corner.getX() + gap_left_width_ /2;
    wall_pose.position.y =  (2.5 + gap_y_offset_/2) /2;
    wall_primitive.dimensions[0] = gap_left_width_;
    wall_primitive.dimensions[1] = 2.5 - gap_y_offset_ / 2;
    wall_primitive.dimensions[2] = 1;
    collision_object.primitives.push_back(wall_primitive);
    collision_object.primitive_poses.push_back(wall_pose);

    wall_pose.position.x = right_half_corner.getX() + gap_right_width_ /2;
    wall_pose.position.y = - (2.5 + gap_y_offset_/2) /2;
    wall_pose.orientation.w = 1.0;
    wall_primitive.dimensions[0] = gap_right_width_;
    wall_primitive.dimensions[1] = 2.5 - gap_y_offset_ / 2;
    wall_primitive.dimensions[2] = 1;
    collision_object.primitives.push_back(wall_primitive);
    collision_object.primitive_poses.push_back(wall_pose);

    wall_pose.position.x = 1.0;
    wall_pose.position.y = 2.5;
    wall_pose.orientation.w = 1.0;
    wall_primitive.dimensions[0] = 8;
    wall_primitive.dimensions[1] = 0.6;
    wall_primitive.dimensions[2] = 1;
    collision_object.primitives.push_back(wall_primitive);
    collision_object.primitive_poses.push_back(wall_pose);

    wall_pose.position.x = 1.0;
    wall_pose.position.y = -2.5;
    wall_pose.orientation.w = 1.0;
    wall_primitive.dimensions[0] = 8;
    wall_primitive.dimensions[1] = 0.6;
    wall_primitive.dimensions[2] = 1;
    collision_object.primitives.push_back(wall_primitive);
    collision_object.primitive_poses.push_back(wall_pose);

    collision_object.operation = collision_object.ADD;
    planning_scene_->processCollisionObjectMsg(collision_object);
  }

  void MotionPlanning::plan()
  {
    planInit();

    //sampler
    space_information_->setValidStateSamplerAllocator(boost::bind(&MotionPlanning::allocValidStateSampler, this, _1));

    //optimation objective
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

        if(save_path_flag_) savePath();
        solved_ = true;
      }
    else
      std::cout << "No solution found" << std::endl;
  }

  void MotionPlanning::planInit()
  {
    //planning
    //x, y
    ompl::base::StateSpacePtr se2(new ompl::base::SE2StateSpace());
    ompl::base::RealVectorBounds motion_bounds(2);
    motion_bounds.low[0] = x_low_bound_;
    motion_bounds.low[1] = y_low_bound_;
    motion_bounds.high[0] = x_high_bound_;
    motion_bounds.high[1] = y_high_bound_;
    se2->as<ompl::base::SE2StateSpace>()->setBounds(motion_bounds);
    //joints
    ompl::base::StateSpacePtr r_joints(new ompl::base::RealVectorStateSpace(joint_num_));
    ompl::base::RealVectorBounds joint_bounds(joint_num_);
    joint_bounds.setLow(joint_low_bound_);
    joint_bounds.setHigh(joint_high_bound_);
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
        config_space_ = se2;
        config_space_->as<ompl::base::SE2StateSpace>()->setValidSegmentCountFactor(valid_segment_count_factor_);
        space_information_  = ompl::base::SpaceInformationPtr(new ompl::base::SpaceInformation(config_space_));
        space_information_->setup();
        space_information_->setStateValidityChecker(boost::bind(&MotionPlanning::isStateValid, this, _1));
        space_information_->setStateValidityCheckingResolution(state_validity_check_res_);
        space_information_->setMotionValidator(ompl::base::MotionValidatorPtr(new ompl::base::DiscreteMotionValidator(space_information_)));
        space_information_->setup();
      }
    else if(planning_mode_ == gap_passing::PlanningMode::JOINTS_AND_BASE_MODE)
      {
        config_space_ = se2 + r_joints;
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
        start->as<ompl::base::SE2StateSpace::StateType>()->setXY(start_state_.root_state.at(0), start_state_.root_state.at(1));
        start->as<ompl::base::SE2StateSpace::StateType>()->setYaw(tf::getYaw(tf::Quaternion(start_state_.root_state.at(3), start_state_.root_state.at(4), start_state_.root_state.at(5), start_state_.root_state.at(6))));
        goal->as<ompl::base::SE2StateSpace::StateType>()->setXY(goal_state_.root_state.at(0), goal_state_.root_state.at(1));
        goal->as<ompl::base::SE2StateSpace::StateType>()->setYaw(tf::getYaw(tf::Quaternion(goal_state_.root_state.at(3), goal_state_.root_state.at(4), goal_state_.root_state.at(5), goal_state_.root_state.at(6))));
      }
    else if(planning_mode_ == gap_passing::PlanningMode::JOINTS_AND_BASE_MODE)
      {
        ompl::base::CompoundState* start_tmp = dynamic_cast<ompl::base::CompoundState*> (start.get());
        start_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->setXY(start_state_.root_state.at(0), start_state_.root_state.at(1));
        start_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->setYaw(tf::getYaw(tf::Quaternion(start_state_.root_state.at(3), start_state_.root_state.at(4), start_state_.root_state.at(5), start_state_.root_state.at(6))));
        ompl::base::CompoundState* goal_tmp = dynamic_cast<ompl::base::CompoundState*> (goal.get());
        goal_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->setXY(goal_state_.root_state.at(0), goal_state_.root_state.at(1));
        goal_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->setYaw(tf::getYaw(tf::Quaternion(goal_state_.root_state.at(3), goal_state_.root_state.at(4), goal_state_.root_state.at(5), goal_state_.root_state.at(6))));
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
    nhp_.param("save_path_flag", save_path_flag_, true);
    nhp_.param("load_path_flag", load_path_flag_, false);
    nhp_.param("play_path_flag", play_path_flag_, false);
    nhp_.param("realtime_path_flag", realtime_path_flag_, false);
    nhp_.param("file_name", file_name_, std::string("planning_log.txt"));

    nhp_.param("gap_left_x", gap_left_x_, 1.0);
    nhp_.param("gap_left_y", gap_left_y_, 0.3);
    nhp_.param("gap_x_offset", gap_x_offset_, 0.6); //minus: overlap
    nhp_.param("gap_y_offset", gap_y_offset_, 0.0); //minus: overlap
    nhp_.param("gap_left_width", gap_left_width_, 0.3); //minus: bandwidth
    nhp_.param("gap_right_width", gap_right_width_, 0.3); //minus: bandwidth

    nhp_.param("x_low_bound", x_low_bound_, -2.0);
    nhp_.param("x_high_bound", x_high_bound_, 5.0);
    nhp_.param("y_low_bound", y_low_bound_, -2.2);
    nhp_.param("y_high_bound", y_high_bound_, 2.2);
    nhp_.param("joint_low_bound", joint_low_bound_, -1.58);
    nhp_.param("joint_high_bound", joint_high_bound_, 1.58);

    nhp_.param("ompl_mode", ompl_mode_, 0); //RRT_START_MODE
    nhp_.param("planning_mode", planning_mode_, 0); //ONLY_JOINTS_MODE
    nhp_.param("motion_sequence_rate", motion_sequence_rate_, 10.0);

    nhp_.param("baselink", base_link_, std::string("link1"));
    ROS_ERROR("motion planning: %s", base_link_.c_str());

    nhp_.param("start_state_x", start_state_.root_state.at(0), 0.0);
    nhp_.param("start_state_y", start_state_.root_state.at(1), 0.5);
    double yaw;
    nhp_.param("start_state_theta", yaw, 0.785);
    start_state_.setRootRPY(0, 0, yaw);

    nhp_.param("goal_state_x", goal_state_.root_state.at(0), 0.0);
    nhp_.param("goal_state_y", goal_state_.root_state.at(1), 0.5);
    nhp_.param("goal_state_theta", yaw, 0.785);
    goal_state_.setRootRPY(0, 0, yaw);

    for(int i = 0; i < joint_num_; i++)
      {
        std::stringstream joint_no;
        joint_no << i + 1;

        nhp_.param(std::string("start_state_joint") + joint_no.str(), start_state_.joint_states.at(i), 0.0);
        nhp_.param(std::string("goal_state_joint") + joint_no.str(), goal_state_.joint_states.at(i), 0.0);
      }
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
    if(planning_mode_ == gap_passing::PlanningMode::ONLY_JOINTS_MODE ||
       planning_mode_ == gap_passing::PlanningMode::JOINTS_AND_BASE_MODE)
      {
        if(planning_mode_ == gap_passing::PlanningMode::ONLY_JOINTS_MODE) length_cost_thre_ = 0.1;
        for(int i = 0; i < joint_num_; i ++)
          length_cost_thre_ += fabs(start_state_.joint_states.at(i) - goal_state_.joint_states.at(i));
      }
    return ompl::base::Cost(length_cost_thre_);
  }

  void MotionPlanning::addState(ompl::base::State *ompl_state)
  {
    State new_state = start_state_;

    if(planning_mode_ == gap_passing::PlanningMode::ONLY_JOINTS_MODE || planning_mode_ == gap_passing::PlanningMode::JOINTS_AND_BASE_MODE)
      {
        if(planning_mode_ == gap_passing::PlanningMode::ONLY_JOINTS_MODE)
          {
            for(int j = 0; j < joint_num_; j++)
              new_state.joint_states.at(j) = ompl_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[j];
          }
        else if(planning_mode_ == gap_passing::PlanningMode::JOINTS_AND_BASE_MODE)
          {
            const ompl::base::CompoundState* state_tmp = dynamic_cast<const ompl::base::CompoundState*>(ompl_state);
            new_state.root_state.at(0) = state_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->getX();
            new_state.root_state.at(1) = state_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->getY();
            tf::Quaternion q = tf::createQuaternionFromYaw(state_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->getYaw());
            new_state.root_state.at(3) = q.x();
            new_state.root_state.at(4) = q.y();
            new_state.root_state.at(5) = q.z();
            new_state.root_state.at(6) = q.w();
            for(int j = 0; j < joint_num_; j++)
              new_state.joint_states.at(j) = state_tmp->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[j];
          }

        // dist thre
        sensor_msgs::JointState joint_state;
        for(int j = 0; j < joint_num_; j++)
          {
            std::stringstream ss;
            ss << j + 1;
            joint_state.name.push_back(std::string("joint") + ss.str());
            joint_state.position.push_back(new_state.joint_states.at(j));
          }

        transform_controller_->kinematics(joint_state);
        float min_var = transform_controller_->distThreCheck();

        if(min_var < min_var_)
          {
            min_var_ = min_var;
            min_var_state_ = path_.size();
          }
      }
    else if(planning_mode_ == gap_passing::PlanningMode::ONLY_BASE_MODE)
      {
        new_state.root_state.at(0) = ompl_state->as<ompl::base::SE2StateSpace::StateType>()->getX();
        new_state.root_state.at(1) = ompl_state->as<ompl::base::SE2StateSpace::StateType>()->getY();
        tf::Quaternion q = tf::createQuaternionFromYaw(ompl_state->as<ompl::base::SE2StateSpace::StateType>()->getYaw());
        new_state.root_state.at(3) = q.x();
        new_state.root_state.at(4) = q.y();
        new_state.root_state.at(5) = q.z();
        new_state.root_state.at(6) = q.w();
      }

    addState(new_state);
  }

  void MotionPlanning::savePath()
  {
    std::ofstream ofs;
    ofs.open(file_name_);
    ofs << "start_state_: ";
    for (auto it = start_state_.root_state.begin(); it != start_state_.root_state.end(); it++)
      ofs << " " << *it;
    for (auto it = start_state_.joint_states.begin(); it != start_state_.joint_states.end(); it++)
      ofs << " " << *it;
    ofs << std::endl;
    ofs << "goal_state_: ";
    for (auto it = goal_state_.root_state.begin(); it != goal_state_.root_state.end(); it++)
      ofs << " " << *it;
    for (auto it = goal_state_.joint_states.begin(); it != goal_state_.joint_states.end(); it++)
      ofs << " " << *it;
    ofs << std::endl;

    ofs << "states: " << path_.size()  << std::endl;
    ofs << "planning_mode: " << planning_mode_ << std::endl;
    ofs << "planning_time: " << calculation_time_ << std::endl;
    ofs << "motion_cost: " << best_cost_ << std::endl;
    ofs << "minimum_var: " << min_var_ << std::endl;
    ofs << "minimum_var_state_entry: " << min_var_state_  << std::endl;

    for(int k = 0; k < (int)path_.size();  k++)
      {
        ofs << "state" << k << ": ";
        for (auto it = path_.at(k).root_state.begin(); it != path_.at(k).root_state.end(); it++)
          ofs << " " << *it;
        for (auto it = path_.at(k).joint_states.begin(); it != path_.at(k).joint_states.end(); it++)
          ofs << " " << *it;
        ofs << std::endl;
      }
    ofs << "end"  << std::endl;
    ofs.close();
  }

  void MotionPlanning::loadPath()
  {
    std::ifstream ifs(file_name_.c_str());

    if(ifs.fail())
      {
        ROS_ERROR("File do not exist");
        return;
      }

    int state_list;
    std::stringstream ss[11];
    std::string str;
    std::string header;
    //1 start and goal state
    std::getline(ifs, str);
    ss[0].str(str);
    ss[0] >> header;
    for (int i = 0; i < 7; i ++)
      ss[0] >> start_state_.root_state.at(i);
    for (int i = 0; i < start_state_.joint_states.size(); i++)
      ss[0] >> start_state_.joint_states.at(i);

    start_state_.root_state.at(0) += file_state_offset_x_;
    start_state_.root_state.at(1) += file_state_offset_y_;
    start_state_.root_state.at(2) += file_state_offset_z_;

    std::getline(ifs, str);
    ss[1].str(str);
    ss[1] >> header;
    for (int i = 0; i < 7; i ++)
      ss[1] >> goal_state_.root_state.at(i);
    for (int i = 0; i < goal_state_.joint_states.size(); i++)
      ss[1] >> goal_state_.joint_states.at(i);

    goal_state_.root_state.at(0) += file_state_offset_x_;
    goal_state_.root_state.at(1) += file_state_offset_y_;
    goal_state_.root_state.at(2) += file_state_offset_z_;

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
    ss[7] >> header >> min_var_state_;
    std::cout << header << min_var_state_  <<std::endl;

    for(int k = 0; k < state_list;  k++)
      {
        std::stringstream ss_tmp;
        std::vector<double> state_vec(7 + start_state_.joint_states.size());

        std::getline(ifs, str);
        ss_tmp.str(str);
        ss_tmp >> header;
        for (int i = 0; i < state_vec.size(); i++) ss_tmp >> state_vec.at(i);

        state_vec.at(0) += file_state_offset_x_;
        state_vec.at(1) += file_state_offset_y_;
        state_vec.at(2) += file_state_offset_z_;
        path_.push_back(root2cog(state_vec));
      }
    solved_ = true;
  }

  /* keypose size: 7 + joint_state_size */
  State MotionPlanning::root2cog(const std::vector<double> &keypose)
  {
    State state = start_state_;

    sensor_msgs::JointState joint_state;
    joint_state.name = start_state_.joint_names;
    for(int i = 0; i < joint_state.name.size(); i++)
      state.joint_states.at(i) = (keypose.at(7 + i));
    joint_state.position = state.joint_states;

    transform_controller_->kinematics(joint_state);
    tf::Transform cog_root = transform_controller_->getCog();
    tf::Transform root_world;

    for(int i = 0; i < 7; i++)
        state.root_state.at(i) = keypose.at(i);

    root_world.setOrigin(tf::Vector3(keypose[0], keypose[1], keypose[2]));
    root_world.setRotation(tf::Quaternion(keypose[3], keypose[4], keypose[5], keypose[6]));
    tf::Transform cog_world = root_world * cog_root;

    std::vector<double> keypose_cog;
    state.cog_state.at(0) = root_world.getOrigin().getX();
    state.cog_state.at(1) = root_world.getOrigin().getY();
    state.cog_state.at(2) = root_world.getOrigin().getZ();

    tf::Quaternion q = cog_world.getRotation();
    state.cog_state.at(3) = q.x();
    state.cog_state.at(4) = q.y();
    state.cog_state.at(5) = q.z();
    state.cog_state.at(6) = q.w();

    return state;
  }

  /* keypose size: 7 + joint_state_size */
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
    for(int i = 0; i < joint_state.name.size(); i++)
      state.joint_states.at(i) = keypose.at(7 + i);
    joint_state.position = state.joint_states;

    KDL::Rotation kdl_q;
    tf::quaternionTFToKDL(baselink_desired_att_, kdl_q);
    transform_controller_->setCogDesireOrientation(kdl_q);
    transform_controller_->kinematics(joint_state);
    tf::Transform cog_root = transform_controller_->getCog(); // cog in root frame

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

  robot_state::RobotState MotionPlanning::setRobotState2Moveit(State state, boost::shared_ptr<planning_scene::PlanningScene> planning_scene)
  {
    robot_state::RobotState& robot_state = planning_scene->getCurrentStateNonConst();
    robot_state.setVariablePosition(std::string("base/trans_x"), state.root_state.at(0));
    robot_state.setVariablePosition(std::string("base/trans_y"), state.root_state.at(1));
    robot_state.setVariablePosition(std::string("base/trans_z"), state.root_state.at(2));
    robot_state.setVariablePosition(std::string("base/rot_x"),   state.root_state.at(3));
    robot_state.setVariablePosition(std::string("base/rot_y"),   state.root_state.at(4));
    robot_state.setVariablePosition(std::string("base/rot_z"),   state.root_state.at(5));
    robot_state.setVariablePosition(std::string("base/rot_w"),   state.root_state.at(6));

    for(int i = 0; i < start_state_.joint_names.size(); i++)
      robot_state.setVariablePosition(state.joint_names.at(i), state.joint_states.at(i));

    return robot_state;
  }

  void MotionPlanning::robotOdomCallback(const nav_msgs::OdometryConstPtr& msg)
  {
    robot_cog_odom_ = *msg;
  }

  void MotionPlanning::robotJointStatesCallback(const sensor_msgs::JointStateConstPtr& joints_msg)
  {
    std::vector<double> real_robot_state(7 + start_state_.joint_names.size(), 0);

    /* cog, pos, attitude */
    real_robot_state.at(0) = robot_cog_odom_.pose.pose.position.x;
    real_robot_state.at(1) = robot_cog_odom_.pose.pose.position.y;
    real_robot_state.at(2) = robot_cog_odom_.pose.pose.position.z;
    real_robot_state.at(3) = robot_cog_odom_.pose.pose.orientation.x;
    real_robot_state.at(4) = robot_cog_odom_.pose.pose.orientation.y;
    real_robot_state.at(5) = robot_cog_odom_.pose.pose.orientation.z;
    real_robot_state.at(6) = robot_cog_odom_.pose.pose.orientation.w;

    /* joint state */
    if(start_state_.joint_names.size() != joints_msg->name.size())
      {
        ROS_ERROR("size of joint is not equal: rosmsg: %d, file: %d", (int)joints_msg->name.size(), (int)start_state_.joint_names.size());
        return;
      }

    for (int i = 0; i < joints_msg->name.size(); i++)
      {
        auto itr = std::find(start_state_.joint_names.begin(), start_state_.joint_names.end(), joints_msg->name.at(i));
        if (itr == start_state_.joint_names.end())
          {
            ROS_ERROR("can not find the joint name: %s", joints_msg->name.at(i).c_str());
            return;
          }

        size_t index = std::distance( start_state_.joint_names.begin(), itr );
        real_robot_state.push_back(joints_msg->position[i]);
      }

    robot_state::RobotState robot_state = setRobotState2Moveit(cog2root(real_robot_state), real_state_scene_);
    real_state_scene_->getPlanningSceneMsg(real_state_scene_msg_);
    real_state_scene_msg_.is_diff = true;
    real_state_scene_diff_pub_.publish(real_state_scene_msg_);

    /* check collision */
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    planning_scene_->checkCollision(collision_request, collision_result, robot_state, acm_);

    if(collision_result.collision) ROS_ERROR("Robot collision with env");
  }

  void MotionPlanning::continousPathCallback(const std_msgs::Float64MultiArrayConstPtr& msg)
  {
    assert(msg->data.size() == 6 + joint_num_);

    std::vector<double> state_vec(7 + start_state_.joint_names.size(), 0);

    /* convert from euler to quaternion */
    for(int i = 0; i < 3; i++) state_vec.at(i) = msg->data.at(i);
    tf::Quaternion q; q.setRPY(msg->data.at(3), msg->data.at(4), msg->data.at(5));
    state_vec.at(3) = q.x();
    state_vec.at(4) = q.y();
    state_vec.at(5) = q.z();
    state_vec.at(6) = q.w();
    /* joint state: correct order */
    for(int i = 0; i < joint_num_; i++) state_vec.at(7 + i) = msg->data.at(6 + i);

    setRobotState2Moveit(cog2root(state_vec), planning_scene_);
    planning_scene_->getPlanningSceneMsg(planning_scene_msg_);
    planning_scene_msg_.is_diff = true;
    planning_scene_diff_pub_.publish(planning_scene_msg_);
  }

  void MotionPlanning::desireCoordinateCallback(const aerial_robot_base::DesireCoordConstPtr & msg)
  {
    baselink_desired_att_.setRPY(msg->roll, msg->pitch, msg->yaw);
  }

}
