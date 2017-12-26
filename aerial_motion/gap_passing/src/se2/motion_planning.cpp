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

#include <gap_passing/se2/motion_control.h>
#include <gap_passing/se2/motion_planning.h>

/* the StabilityObjective does not work!!!!! */
namespace se2
{
  MotionPlanning::MotionPlanning(ros::NodeHandle nh, ros::NodeHandle nhp) :nh_(nh), nhp_(nhp)
  {
    transform_controller_ = boost::shared_ptr<TransformController>(new TransformController(nh_, nhp_, false));
    joint_num_ = transform_controller_->getRotorNum() - 1;
    move_start_flag_ = false;

    rosParamInit();

    motion_control_ = new MotionControl(nh, nhp, transform_controller_);

    planning_scene_diff_pub_ = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

    keyposes_server_ = nh_.advertiseService("keyposes_server", &MotionPlanning::getKeyposes, this);
    endposes_client_ = nh_.serviceClient<gap_passing::Endposes>("endposes_server");
    robot_move_start_sub_ = nh_.subscribe<std_msgs::Empty>("/move_start", 1, &MotionPlanning::moveStartCallback, this);
    desired_state_sub_ = nh_.subscribe<std_msgs::Float64MultiArray>("/desired_state", 1, &MotionPlanning::desiredStateCallback, this);

    /* experiment data replay */
    experiment_scene_diff_pub_ = nh_.advertise<moveit_msgs::PlanningScene>("experiment_scene", 1);
    experiment_robot_cog_odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/uav/cog/odom", 1, &MotionPlanning::experimentRobotOdomCallback, this);
    experiment_robot_joint_states_sub_ = nh_.subscribe<sensor_msgs::JointState>("/hydrusx/joint_states", 1, &MotionPlanning::experimentRobotJointStatesCallback, this);

    // if play log path, start/goal state is already read when initalizing motion_control_
    if (play_log_path_){
      start_state_ = motion_control_->start_state_;
      goal_state_ = motion_control_->goal_state_;
    }
    // if plan path online, start/goal state is requested from ros service
    else{
      gap_passing::Endposes endposes_srv;
      endposes_srv.request.inquiry = true;
      ROS_INFO("Waiting for endposes from service.");
      while (!endposes_client_.call(endposes_srv)){
        // wait for endposes
      }
      ROS_INFO("Get endposes from serivce.");
      std::cout << "Start state: ";
      for (int i = 0; i < 3 + joint_num_; ++i)
        std::cout << endposes_srv.response.start_pose.data[i] << ", ";
      std::cout << "\nEnd state: ";
      for (int i = 0; i < 3 + joint_num_; ++i)
        std::cout << endposes_srv.response.end_pose.data[i] << ", ";
      std::cout << "\n";
      start_state_ = cog2root(endposes_srv.response.start_pose.data);
      goal_state_ = cog2root(endposes_srv.response.end_pose.data);
    }

    // initalization desired state variable, since desired state is not received until robot is really moving (later than receive move start topic)
    for (int i = 0; i < 3 + joint_num_; ++i)
      deisred_state_.push_back(start_state_[i]);

    if(simulator_ && planning_mode_ != gap_passing::PlanningMode::ONLY_JOINTS_MODE)
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
        planning_scene_ = new planning_scene::PlanningScene(kinematic_model_);
        /* experiment data replay */
        experiment_scene_ = new planning_scene::PlanningScene(kinematic_model_);
        tolerance_ = 0.01;
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
        /* experiment data replay */
        delete experiment_scene_;
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
    sensor_msgs::JointState joint_state;

    if(solved_ && ros::ok())
      {
        if(first_flag)
          {
            ROS_WARN("plan size is %d, planning time is %f, motion cost is %f", motion_control_->getPathSize(), motion_control_->getPlanningTime(), motion_control_->getMotionCost());
            float min_var = motion_control_->getMinVar();
            std::cout << "min_var is: "<< min_var;
            int min_var_state_index = motion_control_->getMinVarStateIndex();
            for(int i = 0; i < joint_num_; i++)
              std::cout << " joint" << i+1 << ":" << (motion_control_->getState(min_var_state_index)).state_values[3 + i];
            std::cout << std::endl;

            ROS_WARN("semi stable states %d, ratio: %f", motion_control_->getSemiStableStates(),
                     (float)motion_control_->getSemiStableStates() / motion_control_->getPathSize());

            planning_path.resize(0);
            motion_control_->getPlanningPath(planning_path);

            if(planning_mode_ == gap_passing::PlanningMode::JOINTS_AND_BASE_MODE){
              keyposes_cog_vec_.resize(0);
              for (int i = 0; i < motion_control_->getPathSize(); ++i){
                std::vector<double> keypose_root; // keypose in root frame
                for (int j = 0; j < 3 + joint_num_; ++j)
                  keypose_root.push_back((motion_control_->getState(i)).state_values[j]);
                std::vector<double> keypose_cog = root2cog(keypose_root);
                keyposes_cog_vec_.push_back(keypose_cog);
              }
            }

            first_flag = false;
          }
        else if(simulator_)
          {
            if(planning_mode_ == gap_passing::PlanningMode::ONLY_JOINTS_MODE)
              {
                sensor_msgs::JointState joint_state_msg;
                joint_state_msg.header.stamp = ros::Time::now();
                if(planning_path[state_index].state_values.size() != 3 + joint_num_)
                  ROS_ERROR("wrong size of planning joint state and this state");
                joint_state_msg.position.resize(0);
                joint_state_msg.name.resize(0);
                for(int i = 0; i < joint_num_; i ++)
                  {
                    std::stringstream joint_no2;
                    joint_no2 << i + 1;
                    joint_state_msg.name.push_back(std::string("joint") + joint_no2.str());
                    joint_state_msg.position.push_back(planning_path[state_index].state_values[3 + i]);
                  }
                motion_control_->joint_cmd_pub_.publish(joint_state_msg);
              }
            else
              {
                robot_state::RobotState& robot_state = planning_scene_->getCurrentStateNonConst();

                std::vector<double> ex_curr_state(3 + joint_num_ + 7); //se(2) + joint_num + 7
                if (move_start_flag_){
                  std::vector<double> ex_root_state = cog2root(deisred_state_);
                  for (int i = 0; i < 3 + joint_num_; ++i)
                    ex_curr_state[i] = ex_root_state[i];
                  robot_state.setVariablePositions(ex_curr_state);
                }
                else{
                  ex_curr_state[0] = (motion_control_->getState(state_index)).state_values[0]; //x
                  ex_curr_state[1] = (motion_control_->getState(state_index)).state_values[1]; //y
                  ex_curr_state[2] = (motion_control_->getState(state_index)).state_values[2]; //yaw
                  for(int index = 0 ; index < joint_num_; index++)
                    ex_curr_state[index + 3] = (motion_control_->getState(state_index)).state_values[index + 3];
                  robot_state.setVariablePositions(ex_curr_state);
                }

                planning_scene_->getPlanningSceneMsg(planning_scene_msg_);
                planning_scene_msg_.is_diff = true;
                planning_scene_diff_pub_.publish(planning_scene_msg_);

                /* experiment data replay */
                if (replay_experiment_data_flag_){
                  robot_state::RobotState& replay_robot_state = experiment_scene_->getCurrentStateNonConst();
                  std::vector<double> replay_curr_state(3 + joint_num_ + 7); //se(2) + joint_num + 7
                  if (move_start_flag_){
                    std::vector<double> replay_cog_state;
                    replay_cog_state.push_back(experiment_robot_cog_odom_.pose.pose.position.x);
                    replay_cog_state.push_back(experiment_robot_cog_odom_.pose.pose.position.y);
                    // yaw
                    tf::Quaternion q(experiment_robot_cog_odom_.pose.pose.orientation.x,
                                     experiment_robot_cog_odom_.pose.pose.orientation.y,
                                     experiment_robot_cog_odom_.pose.pose.orientation.z,
                                     experiment_robot_cog_odom_.pose.pose.orientation.w);
                    tf::Matrix3x3 rot_mat;
                    rot_mat.setRotation(q);
                    tfScalar r,p,y;
                    rot_mat.getRPY(r, p, y);
                    replay_cog_state.push_back(y);
                    for (int i = 0; i < joint_num_; ++i) // joints
                      replay_cog_state.push_back(experiment_robot_joint_states_.position[i]);

                    std::vector<double> replay_root_state = cog2root(replay_cog_state);
                    for (int i = 0; i < 3 + joint_num_; ++i)
                      replay_curr_state[i] = replay_root_state[i];
                    replay_robot_state.setVariablePositions(replay_curr_state);

                    experiment_scene_->getPlanningSceneMsg(experiment_scene_msg_);
                    experiment_scene_msg_.is_diff = true;
                    experiment_scene_diff_pub_.publish(experiment_scene_msg_);
                  }

                }
              }

            state_index ++;
            if(state_index ==  motion_control_->getPathSize()) state_index = 0;
          }

      }
  }

  bool MotionPlanning::getKeyposes(gap_passing::Keyposes::Request &req, gap_passing::Keyposes::Response &res)
  {
    int keyposes_num = keyposes_cog_vec_.size();
    if (solved_ && ros::ok() && keyposes_num){
      res.available_flag = true;
      res.states_cnt = keyposes_num;
      res.dim = 3 + joint_num_;
      res.data.layout.dim.push_back(std_msgs::MultiArrayDimension());
      res.data.layout.dim.push_back(std_msgs::MultiArrayDimension());
      res.data.layout.dim[0].label = "height";
      res.data.layout.dim[1].label = "width";
      res.data.layout.dim[0].size = keyposes_num;
      res.data.layout.dim[1].size = 3 + joint_num_;
      res.data.layout.dim[0].stride = keyposes_num * (3 + joint_num_);
      res.data.layout.dim[1].stride = 3 + joint_num_;
      res.data.layout.data_offset = 0;

      for (int i = 0; i < keyposes_num; ++i){
        for (int j = 0; j < 3 + joint_num_; ++j)
          res.data.data.push_back(keyposes_cog_vec_[i][j]);
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

    std::vector<double> current_state(3 + joint_num_, 0);

    if(planning_mode_ == gap_passing::PlanningMode::ONLY_JOINTS_MODE)
      {
        for(int i = 0; i < joint_num_; i++)
          current_state[3 + i] = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i];
      }
    else if(planning_mode_ == gap_passing::PlanningMode::ONLY_BASE_MODE || planning_mode_ == gap_passing::PlanningMode::ONLY_BASE_MODE + gap_passing::PlanningMode::ORIGINAL_MODE)
      {
        current_state[0] = state->as<ompl::base::SE2StateSpace::StateType>()->getX();
        current_state[1] = state->as<ompl::base::SE2StateSpace::StateType>()->getY();
        current_state[2] = state->as<ompl::base::SE2StateSpace::StateType>()->getYaw();

        for(int i = 0; i < joint_num_; i++)
          current_state[i + 3] = start_state_[3 + i];

      }
    else if(planning_mode_ == gap_passing::PlanningMode::JOINTS_AND_BASE_MODE)
      {
        const ompl::base::CompoundState* state_tmp = dynamic_cast<const ompl::base::CompoundState*>(state);

        current_state[0] = state_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->getX();
        current_state[1] = state_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->getY();
        current_state[2] = state_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->getYaw();
        for(int i = 0; i < joint_num_; i++)
          current_state[i + 3] = state_tmp->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[i];
      }

    //check distance thresold
    if(planning_mode_ == gap_passing::PlanningMode::JOINTS_AND_BASE_MODE || planning_mode_ == gap_passing::PlanningMode::ONLY_JOINTS_MODE)
      {
        if(stability_opt_weight_ == 0)
          {//only path length opt, we have to judge the form whethe valid or not
            sensor_msgs::JointState joint_state;
            for(int i = 0; i < joint_num_; i++)
              {
                std::stringstream ss;
                ss << i + 1;
                joint_state.name.push_back(std::string("joint") + ss.str());
                joint_state.position.push_back(current_state[i + 3]);
              }
            transform_controller_->kinematics(joint_state);
            double dist_thre_check = transform_controller_->distThreCheck();

            if(dist_thre_check == 0) return false;
            if(!transform_controller_->modelling()) return false;
          }

        if(planning_mode_ == gap_passing::PlanningMode::ONLY_JOINTS_MODE) return true;
      }

    robot_state::RobotState& robot_state = planning_scene_->getCurrentStateNonConst();

    //check collision
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;

    std::vector<double> ex_curr_state(3 + joint_num_ + 7); //se(2) + joint_num + 7(?)
    ex_curr_state[0] = current_state[0]; //x
    ex_curr_state[1] = current_state[1]; //y
    ex_curr_state[2] = current_state[2]; //yaw
    for(int index = 0 ; index < joint_num_; index++)
      ex_curr_state[index + 3] = current_state[index + 3];

    robot_state.setVariablePositions(ex_curr_state);
    planning_scene_->checkCollision(collision_request, collision_result, robot_state, acm_);

    if(collision_result.collision) return false;

    return true;
  }

  std::vector<double> MotionPlanning::cog2root(std::vector<double> &keypose)
  {
    sensor_msgs::JointState joint_state;
    for(int i = 0; i < joint_num_; i++)
      {
        std::stringstream ss;
        ss << i + 1;
        joint_state.name.push_back(std::string("joint") + ss.str());
      }
    joint_state.position.resize(0);
    for (int j = 0; j < joint_num_; ++j)
      joint_state.position.push_back(keypose[3 + j]);
    transform_controller_->kinematics(joint_state);
    tf::Transform cog_root = transform_controller_->getCog(); // cog in root frame
    tf::Transform root_cog = cog_root.inverse();
    tf::Transform cog_world;
    cog_world.setOrigin(tf::Vector3(keypose[0],
                                    keypose[1],
                                    0.0));
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, keypose[2]);
    cog_world.setRotation(q);
    tf::Transform root_world = cog_world * root_cog;

    std::vector<double> keypose_root;
    keypose_root.push_back(root_world.getOrigin().getX());
    keypose_root.push_back(root_world.getOrigin().getY());
    q = root_world.getRotation();
    tf::Matrix3x3  rot_mat(q);
    tfScalar e_r, e_p, e_y;
    rot_mat.getRPY(e_r, e_p, e_y);
    keypose_root.push_back(e_y);
    for (int i = 0; i < joint_num_; i++)
      keypose_root.push_back(keypose[3 + i]);
    return keypose_root;
  }

  std::vector<double> MotionPlanning::root2cog(std::vector<double> &keypose)
  {
    sensor_msgs::JointState joint_state;
    for(int i = 0; i < joint_num_; i++)
      {
        std::stringstream ss;
        ss << i + 1;
        joint_state.name.push_back(std::string("joint") + ss.str());
      }
    joint_state.position.resize(0);
    for (int j = 0; j < joint_num_; ++j)
      joint_state.position.push_back(keypose[3 + j]);
    transform_controller_->kinematics(joint_state);
    tf::Transform cog_root = transform_controller_->getCog();
    tf::Transform root_world;
    root_world.setOrigin(tf::Vector3(keypose[0],
                                     keypose[1],
                                     0.0));
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, keypose[2]);
    root_world.setRotation(q);
    tf::Transform cog_world = root_world * cog_root;

    std::vector<double> keypose_root;
    keypose_root.push_back(cog_world.getOrigin().getX());
    keypose_root.push_back(cog_world.getOrigin().getY());
    q = cog_world.getRotation();
    tf::Matrix3x3  rot_mat(q);
    tfScalar e_r, e_p, e_y;
    rot_mat.getRPY(e_r, e_p, e_y);
    keypose_root.push_back(e_y);
    for (int i = 0; i < joint_num_; i++)
      keypose_root.push_back(keypose[3 + i]);
    return keypose_root;
  }

  void MotionPlanning::gapEnvInit()
  {//pub the collision object => gap env
    left_half_corner = tf::Vector3(gap_left_x_, gap_left_y_ , gap_left_width_);
    right_half_corner = tf::Vector3(gap_left_x_ + gap_x_offset_, gap_left_y_ + gap_x_offset_ , gap_right_width_);

    collision_object_.header.frame_id = "world";
    collision_object_.id = "box";
    geometry_msgs::Pose pose1, pose2,pose3,pose4;
    pose1.position.x = left_half_corner.getX() + gap_left_width_ /2;
    pose1.position.y =  (2.5 + gap_y_offset_/2) /2;
    pose1.position.z = 0.0;
    pose1.orientation.w = 1.0;
    pose2.position.x = right_half_corner.getX() + gap_right_width_ /2;
    pose2.position.y = - (2.5 + gap_y_offset_/2) /2;
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
    primitive1.dimensions[1] = 2.5 - gap_y_offset_ / 2;
    primitive1.dimensions[2] = 1;
    collision_object_.primitives.push_back(primitive1);
    collision_object_.primitive_poses.push_back(pose1);
    primitive2.type = primitive2.BOX;
    primitive2.dimensions.resize(3);
    primitive2.dimensions[0] = gap_right_width_;
    primitive2.dimensions[1] = 2.5 - gap_y_offset_ / 2;
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
    ROS_WARN("moveit robot state number: %d, motion control state numb: %d",  init_state.getVariableCount(),  start_state_.size());
    /* temp */
    std::vector<double> start_state(3 + joint_num_ + 7); //se(2) + joint_num + 7(?)
    start_state[0] = start_state_[0]; //x
    start_state[1] = start_state_[1]; //y
    start_state[2] = start_state_[2]; //yaw
    for(int index = 0 ; index < joint_num_; index++)
      start_state[index + 3] = start_state_[index + 3];
    init_state.setVariablePositions(start_state);

    planning_scene_->getPlanningSceneMsg(planning_scene_msg_);
    planning_scene_msg_.is_diff = true;
    planning_scene_diff_pub_.publish(planning_scene_msg_);

    robot_state::RobotState& robot_state = planning_scene_->getCurrentStateNonConst();
    //check collision
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    robot_state.setVariablePositions(start_state);
    planning_scene_->checkCollision(collision_request, collision_result, robot_state, acm_);

    //if(collision_result.collision) ROS_WARN("collsion OKOKOKOKOKOKOO");
  }

  void MotionPlanning::Planning()
  {
    //planning
    //x, y
    ompl::base::StateSpacePtr se2(new ompl::base::SE2StateSpace());
    ompl::base::RealVectorBounds motion_bounds(2);
    motion_bounds.low[0] = -2;
    motion_bounds.low[1] = -2.2;
    motion_bounds.high[0] = 5;
    motion_bounds.high[1] = 2.2;
    se2->as<ompl::base::SE2StateSpace>()->setBounds(motion_bounds);
    //joints
    ompl::base::StateSpacePtr r_joints(new ompl::base::RealVectorStateSpace(joint_num_));
    ompl::base::RealVectorBounds joint_bounds(joint_num_);
    joint_bounds.setLow(-1.58);
    joint_bounds.setHigh(1.58);
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
            start->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = start_state_[3 + i];
            goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = goal_state_[3 + i];
          }
      }
    else if(planning_mode_ == gap_passing::PlanningMode::ONLY_BASE_MODE)
      {
        start->as<ompl::base::SE2StateSpace::StateType>()->setXY(start_state_[0], start_state_[1]);
        start->as<ompl::base::SE2StateSpace::StateType>()->setYaw(start_state_[2]);
        goal->as<ompl::base::SE2StateSpace::StateType>()->setXY(goal_state_[0], goal_state_[1]);
        goal->as<ompl::base::SE2StateSpace::StateType>()->setYaw(goal_state_[2]);
      }

    else if(planning_mode_ == gap_passing::PlanningMode::JOINTS_AND_BASE_MODE)
      {
        ompl::base::CompoundState* start_tmp = dynamic_cast<ompl::base::CompoundState*> (start.get());
        start_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->setXY(start_state_[0], start_state_[1]);
        start_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->setYaw(start_state_[2]);

        ompl::base::CompoundState* goal_tmp = dynamic_cast<ompl::base::CompoundState*> (goal.get());
        goal_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->setXY(goal_state_[0], goal_state_[1]);
        goal_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->setYaw(goal_state_[2]);

        for(int i = 0; i < joint_num_; i ++)
          {
            start_tmp->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[i] = start_state_[3 + i];
            goal_tmp->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[i] = goal_state_[3 + i];
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

        motion_control_->planStoring(plan_states_, planning_mode_, start_state_, goal_state_, best_cost_, calculation_time_);

        solved_ = true;
      }
    else
      std::cout << "No solution found" << std::endl;

  }

  void MotionPlanning::rosParamInit()
  {
    nhp_.param("simulator", simulator_, true);
    /* experiment data replay */
    nhp_.param("replay_experiment_data", replay_experiment_data_flag_, false);

    nhp_.param("gap_left_x", gap_left_x_, 1.0);
    nhp_.param("gap_left_y", gap_left_y_, 0.3);
    nhp_.param("gap_x_offset", gap_x_offset_, 0.6); //minus: overlap
    nhp_.param("gap_y_offset", gap_y_offset_, 0.0); //minus: overlap
    nhp_.param("gap_left_width", gap_left_width_, 0.3); //minus: bandwidth
    nhp_.param("gap_right_width", gap_right_width_, 0.3); //minus: bandwidth

    //nhp_.param("coefficient_rate", coefficient_rate_, 0.035);
    nhp_.param("ompl_mode", ompl_mode_, 0); //RRT_START_MODE
    nhp_.param("planning_mode", planning_mode_, 0); //ONLY_JOINTS_MODE
    nhp_.param("motion_sequence_rate", motion_sequence_rate_, 10.0);

    start_state_.resize(3 + joint_num_);
    nhp_.param("start_state_x", start_state_[0], 0.0);
    nhp_.param("start_state_y", start_state_[1], 0.5);
    nhp_.param("start_state_theta", start_state_[2], 0.785);
    goal_state_.resize(3 + joint_num_);
    nhp_.param("goal_state_x", goal_state_[0], 3.0);
    nhp_.param("goal_state_y", goal_state_[1], 0.5);
    nhp_.param("goal_state_theta", goal_state_[2], 0.785);

    for(int i = 0; i < joint_num_; i++)
      {
        std::stringstream joint_no;
        joint_no << i + 1;

        nhp_.param(std::string("start_state_joint") + joint_no.str(), start_state_[3 + i], 0.0);
        nhp_.param(std::string("goal_state_joint") + joint_no.str(), goal_state_[3 + i], 0.0);
      }

    nhp_.param("real_robot_move_base", real_robot_move_base_, false);

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
          length_cost_thre_ += fabs(start_state_[3 + i] - goal_state_[3 + i]);
      }
    return ompl::base::Cost(length_cost_thre_);
  }

  void MotionPlanning::moveStartCallback(const std_msgs::Empty msg){
    move_start_flag_ = true;
  }

  void MotionPlanning::desiredStateCallback(const std_msgs::Float64MultiArrayConstPtr& msg){
    for (int i = 0; i < 3 + joint_num_; ++i)
      deisred_state_[i] = msg->data[i];
  }

  void MotionPlanning::experimentRobotOdomCallback(const nav_msgs::OdometryConstPtr& msg){
    experiment_robot_cog_odom_ = *msg;
  }

  void MotionPlanning::experimentRobotJointStatesCallback(const sensor_msgs::JointStateConstPtr& joints_msg){
    experiment_robot_joint_states_ = *joints_msg;
  }
}
