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

#include <squeeze_navigation/squeeze_navigation.h>
#include <tf/transform_broadcaster.h>

/* end effector IK */
#include <differential_kinematics/motion/end_effector_ik_solver_core.h>

namespace
{
  pluginlib::ClassLoader<squeeze_motion_planner::Base> plugin_loader("squeeze_navigation", "squeeze_motion_planner::Base");

  /* end-effector ik solver */
  /*-- to avoid:
    /usr/bin/ld: CMakeFiles/squeeze_navigation_node.dir/src/squeeze_navigation_node.cpp.o: undefined reference to symbol 'ccd_vec3_origin'
    //usr/lib/x86_64-linux-gnu/libccd.so.2: error adding symbols: DSO missing from command line
    --*/
  boost::shared_ptr<EndEffectorIKSolverCore> end_effector_ik_solver_;

  nav_msgs::Odometry robot_baselink_odom_;
  aerial_robot_msgs::PoseControlPid controller_debug_;
  KDL::JntArray joint_vector_;
  sensor_msgs::JointState joint_state_;
  bool real_odom_flag_ = false;
  int state_index_ = 0;
  tf::Vector3 first_contact_point_, first_contact_normal_;
  std_msgs::ColorRGBA desired_state_color_;
  ros::Publisher debug_pub_;
  double contact_dist_err_thresh_;
  double contact_reaction_force_;
  double move_friction_force_;
  double start_return_time_;

  double max_joint_vel = 0; //debug

  bool simulation_ = false;

  bool once_flag_ = true;
}

SqueezeNavigation::SqueezeNavigation(ros::NodeHandle nh, ros::NodeHandle nhp):
  nh_(nh), nhp_(nhp), move_start_flag_(false), return_flag_(false), first_time_in_new_phase_(true), discrete_path_(0)
{
  rosParamInit();

  /* ros pub/sub, srv */
  move_start_flag_sub_ = nh_.subscribe("move_start", 1, &SqueezeNavigation::moveStartCallback, this);
  return_flag_sub_ = nh_.subscribe("return", 1, &SqueezeNavigation::returnCallback, this);
  phase_up_sub_ = nh_.subscribe("phase_proceed", 1, &SqueezeNavigation::phaseUpCallback, this);
  adjust_initial_state_sub_ = nh_.subscribe("adjust_robot_initial_state", 1, &SqueezeNavigation::adjustInitalStateCallback, this);
  plan_start_flag_sub_ = nh_.subscribe("plan_start", 1, &SqueezeNavigation::planSqueezeMotionCallback, this);
  joy_stick_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 1, &SqueezeNavigation::joyStickControl, this);

  joints_ctrl_pub_ = nh_.advertise<sensor_msgs::JointState>("joints_ctrl", 1);
  flight_nav_pub_ = nh_.advertise<aerial_robot_msgs::FlightNav>("uav/nav", 1);
  rot_nav_pub_ = nh_.advertise<nav_msgs::Odometry>("target_rotation_motion", 1);
  desired_path_pub_ = nh_.advertise<moveit_msgs::DisplayRobotState>("desired_robot_state", 1);
  debug_pub_ = nh_.advertise<moveit_msgs::DisplayRobotState>("debug_robot_state", 1);
  end_effector_pos_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("end_effector_pos", 1);

  robot_baselink_odom_sub_ = nh_.subscribe("uav/baselink/odom", 1, &SqueezeNavigation::robotOdomCallback, this);
  robot_joint_states_sub_ = nh_.subscribe("joint_states", 1, &SqueezeNavigation::robotJointStatesCallback, this);
  controller_debug_sub_ = nh_.subscribe("debug/pose/pid", 1, &SqueezeNavigation::controlDebugCallback, this);

  /* robot model */
  // TODO: temporary
  nhp_.param("motion_type", motion_type_, 0);
  if (motion_type_ == motion_type::SE2) //SE2
    robot_model_ptr_ = boost::shared_ptr<HydrusRobotModel>(new HydrusRobotModel(true));
  else //SE3
    robot_model_ptr_ = boost::shared_ptr<HydrusRobotModel>(new Dragon::HydrusLikeRobotModel(true));
  /* set the joint angle limit */
  for(auto itr : robot_model_ptr_->getLinkJointNames())
    {
      auto joint_ptr = robot_model_ptr_->getUrdfModel().getJoint(itr);
      assert(joint_ptr != nullptr);

      angle_min_vec_.push_back(joint_ptr->limits->lower);
      angle_max_vec_.push_back(joint_ptr->limits->upper);
    }

  /* end effector ik solver */
  end_effector_ik_solver_ = boost::shared_ptr<EndEffectorIKSolverCore>(new EndEffectorIKSolverCore(nh_, ros::NodeHandle(nhp_, "end_effector"), robot_model_ptr_, false));
  /* set end effector */
  std::string end_effector_name;
  nhp_.param("end_effector_name", end_effector_name, std::string("root"));
  double end_effector_dist; // the distance of end effector from parent segment (link)

  nhp_.param("end_effector_dist", end_effector_dist, 0.0);
  end_effector_ik_solver_->setEndEffectorPose(end_effector_name,
                                              tf::Transform(tf::createIdentityQuaternion(), tf::Vector3(end_effector_dist, 0, 0)));

  /* discrete path search */
  std::string discrete_path_search_method_name;
  nhp_.param("discrete_path_search_method_name", discrete_path_search_method_name, std::string("none"));
  discrete_path_planner_ = plugin_loader.createInstance(discrete_path_search_method_name);
  discrete_path_planner_->initialize(nh_, nhp_, robot_model_ptr_);

  /* continous path generator */
  bspline_ptr_ = boost::shared_ptr<BsplineRos>(new BsplineRos(nh_, nhp_));

  /* navigation timer */
  navigate_timer_ = nh_.createTimer(ros::Duration(1.0 / controller_freq_), &SqueezeNavigation::stateMachine, this);
}

void SqueezeNavigation::rosParamInit()
{
  /* ros param */
  ros::NodeHandle nh_global("~");
  nh_global.param("/use_sim_time", simulation_, false);

  nhp_.param("headless", headless_, true);
  nhp_.param("debug_verbose", debug_verbose_, false);
  nhp_.param("replay", replay_, false);

  // end effector
  std::vector<double> first_contact_point;
  nhp_.getParam("first_contact_point", first_contact_point);
  first_contact_point_.setValue(first_contact_point.at(0),
                                first_contact_point.at(1),
                                first_contact_point.at(2));
  std::vector<double> first_contact_normal;
  nhp_.getParam("first_contact_normal", first_contact_normal);
  first_contact_normal_.setValue(first_contact_normal.at(0),
                                 first_contact_normal.at(1),
                                 first_contact_normal.at(2));


  nhp_.param("contact_dist_err_thresh", contact_dist_err_thresh_, 0.0);
  nhp_.param("contact_reaction_force", contact_reaction_force_, 0.0);
  nhp_.param("move_friction_force", move_friction_force_, 5.0);

  // discrete path
  nhp_.param("discrete_path_debug_flag", discrete_path_debug_flag_, false);

  // continuous path
  nhp_.param("bspline_degree", bspline_degree_, 5);

  // navigation
  nhp_.param("teleop_flag", teleop_flag_, true);
  nhp_.param("init_phase", motion_phase_, (int)PHASE0);
  nhp_.param("control_frequency", controller_freq_, 40.0);
  double temp;
  nhp_.param("desired_state_color_r", temp, 0.0);
  desired_state_color_.r = temp;
  nhp_.param("desired_state_color_g", temp, 0.0);
  desired_state_color_.g = temp;
  nhp_.param("desired_state_color_b", temp, 1.0);
  desired_state_color_.b = temp;
  nhp_.param("desired_state_color_a", temp, 0.6);
  desired_state_color_.a = temp;

  nhp_.param("return_delay", return_delay_, 15.0);
}

/* only plan for squeezing phase */
void SqueezeNavigation::planSqueezeMotionCallback(const std_msgs::Empty msg)
{
  ROS_INFO("[SqueezeNavigation] Receive plan start topic, skip to squeeze phase");
  state_index_ = 0; //reset
  motion_phase_ = PHASE5;
}

void SqueezeNavigation::phaseUpCallback(const std_msgs::Empty msg)
{
  if(motion_phase_ == PHASE0)
    {
      ROS_WARN("squeeze navigation: start motion");
      motion_phase_++;
      first_time_in_new_phase_ = true;
    }
  else
    {
      if(teleop_flag_)
        {
          motion_phase_++;
          ROS_WARN("squeeze navigation: enter PHASE%d", motion_phase_);
          first_time_in_new_phase_ = true;
        }
    }
}

void SqueezeNavigation::moveStartCallback(const std_msgs::Empty msg)
{
  if(discrete_path_.size() == 0)
    {
      ROS_ERROR("have not get planned path");
      return;
    }
  ROS_INFO("[SqueezeNavigation] Receive move start topic.");

  /* start move */
  startNavigate();
}

void SqueezeNavigation::stateMachine(const ros::TimerEvent& event)
{
  switch(motion_phase_)
    {
    case PHASE0:
      {
        // do nothing
        break;
      }
    case PHASE1:
      {
        if(first_time_in_new_phase_)
          {
            /* plan the end effector IK */
            tf::Transform target_frame(tf::createIdentityQuaternion(), first_contact_point_);

            /* -- set offset for point -- */
            std::vector<double> contact_offset;
            nhp_.getParam("contact_offset", contact_offset);
            target_frame.getOrigin() +=  tf::Vector3(contact_offset.at(0), contact_offset.at(1), contact_offset.at(2));

            /* -- set orientation -- */
            bool orientation_flag = false;
            std::vector<double> first_contact_euler;
            nhp_.getParam("first_contact_euler", first_contact_euler);
            if(!first_contact_euler.empty())
              {
                orientation_flag = true;
                target_frame.setRotation(tf::createQuaternionFromRPY(first_contact_euler.at(0),
                                                                     first_contact_euler.at(1),
                                                                     first_contact_euler.at(2)));
                ROS_WARN("Phase1: get first contact euler [%f, %f, %f]",
                         first_contact_euler.at(0),
                         first_contact_euler.at(1),
                         first_contact_euler.at(2));
              }

            ROS_WARN("Phase1: get first contact point [%f, %f, %f]",
                     target_frame.getOrigin().x(),
                     target_frame.getOrigin().y(),
                     target_frame.getOrigin().z());


            geometry_msgs::Pose root_pose;
            tf::Transform root_tf;
#if 0 //get init state from real robot state
            MultilinkState::convertBaselinkPose2RootPose(robot_model_ptr_, robot_baselink_odom_.pose.pose, joint_vector_, root_pose);
#else //get init state from target state
            tf::Quaternion desired_att = tf::createQuaternionFromRPY(0, 0, controller_debug_.yaw.target_p);//CoG roll&pitch is zero, only yaw
            geometry_msgs::Pose cog_pose;
            cog_pose.position.x = controller_debug_.x.target_p;
            cog_pose.position.y = controller_debug_.y.target_p;
            cog_pose.position.z = controller_debug_.z.target_p;
            cog_pose.orientation.w = 1;
            MultilinkState::convertCogPose2RootPose(robot_model_ptr_, desired_att, cog_pose, joint_vector_, root_pose);
#endif
            tf::poseMsgToTF(root_pose, root_tf);

            if(!end_effector_ik_solver_->inverseKinematics(target_frame, joint_state_, root_tf, orientation_flag, true, std::string(""), std::string(""), false, false))
              {
                ROS_ERROR("squeeze navigation: can not get valid end effector ik pose in phase1");
                reset();
                break;
              }

            discrete_path_ = end_effector_ik_solver_->getPathConst();

            /* get continous path */
            double approach_trajectory_period;
            nhp_.param("approach_trajectory_period", approach_trajectory_period, 5.0);
            continuousPath(discrete_path_, approach_trajectory_period);

            //if (!teleop_flag_ || discrete_path_debug_flag_) do not need stop (step debug)
            startNavigate();


            first_time_in_new_phase_ = false;
          }

        /* finish approach motion */
        if(!teleop_flag_ && !move_start_flag_)
          {
            motion_phase_++;
            first_time_in_new_phase_ = true;
          }

        if(replay_)
          {
            auto segs_tf = robot_model_ptr_->fullForwardKinematics(joint_vector_);
            tf::Transform fc_pose_world_frame;
            tf::poseMsgToTF(robot_baselink_odom_.pose.pose, fc_pose_world_frame);
            tf::Transform end_link_fc_frame;
            tf::transformKDLToTF(segs_tf.at("fc").Inverse() * segs_tf.at(end_effector_ik_solver_->getParentSegName()), end_link_fc_frame);

            tf::Transform end_effector_world_frame = fc_pose_world_frame * end_link_fc_frame * end_effector_ik_solver_->getEndEffectorRelativePose();

            geometry_msgs::Vector3Stamped ee_pos_;
            ee_pos_.header = robot_baselink_odom_.header;
            ee_pos_.vector.x = end_effector_world_frame.getOrigin().x();
            ee_pos_.vector.y = end_effector_world_frame.getOrigin().y();
            ee_pos_.vector.z = end_effector_world_frame.getOrigin().z();
            end_effector_pos_pub_.publish(ee_pos_);
          }

        break;
      }
    case PHASE2:
      {
        auto segs_tf = robot_model_ptr_->fullForwardKinematics(joint_vector_);

        if(segs_tf.find("fc") == segs_tf.end())
          {
            ROS_ERROR("squeeze navigation: can not tf for fc in phase2");
            reset();
            break;
          }

        tf::Transform fc_pose_world_frame;
        tf::poseMsgToTF(robot_baselink_odom_.pose.pose, fc_pose_world_frame);
        tf::Transform end_link_fc_frame;
        tf::transformKDLToTF(segs_tf.at("fc").Inverse() * segs_tf.at(end_effector_ik_solver_->getParentSegName()), end_link_fc_frame);

        tf::Transform end_effector_world_frame = fc_pose_world_frame * end_link_fc_frame * end_effector_ik_solver_->getEndEffectorRelativePose();

        // debug
        ROS_INFO("phase2: end effector position [%f, %f, %f]",
                 end_effector_world_frame.getOrigin().x(),
                 end_effector_world_frame.getOrigin().y(),
                 end_effector_world_frame.getOrigin().z());

        if(first_time_in_new_phase_)
          {
            /* second closer approach: only move CoG */
            /* TODO1: change the joint */

            /* TODO2: diff_vec should be same with contact_offset */
            tf::Vector3 diff_vec = (first_contact_point_ - end_effector_world_frame.getOrigin()).dot(first_contact_normal_) * first_contact_normal_;

            aerial_robot_msgs::FlightNav nav_msg;
            nav_msg.header.frame_id = std::string("/world");
            nav_msg.header.stamp = ros::Time::now();
            nav_msg.control_frame = nav_msg.WORLD_FRAME;
            nav_msg.target = nav_msg.COG;
            nav_msg.pos_xy_nav_mode = nav_msg.POS_MODE;
            nav_msg.target_pos_x = controller_debug_.x.target_p + diff_vec.x();
            nav_msg.target_pos_y = controller_debug_.y.target_p + diff_vec.y();
            nav_msg.pos_z_nav_mode = nav_msg.POS_MODE;
            nav_msg.target_pos_z = controller_debug_.z.target_p + diff_vec.z();

            if(!replay_) flight_nav_pub_.publish(nav_msg);

            first_time_in_new_phase_ = false;
          }

        if((end_effector_world_frame.getOrigin() - first_contact_point_).dot(first_contact_normal_) < contact_dist_err_thresh_  && once_flag_)
          {
            ROS_WARN("apply vertical force in phase2");
            /* apply external wrench for the vertical contact force */
            ros::ServiceClient client = nh_.serviceClient<gazebo_msgs::ApplyBodyWrench>("/apply_external_wrench");
            gazebo_msgs::ApplyBodyWrench srv;
            srv.request.body_name = end_effector_ik_solver_->getParentSegName();
            geometry_msgs::Point reference_point;
            tf::pointTFToMsg(end_effector_ik_solver_->getEndEffectorRelativePose().getOrigin(), reference_point);
            srv.request.reference_point = reference_point;
            /* consider the contact narmal */
            tf::vector3TFToMsg(first_contact_normal_ * contact_reaction_force_,
                               srv.request.wrench.force);

            if(!simulation_ && !discrete_path_debug_flag_)
              client.call(srv);

            if(!teleop_flag_ && !move_start_flag_)
              {
                motion_phase_++;
                first_time_in_new_phase_ = true;
              }

            once_flag_ = false;
          }
        break;
      }
    case PHASE3:
      {
        if(first_time_in_new_phase_)
          {
            std::vector<double> move_end_point;
            nhp_.getParam("move_end_point", move_end_point);
            ROS_WARN("Phase3: get move end point [%f, %f, %f]", move_end_point.at(0), move_end_point.at(1), move_end_point.at(2));

            /* plan the end effector IK */
            tf::Transform target_frame(tf::createIdentityQuaternion(),
                                       tf::Vector3(move_end_point.at(0),
                                                   move_end_point.at(1),
                                                   move_end_point.at(2)));

            /* -- set orientation -- */
            bool orientation_flag = false;
            std::vector<double> move_end_euler;
            nhp_.getParam("move_end_euler", move_end_euler);
            if(!move_end_euler.empty())
              {
                orientation_flag = true;
                target_frame.setRotation(tf::createQuaternionFromRPY(move_end_euler.at(0),
                                                                     move_end_euler.at(1),
                                                                     move_end_euler.at(2)));
                ROS_WARN("Phase3: get move end euler [%f, %f, %f]",
                         move_end_euler.at(0), move_end_euler.at(1), move_end_euler.at(2));
              }

            geometry_msgs::Pose root_pose;
            tf::Transform root_tf;
            MultilinkState::convertBaselinkPose2RootPose(robot_model_ptr_, robot_baselink_odom_.pose.pose, joint_vector_, root_pose);
            tf::poseMsgToTF(root_pose, root_tf);
            if(!end_effector_ik_solver_->inverseKinematics(target_frame, joint_state_, root_tf, orientation_flag, true, std::string(""), std::string(""), false, false))
              {
                ROS_ERROR("squeeze navigation: can not get valid end effector ik pose in phase3");
                reset();
                break;
              }

            discrete_path_ = end_effector_ik_solver_->getPathConst();

            /* get continous path */
            double move_trajectory_period;
            nhp_.param("move_trajectory_period", move_trajectory_period, 5.0);
            continuousPath(discrete_path_, move_trajectory_period);

            //if (!teleop_flag_ || discrete_path_debug_flag_) do not need stop (step debug)
            startNavigate();

            /* apply external wrench for the vertical contact force and moving friction */
            ros::ServiceClient client = nh_.serviceClient<gazebo_msgs::ApplyBodyWrench>("/apply_external_wrench");
            gazebo_msgs::ApplyBodyWrench srv;
            srv.request.body_name = end_effector_ik_solver_->getParentSegName();
            geometry_msgs::Point reference_point;
            tf::pointTFToMsg(end_effector_ik_solver_->getEndEffectorRelativePose().getOrigin(), reference_point);
            srv.request.reference_point = reference_point;

            tf::Vector3 friction_direction = (first_contact_point_ - target_frame.getOrigin()).normalize(); // opposite to the moveing direction

            /* assumption, the first_contact_normal_ is orthogonal to the moveing direction */
            assert(friction_direction.dot(first_contact_normal_)  < 1e6);

            tf::vector3TFToMsg(first_contact_normal_ * contact_reaction_force_ +
                               move_friction_force_ * friction_direction,
                               srv.request.wrench.force);


            /* deprecated: only support horizontal pushing and pulling (i.e. height is constant)
               double direction = atan2(move_end_point.at(1) - first_contact_point_.at(1),
               move_end_point.at(0) - first_contact_point_.at(0));

               srv.request.wrench.force.x = cos(-direction) * move_friction_force_; //friction direction is opposite
               srv.request.wrench.force.y = sin(-direction) * move_friction_force_; //friction direction is opposite
               srv.request.wrench.force.z = contact_reaction_force_;
            */

            ROS_WARN("phase3: external force: [%f, %f, %f]",
                     srv.request.wrench.force.x,
                     srv.request.wrench.force.y,
                     srv.request.wrench.force.z);

            if(!simulation_ && !discrete_path_debug_flag_)
              client.call(srv);

            first_time_in_new_phase_ = false;

            once_flag_ = true; //for clear external wrench;
          }

        if(replay_)
          {
            auto segs_tf = robot_model_ptr_->fullForwardKinematics(joint_vector_);
            tf::Transform fc_pose_world_frame;
            tf::poseMsgToTF(robot_baselink_odom_.pose.pose, fc_pose_world_frame);
            tf::Transform end_link_fc_frame;
            tf::transformKDLToTF(segs_tf.at("fc").Inverse() * segs_tf.at(end_effector_ik_solver_->getParentSegName()), end_link_fc_frame);

            tf::Transform end_effector_world_frame = fc_pose_world_frame * end_link_fc_frame * end_effector_ik_solver_->getEndEffectorRelativePose();

            geometry_msgs::Vector3Stamped ee_pos_;
            ee_pos_.header = robot_baselink_odom_.header;
            ee_pos_.vector.x = end_effector_world_frame.getOrigin().x();
            ee_pos_.vector.y = end_effector_world_frame.getOrigin().y();
            ee_pos_.vector.z = end_effector_world_frame.getOrigin().z();
            end_effector_pos_pub_.publish(ee_pos_);
          }

        if(!move_start_flag_)
          {
            if(once_flag_)
              {
                /* clear external wrench */
                ros::ServiceClient client = nh_.serviceClient<gazebo_msgs::BodyRequest>("/clear_external_wrench");
                gazebo_msgs::BodyRequest srv;
                srv.request.body_name = end_effector_ik_solver_->getParentSegName();

                client.call(srv);
                once_flag_ = false;
              }

            if(!teleop_flag_)
              {
                motion_phase_++;
                first_time_in_new_phase_ = true;
              }
          }

        break;
      }
    case PHASE4:
      {
        if(first_time_in_new_phase_)
          {
            std::vector<double> reset_end_effector_point;
            nhp_.getParam("reset_end_effector_point", reset_end_effector_point);

            /* plan the end effector IK */
             tf::Transform target_frame(tf::createIdentityQuaternion(),
                                        tf::Vector3(reset_end_effector_point.at(0),
                                                    reset_end_effector_point.at(1),
                                                    reset_end_effector_point.at(2)));
            geometry_msgs::Pose root_pose;
            tf::Transform root_tf;
            MultilinkState::convertBaselinkPose2RootPose(robot_model_ptr_, robot_baselink_odom_.pose.pose, joint_vector_, root_pose);
            tf::poseMsgToTF(root_pose, root_tf);
            if(!end_effector_ik_solver_->inverseKinematics(target_frame, joint_state_, root_tf, false, true, std::string(""), std::string(""), false, false))
              {
                ROS_ERROR("squeeze navigation: can not get valid end effector ik pose in phase1");
                reset();
                break;
              }

            discrete_path_ = end_effector_ik_solver_->getPathConst();

            /* get continous path */
            double reset_pose_period;
            nhp_.param("reset_pose_period", reset_pose_period, 5.0);
            continuousPath(discrete_path_, reset_pose_period);

            //if (!teleop_flag_ || discrete_path_debug_flag_) do not need stop (step debug)
            startNavigate();

            first_time_in_new_phase_ = false;

            /* TODO:hard-coding */
            //reset the collision env */
            double cover_thickness, squeeze_center_offset_x, squeeze_center_offset_y;
            nhp_.param("cover_thickness", cover_thickness, 0.0);
            nhp_.param("squeeze_center_offset_x", squeeze_center_offset_x, 0.0);
            nhp_.param("squeeze_center_offset_y", squeeze_center_offset_y, 0.0);
            tf::Transform openning_center_frame = discrete_path_planner_->getOpenningCenterFrame();
            openning_center_frame.getOrigin() += tf::Vector3(squeeze_center_offset_x,
                                                            squeeze_center_offset_y,
                                                            cover_thickness /2);
            discrete_path_planner_->setOpenningCenterFrame(openning_center_frame);
          }

        /* finish approach motion */
        if(!teleop_flag_ && !move_start_flag_)
          {
            motion_phase_++;
            first_time_in_new_phase_ = true;
          }

        break;
      }
    case PHASE5:
      {
        /* get valid squeeze path */
        if(first_time_in_new_phase_)
          {
            /* disable the real motion*/
            move_start_flag_ = false;

            bool start_squeeze_path_from_real_state;
            nhp_.param("start_squeeze_path_from_real_state", start_squeeze_path_from_real_state, false);

            /*-- 1. get discrete path for squeezing motion --*/
            bool load_squeeze_path_flag;
            nhp_.param("load_squeeze_path_flag", load_squeeze_path_flag, false);
            if(load_squeeze_path_flag)
              {
                discrete_path_planner_->loadPath();
                if(discrete_path_planner_->getPathConst().size() == 0)
                  {
                    ROS_ERROR("squeeze navigation: can not get valid discrete path from sampling based method");
                    reset();
                    break;
                  }
              }
            else
              { /*-- do online planning --*/
                if(discrete_path_debug_flag_) start_squeeze_path_from_real_state = false;

                if(start_squeeze_path_from_real_state)
                  {
                    /* get from the real state */
                    if(discrete_path_.size() == 0)
                      {
                        geometry_msgs::Pose root_pose;
                        MultilinkState::convertBaselinkPose2RootPose(robot_model_ptr_, robot_baselink_odom_.pose.pose, joint_vector_, root_pose);
                        discrete_path_planner_->setInitState(MultilinkState(robot_model_ptr_, root_pose, joint_vector_));
                      }
                    else
                      discrete_path_planner_->setInitState(discrete_path_.back()); //get the last target state from last navigation
                  }

                if(!discrete_path_planner_->plan(debug_verbose_))
                  {
                    ROS_ERROR("squeeze navigation: cannot get valid planning result for PHASE%d", motion_phase_);
                    reset();
                    break;
                  }
              }

            discrete_path_ = discrete_path_planner_->getPathConst();

            /*-- 2. smoothing the discrete path --*/
            bool discrete_path_filter_flag;
            nhp_.param("discrete_path_filter_flag", discrete_path_filter_flag, false);
            if(discrete_path_filter_flag) discrete_path_ = discretePathSmoothing(discrete_path_);

            /*-- 3. resampling the discrete path --*/
            bool discrete_path_resampling_flag;
            nhp_.param("discrete_path_resampling_flag", discrete_path_resampling_flag, false);
            if(discrete_path_resampling_flag) discrete_path_ = discretePathResampling(discrete_path_);

            /*-- 4. get continous path --*/
            double squeeze_trajectory_period;
            nhp_.param("squeeze_trajectory_period", squeeze_trajectory_period, 100.0);
            continuousPath(discrete_path_, squeeze_trajectory_period);

            if (start_squeeze_path_from_real_state) startNavigate();

            first_time_in_new_phase_ = false;
          }

        /* additional process: return */
        if (!move_start_flag_ && return_flag_)
          {
            double t = ros::Time::now().toSec() - start_return_time_;
            if(t < return_delay_ * 2 / 3)
              {
                nav_msgs::Odometry rot_msg;
                double rate = 1 - t / (return_delay_ * 2 / 3);
                rot_msg.header.stamp = ros::Time::now();
                rot_msg.header.frame_id = std::string("baselink");
                double roll = rate * bspline_ptr_->evaluate(bspline_ptr_->getEndTime() + 1.0 / controller_freq_)[3];
                double pitch = rate * bspline_ptr_->evaluate(bspline_ptr_->getEndTime() + 1.0 / controller_freq_)[4];
                double yaw = bspline_ptr_->evaluate(bspline_ptr_->getEndTime() + 1.0 / controller_freq_)[5];
                rot_msg.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
                rot_nav_pub_.publish(rot_msg);
                ROS_INFO_THROTTLE(1.0, "set robot level and the init joint state ");
              }

            if(t > return_delay_)
              {
                /* set SE2 goal (return) position */
                if (nhp_.hasParam("final_pos_x") && nhp_.hasParam("final_pos_y") && nhp_.hasParam("final_yaw"))
                  {
                    double final_pos_x, final_pos_y, final_yaw;
                    nhp_.getParam("final_pos_x", final_pos_x);
                    nhp_.getParam("final_pos_y", final_pos_y);
                    nhp_.getParam("final_yaw", final_yaw);

                    aerial_robot_msgs::FlightNav nav_msg;
                    nav_msg.header.frame_id = std::string("/world");
                    nav_msg.header.stamp = ros::Time::now();
                    /* x & y */
                    nav_msg.control_frame = nav_msg.WORLD_FRAME;
                    nav_msg.target = nav_msg.COG;
                    nav_msg.pos_xy_nav_mode = nav_msg.POS_MODE;
                    nav_msg.target_pos_x = final_pos_x;
                    nav_msg.target_pos_y = final_pos_y;
                    flight_nav_pub_.publish(nav_msg);

                    /* yaw */
                    nav_msgs::Odometry rot_msg;
                    rot_msg.header.stamp = ros::Time::now();
                    rot_msg.header.frame_id = std::string("baselink");
                    rot_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(final_yaw);
                    rot_nav_pub_.publish(rot_msg);
                  }

                reset();
              }
          }
      }
    default:
      {
        break;
      }
    }

  /* navigation */
  pathNavigate();
}

const std::vector<MultilinkState> SqueezeNavigation::discretePathSmoothing(const std::vector<MultilinkState>& raw_path) const
{
  /* low path filter */
  double filter_rate;
  nhp_.param("filter_rate", filter_rate, 0.1);

  int joint_num = robot_model_ptr_->getLinkJointIndices().size();
  std::vector<MultilinkState> filtered_path(0);
  FirFilter states_lpf1 = FirFilter(filter_rate, 3 + joint_num); //root position + joint_num
  FirFilterQuaternion states_lpf2 = FirFilterQuaternion(filter_rate); //root orientation

  /* init state */
  auto init_state = raw_path.at(0);
  Eigen::VectorXd init_state_vec = Eigen::VectorXd::Zero(3 + joint_num);
  init_state_vec.head(3) = Eigen::Vector3d(init_state.getRootPoseConst().position.x,
                                           init_state.getRootPoseConst().position.y,
                                           init_state.getRootPoseConst().position.z);
  for(int j = 0; j < joint_num; j++)
    init_state_vec(3 + j) = init_state.getJointStateConst()(robot_model_ptr_->getLinkJointIndices().at(j));
  states_lpf1.setInitValues(init_state_vec); //init filter with the first value

  tf::Quaternion init_q;
  tf::quaternionMsgToTF(init_state.getRootPoseConst().orientation, init_q);
  states_lpf2.setInitValues(init_q); //init filter with the first value

  /* do filtering */
  for(int index = 0; index < raw_path.size() + 1 / states_lpf1.getFilterFactor(); index++)
    {
      auto state_itr = raw_path.back();
      if(index < raw_path.size()) state_itr = raw_path.at(index);

      Eigen::VectorXd state_vec = Eigen::VectorXd::Zero(3 + joint_num);
      state_vec.head(3) = Eigen::Vector3d(state_itr.getRootPoseConst().position.x,
                                          state_itr.getRootPoseConst().position.y,
                                          state_itr.getRootPoseConst().position.z);

      for(int j = 0; j < joint_num; j++)
        state_vec(3 + j) = state_itr.getJointStateConst()(robot_model_ptr_->getLinkJointIndices().at(j));
      Eigen::VectorXd filtered_state =  states_lpf1.filterFunction(state_vec);

      /* joint */
      auto filtered_joint_vector = state_itr.getJointStateConst();
      for(int j = 0; j < joint_num; j++)
        filtered_joint_vector(robot_model_ptr_->getLinkJointIndices().at(j)) = filtered_state(3 + j);

      /* root position */
      geometry_msgs::Pose filtered_root_pose;
      filtered_root_pose.position.x = filtered_state(0);
      filtered_root_pose.position.y = filtered_state(1);
      filtered_root_pose.position.z = filtered_state(2);

      /* root orientation */
      tf::Quaternion raw_q;
      tf::quaternionMsgToTF(state_itr.getRootPoseConst().orientation, raw_q);
      tf::quaternionTFToMsg(states_lpf2.filterFunction(raw_q), filtered_root_pose.orientation);

      filtered_path.push_back(MultilinkState(robot_model_ptr_, filtered_root_pose, filtered_joint_vector));
    }

  return filtered_path;
}

const std::vector<MultilinkState> SqueezeNavigation::discretePathResampling(const std::vector<MultilinkState>& raw_path) const
{
  /* resampling */
  double resampling_angular_rate;
  double resampling_seg_diff_thresh;
  nhp_.param("resampling_angular_rate", resampling_angular_rate, 1.0);
  nhp_.param("resampling_seg_diff_thresh", resampling_seg_diff_thresh, 0.2);
  double path_length = 0;

  for(int i = 0; i < raw_path.size() - 1; i++)
    {
      /* translation motion */
      tf::Vector3 p1, p2;
      tf::pointMsgToTF(raw_path.at(i).getCogPoseConst().position, p1);
      tf::pointMsgToTF(raw_path.at(i + 1).getCogPoseConst().position, p2);
      path_length += (p1-p2).length();

      /* rotational motion */
      double angle_diff = fabs(raw_path.at(i).getBaselinkDesiredAttConst().angleShortestPath(raw_path.at(i+1).getBaselinkDesiredAttConst()));
      path_length += resampling_angular_rate * angle_diff;
      //ROS_WARN("%d: angle_diff: %f", i, angle_diff);
    }

  double ave_delta_trans = path_length / raw_path.size();
  ROS_WARN("path length is %f, average delta trans is %f", path_length, ave_delta_trans);

  /* do resampling */
  std::vector<MultilinkState> resampling_path;
  double delta_trans = 0;
  bool not_enough = false;
  resampling_path.push_back(raw_path.front());

  for(int i = 1; i < raw_path.size();)
    {
      tf::Vector3 prev_p;
      tf::pointMsgToTF(raw_path.at(i - 1).getCogPoseConst().position, prev_p);
      tf::Quaternion prev_q = raw_path.at(i - 1).getBaselinkDesiredAttConst();
      tf::Vector3 new_p;
      tf::pointMsgToTF(raw_path.at(i).getCogPoseConst().position, new_p);
      tf::Quaternion new_q = raw_path.at(i).getBaselinkDesiredAttConst();
      tf::Vector3 ref_p;
      tf::pointMsgToTF(resampling_path.back().getCogPoseConst().position, ref_p);
      tf::Quaternion ref_q = resampling_path.back().getBaselinkDesiredAttConst();

      if(delta_trans == 0) delta_trans = (ref_p - new_p).length() + resampling_angular_rate * fabs(new_q.angleShortestPath(ref_q));
      if(not_enough)
        {
          delta_trans += ((prev_p - new_p).length() + resampling_angular_rate * fabs(new_q.angleShortestPath(prev_q)));
          not_enough = false;
        }

      if(debug_verbose_)
        {
          double r,p,y;
          tf::Matrix3x3(resampling_path.back().getBaselinkDesiredAttConst()).getRPY(r,p,y);
          ROS_INFO("ref pose [%f, %f, %f] [%f, %f, %f]", ref_p.x(), ref_p.y() ,ref_p.z(), r, p, y);
          std::cout << "new point vs resample ref point: [" << i << ", " << resampling_path.size() << "], inrcement vs delta_trans vs ave: [" << (ref_p - new_p).length() + resampling_angular_rate * fabs(new_q.angleShortestPath(ref_q)) << ", " << delta_trans << ", " << ave_delta_trans << "]. ";
        }

      if(fabs(delta_trans - ave_delta_trans) < resampling_seg_diff_thresh * ave_delta_trans)
        {
          if(debug_verbose_)
            {
              std::cout << "add raw state directly since convergence. rate: " << fabs(delta_trans - ave_delta_trans) / ave_delta_trans << ". " << std::endl;
              ROS_WARN("delta_trans: %f", delta_trans);
            }
          resampling_path.push_back(raw_path.at(i));
          delta_trans = 0;
          i++;
        }
      else if (delta_trans >  ave_delta_trans)
        {
          // interpolation
          double interpolate_rate = 1 - (delta_trans - ave_delta_trans) / ((prev_p - new_p).length() + resampling_angular_rate * fabs(prev_q.angleShortestPath(new_q)));
          if(debug_verbose_)
            std::cout << " interpolation since too big delta trans. interpolate rate: " << interpolate_rate << ". "  << std::endl;


          /* joint */
          auto joint_vector = raw_path.at(i - 1).getJointStateConst();
          for(auto index: robot_model_ptr_->getLinkJointIndices())
            joint_vector(index) = raw_path.at(i - 1).getJointStateConst()(index) * (1 - interpolate_rate) + raw_path.at(i).getJointStateConst()(index) * interpolate_rate;


          /* cog pose */
          geometry_msgs::Pose cog_pose;
          tf::pointTFToMsg(prev_p * (1 - interpolate_rate) + new_p * interpolate_rate, cog_pose.position);
          cog_pose.orientation.w = 1;

          /* baselink attitude and  root pose */
          tf::Quaternion desired_baselink_q = raw_path.at(i - 1).getBaselinkDesiredAttConst().slerp(raw_path.at(i).getBaselinkDesiredAttConst(), interpolate_rate);

          resampling_path.push_back(MultilinkState(robot_model_ptr_,
                                                   desired_baselink_q, cog_pose,
                                                   joint_vector));
          tf::Vector3 new_ref_p;
          tf::pointMsgToTF(resampling_path.back().getCogPoseConst().position, new_ref_p);
          tf::Quaternion new_ref_q = resampling_path.back().getBaselinkDesiredAttConst();
          delta_trans = (new_ref_p - new_p).length() + resampling_angular_rate * fabs(new_q.angleShortestPath(new_ref_q));

          if(debug_verbose_) ROS_WARN("delta_trans: %f", delta_trans);
        }
      else
        {
          not_enough = true;
          i++;
          if(debug_verbose_) std::cout << "  not enough delta trans. "  << std::endl;
        }
    }

  return resampling_path;
}

void SqueezeNavigation::continuousPath(const std::vector<MultilinkState>& discrete_path, double trajectory_period)
{
  if(replay_) return;

  int joint_num = robot_model_ptr_->getLinkJointIndices().size();

  /* insert data */
  std::vector<std::vector<double> > control_point_list;
  for (int i = -1; i < (int)discrete_path.size() + 1; i++)
    {
      /* add one more start & end keypose to guarantee speed 0 */
      int id = i;
      if (id < 0) id = 0;
      if (id >= discrete_path.size()) id = discrete_path.size() - 1;

      std::vector<double> control_point;

      // cog position
      const auto cog_pos = discrete_path.at(id).getCogPoseConst().position;
      control_point.push_back(cog_pos.x);
      control_point.push_back(cog_pos.y);
      control_point.push_back(cog_pos.z);

      // euler enagles
      tf::Matrix3x3 att(discrete_path.at(id).getBaselinkDesiredAttConst());
      double r, p, y; att.getRPY(r, p, y);
      control_point.push_back(r);
      control_point.push_back(p);
      /* TODO: use quaternion bspline: keep yaw euler angle continous */
      control_point.push_back(generateContinousEulerAngle(y, i));

      /* set joint state */
      for(auto itr : robot_model_ptr_->getLinkJointIndices())
        control_point.push_back(discrete_path.at(id).getJointStateConst()(itr));

      control_point_list.push_back(control_point);
    }

  bspline_ptr_->initialize(true, 0, trajectory_period, bspline_degree_, control_point_list);
  std::vector<int> pos_indicies{0,1,2};
  bspline_ptr_->display3dPath(pos_indicies);

  ROS_INFO("Spline display finished.");
}

void SqueezeNavigation::adjustInitalStateCallback(const std_msgs::Empty msg)
{
  if(discrete_path_.size() == 0)
    {
      ROS_ERROR("have not get planned path");
      return;
    }

  const MultilinkState& init_state = discrete_path_.at(0);

  /* publish uav nav */
  aerial_robot_msgs::FlightNav nav_msg;
  nav_msg.header.frame_id = std::string("/world");
  nav_msg.header.stamp = ros::Time::now();

  /* x & y */
  nav_msg.control_frame = nav_msg.WORLD_FRAME;
  nav_msg.target = nav_msg.COG;
  nav_msg.pos_xy_nav_mode = nav_msg.POS_MODE;
  nav_msg.target_pos_x = init_state.getCogPoseConst().position.x;
  nav_msg.target_pos_y = init_state.getCogPoseConst().position.y;

  /* z */
  if(motion_type_ == motion_type::SE2)
    {
      nav_msg.pos_z_nav_mode = nav_msg.NO_NAVIGATION;
    }
  else if(motion_type_ == motion_type::SE3)
    {
      nav_msg.pos_z_nav_mode = nav_msg.POS_MODE;
      nav_msg.target_pos_z = init_state.getCogPoseConst().position.z;
    }
  flight_nav_pub_.publish(nav_msg);

  /* joint states */
  sensor_msgs::JointState joints_msg;
  joints_msg.header = nav_msg.header;
  for(auto itr : robot_model_ptr_->getLinkJointIndices())
    joints_msg.position.push_back(init_state.getJointStateConst()(itr));
  joints_ctrl_pub_.publish(joints_msg);

  /* rpy */
  tf::Matrix3x3 att(init_state.getBaselinkDesiredAttConst());
  double r, p, y; att.getRPY(r, p, y);

  /* se3: roll & pitch */
  nav_msgs::Odometry rot_msg;
  rot_msg.header.stamp = ros::Time::now();
  rot_msg.header.frame_id = std::string("baselink");
  rot_msg.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(r, p, y);
  rot_nav_pub_.publish(rot_msg);
}

void SqueezeNavigation::returnCallback(const std_msgs::Empty msg)
{
  if(move_start_flag_)
    {
      ROS_WARN("still performing desired path motion");
      return;
    }
  return_flag_ = true;

  start_return_time_ = ros::Time::now().toSec();

  /* send init joint state */
  sensor_msgs::JointState joints_msg;
  joints_msg.header.stamp = ros::Time::now();
  for(int i = 0 ; i < robot_model_ptr_->getLinkJointNames().size(); i++)
    {
      joints_msg.name.push_back(robot_model_ptr_->getLinkJointNames().at(i));
      joints_msg.position.push_back(discrete_path_planner_->getPathConst().at(0).getJointStateConst()(robot_model_ptr_->getLinkJointIndices().at(i)));
    }

  joints_ctrl_pub_.publish(joints_msg);
}

void SqueezeNavigation::pathNavigate()
{
  /* some special visualize process, such as ENV */
  if(!headless_) discrete_path_planner_->visualizeFunc();

  if(!move_start_flag_) return;

  int joint_num = robot_model_ptr_->getLinkJointIndices().size();
  moveit_msgs::DisplayRobotState display_robot_state;

  /* test (debug) the discrete path */
  if(discrete_path_debug_flag_)
    {
      if(state_index_ == discrete_path_.size()) return; //debug

      display_robot_state.state.joint_state.header.seq = state_index_;
      display_robot_state.state.joint_state.header.stamp = ros::Time::now();
      display_robot_state.state = discrete_path_.at(state_index_).getRootJointStateConst<moveit_msgs::RobotState>();

      desired_path_pub_.publish(display_robot_state);

      /* debug */
      {
        moveit_msgs::DisplayRobotState debug_robot_state;

        if(state_index_ +1 < discrete_path_.size())
          {
            tf::Vector3 p1, p2;
            tf::pointMsgToTF(discrete_path_.at(state_index_).getCogPoseConst().position, p1);
            tf::pointMsgToTF(discrete_path_.at(state_index_ + 1).getCogPoseConst().position, p2);
            debug_robot_state.state.joint_state.position.push_back((p1-p2).length());

            debug_robot_state.state.joint_state.position.push_back((discrete_path_.at(state_index_ + 1).getBaselinkDesiredAttConst() * discrete_path_.at(state_index_).getBaselinkDesiredAttConst().inverse()).getAngle());

            double joint_angle_sum = 0;
            for(auto itr : robot_model_ptr_->getLinkJointIndices())
              {
                double delta_angle = discrete_path_.at(state_index_ + 1).getJointStateConst()(itr) - discrete_path_.at(state_index_).getJointStateConst()(itr);
                joint_angle_sum += (delta_angle * delta_angle);
              }
            debug_robot_state.state.joint_state.position.push_back(sqrt(joint_angle_sum));

            debug_pub_.publish(debug_robot_state);
          }
      }

      if(++state_index_ == discrete_path_.size()) state_index_ = 0;

      return;
    }

  double cur_time = ros::Time::now().toSec() - move_start_time_;
  std::vector<double> des_pos = bspline_ptr_->evaluate(cur_time + 1.0 / controller_freq_);
  std::vector<double> des_vel = bspline_ptr_->evaluate(cur_time, 1);

  {
    // debug
    moveit_msgs::DisplayRobotState debug_robot_state;
    debug_robot_state.state.joint_state.position.push_back(tf::Vector3(des_vel[0], des_vel[1], des_vel[2]).length());
    debug_robot_state.state.joint_state.position.push_back(tf::Vector3(des_vel[3], des_vel[4], 0).length()); //no need to add yaw velocity

    double joint_angle_sum = 0;
    for(int i = 0; i < joint_num; i++)
      {
        joint_angle_sum += (des_vel[6+i] * des_vel[6+i]);
        if(fabs(des_vel[6+i]) > max_joint_vel)
          {
            max_joint_vel = fabs(des_vel[6+i]);
            ROS_WARN("max joint vel: %f", max_joint_vel);
          }
      }
    debug_robot_state.state.joint_state.position.push_back(sqrt(joint_angle_sum));

    debug_pub_.publish(debug_robot_state);
  }

  /* send general flight navigation command (pos + yaw) */
  aerial_robot_msgs::FlightNav nav_msg;
  nav_msg.header.frame_id = std::string("/world");
  nav_msg.header.stamp = ros::Time::now();
  /* x & y */
  nav_msg.control_frame = nav_msg.WORLD_FRAME;
  nav_msg.target = nav_msg.COG;
  nav_msg.pos_xy_nav_mode = nav_msg.POS_VEL_MODE;
  nav_msg.target_pos_x = des_pos[0];
  nav_msg.target_pos_y = des_pos[1];
  nav_msg.target_vel_x = des_vel[0];
  nav_msg.target_vel_y = des_vel[1];
  /* z axis */
  if(motion_type_ == motion_type::SE2)
    {
      nav_msg.pos_z_nav_mode = nav_msg.NO_NAVIGATION;
    }
  else if(motion_type_ == motion_type::SE3)
    {
      nav_msg.pos_z_nav_mode = nav_msg.POS_VEL_MODE;
      nav_msg.target_pos_z = des_pos[2];
      nav_msg.target_vel_z = des_vel[2];
    }
  flight_nav_pub_.publish(nav_msg);

  /* joint states */
  sensor_msgs::JointState joints_msg;
  joints_msg.header = nav_msg.header;
  for (int i = 0; i < joint_num; ++i)
    {
      joints_msg.name.push_back(robot_model_ptr_->getLinkJointNames().at(i));
      joints_msg.position.push_back(boost::algorithm::clamp(des_pos[6 + i], angle_min_vec_.at(i), angle_max_vec_.at(i)));
    }

  joints_ctrl_pub_.publish(joints_msg);

  if (debug_verbose_)
    {
      std::cout << "joints: ";
      for (int i = 0; i < joint_num; ++i) std::cout << joints_msg.position[i] << ", ";
      std::cout << "\n\n";
    }

  /* se3: rotation motion */
  nav_msgs::Odometry rot_msg;
  rot_msg.header.stamp = ros::Time::now();
  rot_msg.header.frame_id = std::string("baselink");
  rot_msg.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(des_pos[3], des_pos[4], des_pos[5]);
  // omega_body =  [[ 1 0 -sin(pitch)] [0 cos(roll) cos(pitch)sin(roll)] [0 -sin(roll) cos(pitch)cos(roll)]]  * d_rpy
  tf::Vector3 d_rpy(des_vel[3], des_vel[4], des_vel[5]);
  double sin_roll = sin(des_pos[3]);
  double cos_roll = cos(des_pos[3]);
  double sin_pitch = sin(des_pos[4]);
  double cos_pitch = cos(des_pos[4]);
  rot_msg.twist.twist.angular.x = d_rpy.x() - sin_pitch * d_rpy.z();
  rot_msg.twist.twist.angular.y = cos_roll * d_rpy.y() + sin_roll * cos_pitch * d_rpy.z();
  rot_msg.twist.twist.angular.z = -sin_roll * d_rpy.y() + cos_roll * cos_pitch * d_rpy.z();
  rot_nav_pub_.publish(rot_msg);

  //TODO: set forward roll, pitch
  /* check the end of navigation */
  if(cur_time > bspline_ptr_->getEndTime() + 1.0) // margin: 1.0 [sec]
    {
      ROS_INFO("[SqueezeNavigation] Finish Navigation");
      move_start_flag_ = false;
    }

  /* publish the target pose and form for visualize */
  /* cog pose */
  geometry_msgs::Pose cog_pose;
  cog_pose.position.x = des_pos.at(0);
  cog_pose.position.y = des_pos.at(1);
  cog_pose.position.z = des_pos.at(2);
  cog_pose.orientation = tf::createQuaternionMsgFromYaw(des_pos.at(5)); /* special: only get yaw angle */

  /* joint state:  */
  auto joint_vector = discrete_path_.at(0).getJointStateConst();
  for(int i = 0; i < joint_num; i++)
    joint_vector(robot_model_ptr_->getLinkJointIndices().at(i)) = des_pos.at(6 + i);

  MultilinkState state_tmp(robot_model_ptr_,
                           tf::createQuaternionFromRPY(des_pos[3], des_pos[4], 0), cog_pose,
                           joint_vector);
  display_robot_state.state = state_tmp.getRootJointStateConst<moveit_msgs::RobotState>();
  display_robot_state.state.joint_state.header.stamp = ros::Time::now();

  /* set color */
  for(auto itr: robot_model_ptr_->getUrdfModel().links_)
    {
      moveit_msgs::ObjectColor object_color;
      object_color.id = itr.first;
      object_color.color = desired_state_color_;
      display_robot_state.highlight_links.push_back(object_color);
    }

  desired_path_pub_.publish(display_robot_state);
}

void SqueezeNavigation::robotOdomCallback(const nav_msgs::OdometryConstPtr& msg)
{
  robot_baselink_odom_ = *msg;
  real_odom_flag_ = true;
}

void SqueezeNavigation::controlDebugCallback(const aerial_robot_msgs::PoseControlPidConstPtr& control_msg)
{
  controller_debug_ = *control_msg;
}

void SqueezeNavigation::robotJointStatesCallback(const sensor_msgs::JointStateConstPtr& joints_msg)
{
  joint_state_ = *joints_msg;
  joint_vector_ = robot_model_ptr_->jointMsgToKdl(*joints_msg);

  if(!real_odom_flag_) return;

  if(discrete_path_.size() == 0) return; // do not have valid path

  /* check collision */
  if(discrete_path_search_method_type_ == 0) // sampling base
    {
      geometry_msgs::Pose root_pose;
      MultilinkState::convertBaselinkPose2RootPose(robot_model_ptr_, robot_baselink_odom_.pose.pose, joint_vector_, root_pose);
      discrete_path_planner_->checkCollision(MultilinkState(robot_model_ptr_, root_pose, joint_vector_));
    }
}



void SqueezeNavigation::joyStickControl(const sensor_msgs::JoyConstPtr & joy_msg)
{
  using namespace aerial_robot_navigation;
  sensor_msgs::Joy joy_cmd;
  if(joy_msg->axes.size() == BaseNavigator::PS3_AXES && joy_msg->buttons.size() == BaseNavigator::PS3_BUTTONS)
    {
      joy_cmd = (*joy_msg);
    }
  else if(joy_msg->axes.size() == BaseNavigator::PS4_AXES && joy_msg->buttons.size() == BaseNavigator::PS4_BUTTONS)
    {
      joy_cmd = BaseNavigator::ps4joyToPs3joyConvert(*joy_msg);
    }
  else
    {
      ROS_WARN("the joystick type is not supported (buttons: %d, axes: %d)", (int)joy_msg->buttons.size(), (int)joy_msg->axes.size());
      return;
    }

  /* force landing */
  if(joy_cmd.buttons[BaseNavigator::PS3_BUTTON_SELECT] == 1 && move_start_flag_)
    {
      ROS_INFO("[SqueezeNavigation] Receive force landing command, stop navigation.");
      move_start_flag_ = false;
      return;
    }

  /* landing */
  if(joy_cmd.buttons[BaseNavigator::PS3_BUTTON_CROSS_RIGHT] == 1 && joy_cmd.buttons[BaseNavigator::PS3_BUTTON_ACTION_SQUARE] == 1 && move_start_flag_)
    {
      ROS_INFO("[SqueezeNavigation] Receive normal landing command, stop navigation.");
      move_start_flag_ = false;
      return;
    }
}
