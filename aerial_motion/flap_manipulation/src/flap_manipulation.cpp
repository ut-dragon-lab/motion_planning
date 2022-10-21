// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, JSK Lab
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

#include <flap_manipulation/flap_manipulation.h>
#include <tf/transform_broadcaster.h>

/* end effector IK */
#include <differential_kinematics/motion/end_effector_ik_solver_core.h>

FlapManipulation::FlapManipulation(ros::NodeHandle nh, ros::NodeHandle nhp):
  SqueezeNavigation(nh, nhp)
{
  rosParamInit();

  /* ros pub/sub, srv */
  string topic_name;
  nhp_.param("flap_pose_topic_name", topic_name, std::string("pose"));
  flap_pose_sub_ = nh_.subscribe(topic_name, 1, &FlapManipulation::flapPoseCallback, this); /// topic name: valve/mocap/pose
  apply_wrench_pub_ = nh_.advertise<aerial_robot_msgs::ApplyWrench>("apply_external_wrench", 1);
  clear_wrench_pub_ = nh_.advertise<std_msgs::String>("clear_external_wrench", 1);
  end_effector_pos_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("end_effector_pos", 1);

  /* end effector ik solver */
  end_effector_ik_solver_ = boost::shared_ptr<EndEffectorIKSolverCore>(new EndEffectorIKSolverCore(nh_, ros::NodeHandle(nhp_, "end_effector"), robot_model_ptr_, false));
  /* set end effector */
  std::string end_effector_name;
  nhp_.param("end_effector_name", end_effector_name, std::string("root"));
  double end_effector_dist; // the distance of end effector from parent segment (link)
  nhp_.param("end_effector_dist", end_effector_dist, 0.0);
  end_effector_ik_solver_->setEndEffectorPose(end_effector_name,
                                              tf::Transform(tf::createIdentityQuaternion(), tf::Vector3(end_effector_dist, 0, 0)));

  target_init_ee_pose_.setIdentity();
  target_end_ee_pose_.setIdentity();
  target_reset_ee_pose_.setIdentity();
  target_ee_pose_.setIdentity();
  flap_pose_.setOrigin(tf::Vector3(0, 0, -100)); // invliad height for target flap

  prev_joy_cmd_.axes.resize(aerial_robot_navigation::BaseNavigator::PS4_AXES, 0);
  prev_joy_cmd_.buttons.resize(aerial_robot_navigation::BaseNavigator::PS4_BUTTONS, 0);
}

void FlapManipulation::rosParamInit()
{
  // navigation
  nhp_.param("init_phase", motion_phase_, (int)PHASE0);
  nhp_.param("opening_width", opening_width_, 0.7);
  nhp_.param("opening_margin", opening_margin_, 0.05);
  nhp_.param("flap_width", flap_width_, 0.7);
  nhp_.param("flap_height", flap_height_, 0.04);
  nhp_.param("approach_offset", approach_offset_, 0.05);
  nhp_.param("approach_thresh", approach_thresh_, 0.05);
  nhp_.param("contact_thresh", contact_thresh_, 0.0);
  nhp_.param("reach_thresh", reach_thresh_, 0.04);
  nhp_.param("contact_reaction_force", contact_reaction_force_, 0.0);
  nhp_.param("move_speed", move_speed_, 0.8);
  nhp_.param("move_friction_force", move_friction_force_, 5.0);
  nhp_.param("replan_du", replan_du_, 0.8);
  nhp_.param("yaw_rate", yaw_rate_, 1.0);
  nhp_.param("auto_state_machine", auto_state_machine_, true);
  nhp_.param("squeeze_flag", squeeze_flag_, false);
  nhp_.param("external_wrench_flag", external_wrench_flag_, true);
}

void FlapManipulation::reset()
{
  SqueezeNavigation::reset();

  motion_phase_ = PHASE0;

  target_ee_pose_.setIdentity();
}

void FlapManipulation::flapPoseCallback(const geometry_msgs::PoseStampedConstPtr msg)
{
  tf::poseMsgToTF(msg->pose, flap_pose_);
}

void FlapManipulation::moveStartCallback(const std_msgs::Empty msg)
{
  motion_phase_++;
  std::stringstream ss;
  ss << "flap manipulation: enter phase" << motion_phase_;
  if (motion_phase_ > PHASE0)
    ss << "; force shift";

  plan_flag_ = true;
}

void FlapManipulation::returnCallback(const std_msgs::Empty msg)
{
  return_flag_ = true;
  start_return_time_ = ros::Time::now().toSec();
  motion_phase_ = PHASE0;
  move_flag_ = false;

  /* send init joint state */
  sensor_msgs::JointState joints_msg;
  joints_msg.header.stamp = ros::Time::now();

  /* heuristic way */
  for(int i = 0 ; i < robot_model_ptr_->getRotorNum() - 1; i++)
    {
      joints_msg.name.push_back(std::string("joint") + std::to_string(i+1) + std::string("_pitch"));
      joints_msg.position.push_back(0);
      joints_msg.name.push_back(std::string("joint") + std::to_string(i+1) + std::string("_yaw"));
      joints_msg.position.push_back(2 * M_PI / robot_model_ptr_->getRotorNum());
    }

  joints_ctrl_pub_.publish(joints_msg);


  /* clear external wrench */
  std_msgs::String m;
  m.data = "manipulte_force";
  clear_wrench_pub_.publish(m);

  if(!squeeze_flag_) return_delay_ /= 2; // shorten the time 
}

void FlapManipulation::process(const ros::TimerEvent& event)
{
  if(flap_pose_.getOrigin().z() < 0)
    {
      ROS_ERROR_THROTTLE(1.0, "flap manipulation: can not get valid flap pose");
      return;
    }

  auto segs_tf = robot_model_ptr_->fullForwardKinematics(joint_state_);
  if(segs_tf.find("fc") == segs_tf.end())
    {
      ROS_ERROR("fla manipulation: cannot get tf for fc");
      return;
    }
  tf::Transform fc_pose_world_frame;
  tf::poseMsgToTF(robot_baselink_odom_.pose.pose, fc_pose_world_frame);
  tf::Transform end_link_fc_frame;
  tf::transformKDLToTF(segs_tf.at("fc").Inverse() * segs_tf.at(end_effector_ik_solver_->getParentSegName()), end_link_fc_frame);
  tf::Transform end_effector_world_frame = fc_pose_world_frame * end_link_fc_frame * end_effector_ik_solver_->getEndEffectorRelativePose();
  tf::Vector3 ee_pos = end_effector_world_frame.getOrigin();


  switch(motion_phase_)
    {
    case PHASE0:
      {
        // do nothing
        break;
      }
    case PHASE1:
      {
        if(plan_flag_)
          {
            plan_flag_ = false;

            tf::Vector3 flap_pos = flap_pose_.getOrigin();
            tf::Transform opening_center_frame = SqueezeNavigation::path_planner_->getOpenningCenterFrame(); // TODO: use mocap or recognition for openning position

            ee_orientation_flag_ = false;

            /* heurisitc calculation for the end-effector pose for contact point and normal */
            if(robot_baselink_odom_.pose.pose.position.z < flap_pose_.getOrigin().z())
              {
                // bottom manipulation
                ROS_INFO("The flap object is above the robot, bottom manipulation");
                init_contact_normal_.setValue(0,0,-1);

                // heuristic decision of the push direction: only consider y axis
                // TODO: integrate with recognision and other task condition
                if(ee_pos.y() < flap_pos.y())
                  {
                    init_contact_point_ = flap_pos + tf::Vector3(0, - (opening_width_ / 2 - opening_margin_ * 2), - flap_height_ / 2);
                    target_end_ee_pose_.setOrigin(flap_pos + tf::Vector3(0, opening_width_ / 2 - opening_margin_, - flap_height_ / 2));
                  }
                else
                  {
                    init_contact_point_ = flap_pos + tf::Vector3(0, opening_width_ / 2 - opening_margin_ * 2, - flap_height_ / 2); // doule the opening_margin
                    target_end_ee_pose_.setOrigin(flap_pos + tf::Vector3(0, - (opening_width_ / 2 - opening_margin_), - flap_height_ / 2));
                  }
                target_reset_ee_pose_.setOrigin(flap_pos + tf::Vector3(0, 0, -0.5)); // heuristic

                // redefine the cog motion parameter for IK
                std::vector<double> cog_limit{0.05, 0.05, 0.01};
                ros::NodeHandle nh(nhp_, "end_effector");
                nh.setParam("differential_kinematics_constraint/cog_motion/cog_velocity_limit", cog_limit);
              }
            else
              {
                // lateral manipulation
                ROS_INFO("The flap object is under the robot, lateral manipulation");
                contact_reaction_force_ = 0; // no reaction force, which coincides to the friction
                contact_thresh_ *= 2; // workaround:  the contact trehshold is phase2 is relaxed.

                tf::Vector3 rel_pos = flap_pose_.inverse() * ee_pos;

                // four cases
                if(fabs(rel_pos.x()) > fabs(rel_pos.y()))
                  {
                    if(rel_pos.x() > 0)
                      init_contact_normal_ = flap_pose_.getBasis() * tf::Vector3(1, 0, 0);
                    else
                      init_contact_normal_ = flap_pose_.getBasis() * tf::Vector3(-1, 0, 0);
                  }
                else
                  {
                    if(rel_pos.y() > 0)
                      init_contact_normal_ = flap_pose_.getBasis() * tf::Vector3(0, 1, 0);
                    else
                      init_contact_normal_ = flap_pose_.getBasis() * tf::Vector3(0, -1, 0);
                  }

                init_contact_point_ = flap_pos + init_contact_normal_ * flap_width_ / 2;
                target_end_ee_pose_.setOrigin(flap_pos - init_contact_normal_ * (flap_width_ / 2 - opening_margin_));

                ee_orientation_flag_ = true;

                double pitch = M_PI / 4; // avoid the collision between end link leg and the misumi frame
                double yaw = atan2(-init_contact_normal_.y(), -init_contact_normal_.x());
                target_init_ee_pose_.setRotation(tf::createQuaternionFromRPY(0, pitch, yaw));
                target_end_ee_pose_.setRotation(target_init_ee_pose_.getRotation());
                target_reset_ee_pose_.setOrigin(flap_pos + tf::Vector3(0, 0, 0.05)); // heuristic

                tf::Quaternion opening_rot = opening_center_frame.getRotation();
                opening_center_frame.setRotation(opening_rot * tf::createQuaternionFromRPY(0, M_PI, 0)); // reverse from up-to-down traverse

                // set the return position via rosparam
                nhp_.setParam("final_pos_x", control_terms_.x.target_p);
                nhp_.setParam("final_pos_y", control_terms_.y.target_p);
                nhp_.setParam("final_yaw", tf::getYaw(robot_baselink_odom_.pose.pose.orientation));

                // redefine the cog motion parameter for IK
                std::vector<double> cog_limit{0.08, 0.08, 0.05};
                ros::NodeHandle nh(nhp_, "end_effector");
                nh.setParam("differential_kinematics_constraint/cog_motion/cog_velocity_limit", cog_limit);
              }

            // heuristical setting for squeezing
            opening_center_frame.getOrigin() += tf::Vector3(0, 0, flap_height_ / 2);
            SqueezeNavigation::path_planner_->setOpenningCenterFrame(opening_center_frame);

            // set the local contact normal w.r.t. the flap frame, which is fixed during the entire motion
            target_flap_yaw_ = tf::getYaw(flap_pose_.getRotation());


            /* set offset for contact appraoch */
            double approach_offset;
            nhp_.getParam("approach_offset", approach_offset);
            target_init_ee_pose_.setOrigin(init_contact_point_ + approach_offset * init_contact_normal_);

            ROS_INFO("Phase1: get contact approach point [%f, %f, %f]",
                     target_init_ee_pose_.getOrigin().x(),
                     target_init_ee_pose_.getOrigin().y(),
                     target_init_ee_pose_.getOrigin().z());

            // init state
            geometry_msgs::Pose cog_pose, root_pose;
            cog_pose.position.x = control_terms_.x.target_p;
            cog_pose.position.y = control_terms_.y.target_p;
            cog_pose.position.z = control_terms_.z.target_p;
            cog_pose.orientation.w = 1;
            tf::Quaternion desired_att = tf::createQuaternionFromRPY(0, 0, control_terms_.yaw.err_p + tf::getYaw(robot_baselink_odom_.pose.pose.orientation)); //CoG roll&pitch is zero, only yaw
            MultilinkState::convertCogPose2RootPose(robot_model_ptr_, desired_att, cog_pose, joint_state_, root_pose);
            tf::Transform root_tf;
            tf::poseMsgToTF(root_pose, root_tf);

            // get the path of IK
            if(!end_effector_ik_solver_->inverseKinematics(target_init_ee_pose_, robot_model_ptr_->kdlJointToMsg(joint_state_), root_tf, ee_orientation_flag_, true, std::string(""), std::string(""), false, false))
              {
                ROS_ERROR("flap manipulation: cannot get valid end effector ik pose in phase1");
                reset();
                break;
              }

            /* get continous path */
            double approach_trajectory_period;
            nhp_.param("approach_trajectory_period", approach_trajectory_period, 5.0);
            end_effector_ik_solver_->calcContinuousPath(approach_trajectory_period);

            /* start move imediately */
            move_flag_ = true;
            move_start_time_ = ros::Time::now().toSec();
          }

        // display ee pos in realtime
        if(move_flag_)
          {
            geometry_msgs::Vector3Stamped msg;
            msg.header = robot_baselink_odom_.header;
            tf::vector3TFToMsg(ee_pos, msg.vector);
            end_effector_pos_pub_.publish(msg);
          }
        else
          {
            if (move_start_time_ > 0)
            {
              tf::Vector3 d_pos = ee_pos - target_init_ee_pose_.getOrigin();

              // only check pos different around the init contact normal
              double diff = d_pos.cross(init_contact_normal_).length();
              ROS_INFO_THROTTLE(1, "phase1: end effector position is [%f, %f, %f], 3d diff is [%f, %f, %f], diff: %f, thresh: %f",
                                ee_pos.x(), ee_pos.y(), ee_pos.z(),
                                d_pos.x(), d_pos.y(), d_pos.z(),
                                diff, approach_thresh_);

              if(diff <= approach_thresh_)
                {
                  if(auto_state_machine_)
                    {
                      motion_phase_++;
                      plan_flag_ = true;
                      ROS_INFO("Shift to phase%d. final end effector position is [%f, %f, %f], diff is [%f, %f, %f]", motion_phase_, ee_pos.x(), ee_pos.y(), ee_pos.z(), d_pos.x(), d_pos.y(), d_pos.z());
                    }

                  ros::ServiceClient client = nh_.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");
                  if(client.exists())
                    {
                      gazebo_msgs::DeleteModel srv;
                      srv.request.model_name = std::string("flap");
                      if(client.call(srv))
                        {
                          if (robot_baselink_odom_.pose.pose.position.z < target_init_ee_pose_.getOrigin().z()) // heuristically remove opening frame, becuase of the collsioion between robot link and oepning frame
                            {
                              srv.request.model_name = std::string("opening");
                              client.call(srv);
                            }
                        }
                      else
                        ROS_ERROR("Failed to delete flap gazebo model");
                    }
                }
            }
          }
        break;
      }
    case PHASE2:
      {
        ROS_INFO_THROTTLE(2, "phase2: end effector position [%f, %f, %f], init contact point: [%f, %f, %f], contact_tresh: %f, contact normal: [%f, %f, %f]",
                          ee_pos.x(), ee_pos.y(), ee_pos.z(),
                          init_contact_point_.x(), init_contact_point_.y(), init_contact_point_.z(), contact_thresh_, init_contact_normal_.x(), init_contact_normal_.y(), init_contact_normal_.z());

        if(plan_flag_)
          {
            /* TODO1: change the joint. Right now, only move position of CoG */
            /* TODO2: diff_vec should be same with contact_offset */
            tf::Vector3 diff_vec = (init_contact_point_ - ee_pos).dot(init_contact_normal_) * init_contact_normal_;

            aerial_robot_msgs::FlightNav nav_msg;
            nav_msg.header.frame_id = std::string("/world");
            nav_msg.header.stamp = ros::Time::now();
            nav_msg.control_frame = nav_msg.WORLD_FRAME;
            nav_msg.target = nav_msg.COG;
            nav_msg.pos_xy_nav_mode = nav_msg.POS_MODE;
            nav_msg.target_pos_x = control_terms_.x.target_p + diff_vec.x();
            nav_msg.target_pos_y = control_terms_.y.target_p + diff_vec.y();
            nav_msg.pos_z_nav_mode = nav_msg.POS_MODE;
            nav_msg.target_pos_z = control_terms_.z.target_p + diff_vec.z();
            flight_nav_pub_.publish(nav_msg);

            plan_flag_ = false;
          }

        if((ee_pos - init_contact_point_).dot(init_contact_normal_) < contact_thresh_)
          {
            if(external_wrench_flag_)
              {
                ROS_WARN("apply contact reaction force in the end of phase2");
                aerial_robot_msgs::ApplyWrench msg;
                msg.name = "manipulte_force";
                msg.reference_frame = end_effector_ik_solver_->getParentSegName();
                tf::pointTFToMsg(end_effector_ik_solver_->getEndEffectorRelativePose().getOrigin(), msg.reference_point);
                tf::vector3TFToMsg(-init_contact_normal_ * contact_reaction_force_, msg.wrench.force); // the wrench generated by robot, thus the opposite to the contact reaction force
                apply_wrench_pub_.publish(msg);
              }

            if(auto_state_machine_)
              {
                motion_phase_++;
                plan_flag_ = true;
                ROS_INFO("Shift to phase%d", motion_phase_);
              }
          }
        break;
      }
    case PHASE3:
      {
        if (replan_du_ > 0 && ros::Time::now().toSec() - move_start_time_ >= replan_du_ && !plan_flag_)
          {
            plan_flag_ = true;
            ROS_INFO("flap manipulation: re-plan the trajectory and force");
          }

        if(plan_flag_)
          {
            plan_flag_ = false;


            if(replan_du_ > 0)
              {
                tf::Vector3 move_direction = (target_end_ee_pose_.getOrigin() - init_contact_point_).normalize();
                double diff = (target_end_ee_pose_.getOrigin() - ee_pos).dot(move_direction);
                ROS_INFO("diff with final point is %f, end effector pos: [%f, %f, %f]", diff, ee_pos.x(), ee_pos.y(), ee_pos.z());

                if(diff > reach_thresh_)
                  {
                    ROS_INFO("do re-plan for flap manipulation");

                    /* calculate the move vector */
                    // linear
                    if (diff > move_speed_ * replan_du_) diff = move_speed_ * replan_du_;
                    // yaw
                    tf::Vector3 orth_direction(move_direction.y(), -move_direction.x(), move_direction.z());
                    double yaw_err = target_flap_yaw_ - tf::getYaw(flap_pose_.getRotation());

                    //// debug, do not use in real machine!!!
                    //// yaw_err = target_flap_yaw_ - (tf::getYaw(flap_pose_.getRotation() * tf::createQuaternionFromYaw(-0.2)));
                    tf::Vector3 move_v = orth_direction * yaw_err * yaw_rate_ + move_direction * diff;

                    KDL::JntArray init_joint_array;
                    tf::Transform init_root_pose;
                    if(target_ee_pose_.getOrigin() == tf::Vector3(0,0,0))
                      {
                        // update from real end effector and robot state for the first time
                        //ROS_INFO("update from real end effector state");
                        target_ee_pose_.setOrigin(ee_pos + move_v);
                        target_ee_pose_.getOrigin().setZ(target_end_ee_pose_.getOrigin().getZ()); // the height should not change.

                        geometry_msgs::Pose root_pose;
                        MultilinkState::convertBaselinkPose2RootPose(robot_model_ptr_, robot_baselink_odom_.pose.pose, joint_state_, root_pose);
                        tf::poseMsgToTF(root_pose, init_root_pose);

                        init_joint_array = joint_state_;
                      }
                    else
                      {
                        // update from previous target end efector state
                        target_ee_pose_.getOrigin() += move_v;

                        MultilinkState state = end_effector_ik_solver_->getDiscretePath().back();
                        tf::poseMsgToTF(state.getRootPoseConst(), init_root_pose);
                        init_joint_array = state.getJointStateConst();
                      }
                    target_ee_pose_.setRotation(target_end_ee_pose_.getRotation()); // orientation is fixed

                    ROS_INFO("yaw_err: %f, total move vector: [%f, %f, %f], ee pose: [%f, %f, %f]", yaw_err, move_v.x(), move_v.y(), move_v.z(), target_ee_pose_.getOrigin().x(), target_ee_pose_.getOrigin().y(), target_ee_pose_.getOrigin().z());


                    if(!end_effector_ik_solver_->inverseKinematics(target_ee_pose_, robot_model_ptr_->kdlJointToMsg(init_joint_array), init_root_pose, ee_orientation_flag_, true, std::string(""), std::string(""), false, false))
                      {
                        ROS_ERROR("flap manipulation: cannot get valid end effector ik pose in phase3");
                        reset();
                        break;
                      }

                    end_effector_ik_solver_->calcContinuousPath(replan_du_);


                    /* apply external wrench for the contact force and moving friction */
                    if(external_wrench_flag_)
                      {
                        ROS_WARN("apply contact reaction force in the end of phase3");
                        tf::Vector3 friction_direction = -move_v.normalize(); // opposite to the moveing direction
                        aerial_robot_msgs::ApplyWrench msg;
                        msg.name = "manipulte_force";
                        msg.reference_frame = end_effector_ik_solver_->getParentSegName();
                        tf::pointTFToMsg(end_effector_ik_solver_->getEndEffectorRelativePose().getOrigin(), msg.reference_point);
                        tf::vector3TFToMsg(-init_contact_normal_ * contact_reaction_force_ - move_friction_force_ * friction_direction,
                                           msg.wrench.force); // opposite to the receive force (wrench generated by robot)
                        apply_wrench_pub_.publish(msg);

                        ROS_WARN("phase3: external force: [%f, %f, %f]",
                                 msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z);
                      }

                    /* start move imediately */
                    move_flag_ = true;
                    move_start_time_ = ros::Time::now().toSec();
                  }
                else
                  {
                    ROS_INFO("PHASE3: reach to the goal, finish replan");
                    move_flag_ = false;
                  }
              }
            else
              {
                ROS_INFO("only once plan for straight pushing without manipulation");
                geometry_msgs::Pose root_pose;
                MultilinkState::convertBaselinkPose2RootPose(robot_model_ptr_, robot_baselink_odom_.pose.pose, joint_state_, root_pose);
                tf::Transform root_tf;
                tf::poseMsgToTF(root_pose, root_tf);
                if(!end_effector_ik_solver_->inverseKinematics(target_end_ee_pose_, robot_model_ptr_->kdlJointToMsg(joint_state_), root_tf, ee_orientation_flag_, true, std::string(""), std::string(""), false, false))
                  {
                    ROS_ERROR("flap manipulation: cannot get valid end effector ik pose in phase3");
                    reset();
                    break;
                  }

                /* get continous path */
                double move_trajectory_period;
                nhp_.param("move_trajectory_period", move_trajectory_period, 5.0);
                end_effector_ik_solver_->calcContinuousPath(move_trajectory_period);

                /* apply external wrench for the contact force and moving friction */
                if(external_wrench_flag_)
                  {
                    ROS_WARN("apply contact reaction force in the end of phase3");
                    tf::Vector3 friction_direction = (init_contact_point_ - target_end_ee_pose_.getOrigin()).normalize(); // opposite to the moveing direction
                    aerial_robot_msgs::ApplyWrench msg;
                    msg.name = "manipulte_force";
                    msg.reference_frame = end_effector_ik_solver_->getParentSegName();
                    tf::pointTFToMsg(end_effector_ik_solver_->getEndEffectorRelativePose().getOrigin(), msg.reference_point);
                    tf::vector3TFToMsg(-init_contact_normal_ * contact_reaction_force_ - move_friction_force_ * friction_direction,
                                       msg.wrench.force); // opposite to the receive force (wrench generated by robot)
                    apply_wrench_pub_.publish(msg);

                    ROS_WARN("phase3: external force: [%f, %f, %f]",
                             msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z);
                  }

                /* start move imediately */
                move_flag_ = true;
                move_start_time_ = ros::Time::now().toSec();
              }

          }

        if(move_flag_)
          {
            geometry_msgs::Vector3Stamped msg;
            msg.header = robot_baselink_odom_.header;
            tf::vector3TFToMsg(ee_pos, msg.vector);
            end_effector_pos_pub_.publish(msg);
          }
        else
          {
            /* clear external wrench */
            std_msgs::String msg;
            msg.data = "manipulte_force";
            clear_wrench_pub_.publish(msg);

            if(auto_state_machine_)
              {
                motion_phase_++;
                plan_flag_ = true;
                ROS_INFO("Finish moving, shift to phase%d, clear external wrench", motion_phase_);
              }
          }

        break;
      }
    case PHASE4:
      {
        if(plan_flag_)
          {
            plan_flag_ = false;

            /* plan the end effector IK */
            geometry_msgs::Pose root_pose;
            MultilinkState::convertBaselinkPose2RootPose(robot_model_ptr_, robot_baselink_odom_.pose.pose, joint_state_, root_pose);
            tf::Transform root_tf;
            tf::poseMsgToTF(root_pose, root_tf);
            if(!end_effector_ik_solver_->inverseKinematics(target_reset_ee_pose_, robot_model_ptr_->kdlJointToMsg(joint_state_), root_tf, false, true, std::string(""), std::string(""), false, false))
              {
                ROS_ERROR("flap manipulation: cannot get valid end effector ik pose in phase4");
                reset();
                break;
              }

            /* get continous path */
            double reset_pose_period;
            nhp_.param("reset_pose_period", reset_pose_period, 5.0);
            end_effector_ik_solver_->calcContinuousPath(reset_pose_period);

            /* start move imediately */
            move_flag_ = true;
            move_start_time_ = ros::Time::now().toSec();
          }

        if(move_flag_)
          {
            geometry_msgs::Vector3Stamped msg;
            msg.header = robot_baselink_odom_.header;
            tf::vector3TFToMsg(ee_pos, msg.vector);
            end_effector_pos_pub_.publish(msg);
          }
        else
          {
            if(auto_state_machine_)
              {
                motion_phase_++;
                plan_flag_ = true;
                ROS_INFO("Shift to phase%d", motion_phase_);
              }
          }

        break;
      }
    case PHASE5:
      {
        if(!squeeze_flag_)
          {
            motion_phase_ = PHASE0;
            ROS_INFO("skip squeeze phase");
            break;
          }

        /* do planning */
        SqueezeNavigation::pathSearch();

        if(path_planner_->getDiscretePath().size() > 0)
          {
            move_flag_ = true;
            move_start_time_ = ros::Time::now().toSec();
            ROS_INFO("Start squeezing motion");
            motion_phase_++; // shift phase
          }
      }
    default:
      {
        break;
      }
    }

  /* navigation */
  if(move_flag_ || return_flag_)
    {
      if(motion_phase_ >= PHASE5) // this inequation is not good
        pathNavigate(path_planner_->getDiscretePath(), path_planner_->getContinuousPath());
      else
        pathNavigate(end_effector_ik_solver_->getDiscretePath(), end_effector_ik_solver_->getContinuousPath());
    }
}

void FlapManipulation::joyStickControl(const sensor_msgs::JoyConstPtr & joy_msg)
{
  using namespace aerial_robot_navigation;
  SqueezeNavigation::joyStickControl(joy_msg);

  if(joy_msg->axes.size() == BaseNavigator::PS4_AXES && joy_msg->buttons.size() == BaseNavigator::PS4_BUTTONS)
    {
      // shift phase
      if(joy_msg->buttons[BaseNavigator::PS4_BUTTON_REAR_LEFT_1] == 1 && prev_joy_cmd_.buttons[BaseNavigator::PS4_BUTTON_REAR_LEFT_1] == 0)
        {
          ROS_INFO("[flap manipulation], shift phase command from joystick");
          moveStartCallback(std_msgs::Empty());
        }

      // return home
      if(joy_msg->buttons[BaseNavigator::PS4_BUTTON_REAR_RIGHT_1] == 1 && prev_joy_cmd_.buttons[BaseNavigator::PS4_BUTTON_REAR_RIGHT_1] == 0)
        {
          ROS_INFO("[flap manipulation], return command from joystick");
          returnCallback(std_msgs::Empty());
        }
    }

  prev_joy_cmd_ = *joy_msg;
}


