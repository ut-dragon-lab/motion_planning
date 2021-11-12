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

#ifndef FLAP_MANIPULATION_H_
#define FLAP_MANIPULATION_H_

/* ros */
#include <ros/ros.h>
#include <aerial_robot_msgs/ApplyWrench.h>
#include <std_msgs/String.h>
#include <squeeze_navigation/planner/base_plugin.h>
#include <differential_kinematics/motion/end_effector_ik_solver_core.h>
#include <squeeze_navigation/squeeze_navigation.h>
#include <gazebo_msgs/DeleteModel.h>


enum
  {
    PHASE0, // init phase (hovering)
    PHASE1, // approach under the contact point on the cover (need planning based on differential kinematics)
    PHASE2, // contact to the cover with external wrench (consider the weight of the cover)
    PHASE3, // move the cover (need planning based on differential kinematics)
    PHASE4, // move away from the cover
    PHASE5, // squeeze (need planning based on differential kinematics)
  };


class FlapManipulation :public SqueezeNavigation {
public:
  FlapManipulation(ros::NodeHandle nh, ros::NodeHandle nhp);
  ~FlapManipulation(){}

private:
  ros::Subscriber flap_pose_sub_;
  ros::Publisher apply_wrench_pub_;
  ros::Publisher clear_wrench_pub_;
  ros::Publisher end_effector_pos_pub_;

  int motion_phase_;
  tf::Transform flap_pose_;

  tf::Vector3 init_contact_point_, init_contact_normal_;
  tf::Transform target_init_ee_pose_, target_end_ee_pose_, target_reset_ee_pose_;
  bool ee_orientation_flag_;
  double contact_reaction_force_;
  double approach_offset_;
  double approach_thresh_, contact_thresh_, reach_thresh_;

  // replaning in phase3 for flap manipulation
  double target_flap_yaw_;
  double yaw_rate_;
  tf::Transform  target_ee_pose_;

  bool auto_state_machine_;
  bool squeeze_flag_;
  bool external_wrench_flag_;

  double replan_du_;
  double move_speed_;
  double move_friction_force_;
  double flap_width_, flap_height_;
  double opening_width_, opening_margin_;

  /* end-effector ik solver */
  boost::shared_ptr<EndEffectorIKSolverCore> end_effector_ik_solver_;

  // teleop
  sensor_msgs::Joy prev_joy_cmd_;

  void rosParamInit();
  void reset() override;
  void process(const ros::TimerEvent& event) override;
  void moveStartCallback(const std_msgs::Empty msg) override;
  void returnCallback(const std_msgs::Empty msg) override;
  void joyStickControl(const sensor_msgs::JoyConstPtr & joy_msg) override;

  void flapPoseCallback(const geometry_msgs::PoseStampedConstPtr msg);

};

#endif
