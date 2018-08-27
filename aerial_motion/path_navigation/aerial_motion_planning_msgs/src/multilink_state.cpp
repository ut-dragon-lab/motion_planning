// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, JSK Lab
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

#include <aerial_motion_planning_msgs/multilink_state.h>

/* TODO: the orientation of CoG frame is not resolved!! */

void MultilinkState::cogPose2RootPose(boost::shared_ptr<TransformController> robot_model_ptr)
{
  /* the robot model ptr should  have the true baselink (e.g. fc) */
  assert(cog_update_ && baselink_desired_att_update_ && actuator_update_);

  KDL::Rotation kdl_q;
  tf::quaternionTFToKDL(baselink_desired_att_, kdl_q);
  robot_model_ptr->setCogDesireOrientation(kdl_q);
  robot_model_ptr->forwardKinematics(actuator_state_);

  tf::Transform cog_root = robot_model_ptr->getCog(); // cog in root frame

  /* root */
  tf::Transform cog_tf;
  tf::poseMsgToTF(cog_pose_, cog_tf);
  tf::Transform root_tf = cog_tf * robot_model_ptr->getCog().inverse();
  tf::poseTFToMsg(root_tf, root_pose_);

  root_update_ = true;
}

void MultilinkState::targetRootPose2TargetBaselinkPose(boost::shared_ptr<TransformController> robot_model_ptr)
{
  /* the robot model ptr should  have the true baselink (e.g. fc) */
  assert(root_update_ && actuator_update_);

  tf::Transform root_tf;
  tf::poseMsgToTF(root_pose_, root_tf);
  baselink_desired_att_ = root_tf * robot_model_ptr->getRoot2Link(robot_model_ptr->getBaselink(), actuator_state_).getRotation();
  baselink_desired_att_update_ = true;

  KDL::Rotation kdl_q;
  tf::quaternionTFToKDL(baselink_desired_att_, kdl_q);
  robot_model_ptr->setCogDesireOrientation(kdl_q); // necessary to update the true joint state (e.g., joint state)
  robot_model_ptr->forwardKinematics(actuator_state_);

  tf::pointTFToMsg(root_tf * robot_model_ptr->getCog().getOrigin(), cog_pose_.position);
  cog_update_ = true;
}

const std::vector<double> MultilinkState::getRootActuatorStateConst() const
{
  assert(root_update_ && actuator_update_);

  std::vector<double> states(0);
  states.push_back(root_pose_.position.x);
  states.push_back(root_pose_.position.y);
  states.push_back(root_pose_.position.z);
  states.push_back(root_pose_.orientation.x);
  states.push_back(root_pose_.orientation.y);
  states.push_back(root_pose_.orientation.z);
  states.push_back(root_pose_.orientation.w);

  for(auto itr: actuator_state_.position)
    states.push_back(itr);
  return states;
}

void MultilinkState::setRootActuatorState(const std::vector<double>& states)
{
  assert(states.size() == 7 + actuator_state_.name.size());
  assert(actuator_update_);

  root_pose_.position.x = states.at(0);
  root_pose_.position.y = states.at(1);
  root_pose_.position.z = states.at(2);

  root_pose_.orientation.x = states.at(3);
  root_pose_.orientation.y = states.at(4);
  root_pose_.orientation.z = states.at(5);
  root_pose_.orientation.w = states.at(6);

  actuator_state_.position.assign(states.begin() + 7, states.end());

  // for(int i = 0 ; i < actuator_state_.position.size(); i++)
  //   std::cout << actuator_state_.name.at(i) << ": " <<  actuator_state_.position.at(i) << ", ";
  // std::cout << std::endl;

  root_update_ = true;
  actuator_update_ = true;
}

