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
#include <dragon/model/hydrus_like_robot_model.h> // TODO: change to full vectoring robot model

/* TODO1: the orientation of CoG frame is not resolved!! */
/* TODO2: provide a funcion of nominal_joint_angles for aerial_robot_mode, this does not only benefit dragon, but also other robot model. */

void MultilinkState::convertCogPose2RootPose(boost::shared_ptr<aerial_robot_model::RobotModel> robot_model_ptr,
                                             const tf::Quaternion& baselink_desired_att,
                                             const geometry_msgs::Pose& cog_pose,
                                             const KDL::JntArray& joint_state,
                                             geometry_msgs::Pose& root_pose)
{
  KDL::Rotation kdl_q;
  tf::quaternionTFToKDL(baselink_desired_att, kdl_q);
  robot_model_ptr->setCogDesireOrientation(kdl_q);
  robot_model_ptr->updateRobotModel(joint_state);

  /* root */
  tf::Transform cog_tf;
  tf::poseMsgToTF(cog_pose, cog_tf);
  tf::Transform root2cog_tf;
  tf::transformKDLToTF(robot_model_ptr->getCog<KDL::Frame>(), root2cog_tf);
  tf::Transform root_tf = cog_tf * root2cog_tf.inverse();
  tf::poseTFToMsg(root_tf, root_pose);

}

void MultilinkState::setStatesFromCog(boost::shared_ptr<aerial_robot_model::RobotModel> robot_model_ptr,
                                      const tf::Quaternion& baselink_desired_att,
                                      const geometry_msgs::Pose& cog_pose,
                                      const KDL::JntArray& joint_state)
{
  if(joint_index_map_.empty()) joint_index_map_ = robot_model_ptr->getJointIndexMap();

  /* set cog pose */
  baselink_desired_att_ = baselink_desired_att;
  cog_pose_ = cog_pose;

  /* set joint vector */
  joint_state_ = joint_state;

  /* set root pose */
  convertCogPose2RootPose(robot_model_ptr, baselink_desired_att, cog_pose, joint_state, root_pose_);

  /* special process for model with gimbal module */
  if(gimbal_module_flag_) joint_state_ = boost::dynamic_pointer_cast<Dragon::HydrusLikeRobotModel>(robot_model_ptr)->getGimbalProcessedJoint<KDL::JntArray>();
}

void MultilinkState::convertBaselinkPose2RootPose(boost::shared_ptr<aerial_robot_model::RobotModel> robot_model_ptr,
                                                  const geometry_msgs::Pose& baselink_pose,
                                                  const KDL::JntArray& joint_state,
                                                  geometry_msgs::Pose& root_pose)
{
  tf::Transform baselink_tf;
  tf::poseMsgToTF(baselink_pose, baselink_tf);
  tf::Transform root2baselink_tf;
  tf::transformKDLToTF(robot_model_ptr->forwardKinematics<KDL::Frame>(robot_model_ptr->getBaselinkName(), joint_state), root2baselink_tf);
  tf::Transform root_tf = baselink_tf * root2baselink_tf.inverse();

  tf::poseTFToMsg(root_tf, root_pose);
}

void MultilinkState::convertRootPose2CogPose(boost::shared_ptr<aerial_robot_model::RobotModel> robot_model_ptr,
                                             const geometry_msgs::Pose& root_pose,
                                             const KDL::JntArray& joint_state,
                                             tf::Quaternion& baselink_desired_att,
                                             geometry_msgs::Pose& cog_pose)
{
  tf::Transform root_tf;
  tf::poseMsgToTF(root_pose, root_tf);
  tf::Transform root2baselink_tf;
  tf::transformKDLToTF(robot_model_ptr->forwardKinematics<KDL::Frame>(robot_model_ptr->getBaselinkName(), joint_state), root2baselink_tf);
  baselink_desired_att = root_tf * root2baselink_tf.getRotation();

  KDL::Rotation kdl_q;
  tf::quaternionTFToKDL(baselink_desired_att, kdl_q);
  robot_model_ptr->setCogDesireOrientation(kdl_q);
  robot_model_ptr->updateRobotModel(joint_state);

  tf::Transform root2cog_tf;
  tf::transformKDLToTF(robot_model_ptr->getCog<KDL::Frame>(), root2cog_tf);
  tf::pointTFToMsg(root_tf * root2cog_tf.getOrigin(), cog_pose.position);
}

void MultilinkState::setStatesFromRoot(boost::shared_ptr<aerial_robot_model::RobotModel> robot_model_ptr,
                                       const geometry_msgs::Pose& root_pose,
                                       const KDL::JntArray& joint_state)
{
  if(joint_index_map_.empty()) joint_index_map_ = robot_model_ptr->getJointIndexMap();

  /* set cog pose */
  root_pose_ = root_pose;

  /* set joint vector */
  joint_state_ = joint_state;

  /* set root pose */
  convertRootPose2CogPose(robot_model_ptr, root_pose, joint_state, baselink_desired_att_, cog_pose_);

  /* special process for model with gimbal module */
  if(gimbal_module_flag_) joint_state_ = boost::dynamic_pointer_cast<Dragon::HydrusLikeRobotModel>(robot_model_ptr)->getGimbalProcessedJoint<KDL::JntArray>();
}

template<> const std::vector<double> MultilinkState::getRootJointStateConst() const
{
  std::vector<double> states(0);
  states.push_back(root_pose_.position.x);
  states.push_back(root_pose_.position.y);
  states.push_back(root_pose_.position.z);
  states.push_back(root_pose_.orientation.x);
  states.push_back(root_pose_.orientation.y);
  states.push_back(root_pose_.orientation.z);
  states.push_back(root_pose_.orientation.w);

  for(int i = 0; i < joint_state_.rows(); i++) states.push_back(joint_state_(i));

  return states;
}

template<> const moveit_msgs::RobotState MultilinkState::getRootJointStateConst() const
{
  moveit_msgs::RobotState robot_state;

  robot_state.joint_state.name.reserve(joint_index_map_.size());
  robot_state.joint_state.position.reserve(joint_index_map_.size());
  for(const auto& joint : joint_index_map_)
    {
      robot_state.joint_state.name.push_back(joint.first);
      robot_state.joint_state.position.push_back(joint_state_(joint.second));
    }

  robot_state.multi_dof_joint_state.header.frame_id = "world";
  robot_state.multi_dof_joint_state.joint_names.push_back("root");
  geometry_msgs::Transform root_pose;
  root_pose.translation.x = root_pose_.position.x;
  root_pose.translation.y = root_pose_.position.y;
  root_pose.translation.z = root_pose_.position.z;
  root_pose.rotation = root_pose_.orientation;

  robot_state.multi_dof_joint_state.transforms.push_back(root_pose);
  return robot_state;
}

double generateContinousEulerAngle(double ang, double prev_ang)
{
  if (fabs(ang - prev_ang) > M_PI)
    { // jumping gap in ang angle

      ROS_WARN("[euler angle conversion] find the angle gap, prev: %f, current: %f", prev_ang, ang);

      if (ang > prev_ang)
        {
          while (fabs(ang - prev_ang) > M_PI)
            { // Adjust ang
              ang -= 2 * M_PI;
              if (ang < prev_ang - 2 * M_PI)
                { // adjust overhead
                  ROS_ERROR("Could not find suitable ang. previous ang: %f, current ang: %f", prev_ang, ang);
                  ang += 2 * M_PI;
                  break;
                }
            }
        }
      else
        {
          while (fabs(ang - prev_ang) > M_PI)
            {
              ang += 2 * M_PI;
              if (ang > prev_ang + 2 * M_PI)
                {
                  ROS_ERROR("Could not find suitable ang. previous ang: %f, current ang: %f", prev_ang, ang);
                  ang -= 2 * M_PI;
                  break;
                }
            }
        }
    }

  return ang;
}
