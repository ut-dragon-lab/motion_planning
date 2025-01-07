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

#ifndef DIFFERENTIAL_KINEMATICS_PLANNER_H
#define DIFFERENTIAL_KINEMATICS_PLANNER_H

/* ros */
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

/* robot model */
#include <aerial_robot_model/model/transformable_aerial_robot_model.h>
#include <aerial_motion_planning_msgs/multilink_state.h>

/* for QP solution for force-closure */
#include <qpOASES.hpp>
USING_NAMESPACE_QPOASES

namespace differential_kinematics
{
  namespace cost
  {
    class Base;
  };
  namespace constraint
  {
    class Base;
  };

  class Planner
  {
    using CostContainer = std::vector<boost::shared_ptr<cost::Base> >;
    using ConstraintContainer = std::vector<boost::shared_ptr<constraint::Base> >;

  public:
    Planner(ros::NodeHandle nh, ros::NodeHandle nhp, boost::shared_ptr<aerial_robot_model::transformable::RobotModel> robot_model_ptr);
    ~Planner(){}

    bool solver(CostContainer cost_container, ConstraintContainer constraint_container, bool debug);

    void registerUpdateFunc(std::function<bool(void)> new_func);
    void registerMotionFunc(std::function<void(void)> new_func);

    /* kinematics */
    boost::shared_ptr<aerial_robot_model::transformable::RobotModel> getRobotModelPtr() {return robot_model_ptr_;}
    template<class T> const T getTargetRootPose() const ;
    template<class T> const T getTargetJointVector() const;
    template<class T> void setTargetRootPose(const T target_root_pose);
    template<class T> void setTargetJointVector(const T target_joint_vector);
    const int getMultilinkType() const {return multilink_type_;}

    const std::vector<KDL::Frame>& getRootPoseSequence() const {return target_root_pose_sequence_;}
    const std::vector<KDL::JntArray>& getJointStateSequence() const {return target_joint_vector_sequence_;}

    /* special for gimbal model */
    const bool getGimbalModuleFlag() const { return gimbal_module_flag_; }
  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;
    ros::Publisher joint_state_pub_;
    tf::TransformBroadcaster br_;
    ros::Timer  motion_timer_;

    /* robot model for kinematics */
    boost::shared_ptr<aerial_robot_model::transformable::RobotModel> robot_model_ptr_;
    std::string tf_prefix_;
    uint8_t multilink_type_;
    bool gimbal_module_flag_; // TODO: hard-coding

    /* result  */
    KDL::JntArray target_joint_vector_;
    KDL::Frame target_root_pose_;
    bool solved_;
    int differential_kinematics_count_;

    int sequence_;
    std::vector<KDL::JntArray> target_joint_vector_sequence_;
    std::vector<KDL::Frame> target_root_pose_sequence_;

    /* update function for each loop of the differential kinematics */
    std::vector< std::function<bool(void)> > update_func_vector_;
    /* update function for the result motion, mainly for visualization */
    std::vector< std::function<void(void)> > motion_func_vector_;

    void motionFunc(const ros::TimerEvent & e);
  };

};

#endif
