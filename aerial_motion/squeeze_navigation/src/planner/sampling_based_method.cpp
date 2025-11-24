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

#include <squeeze_navigation/planner/base_plugin.h>
#include <sampling_based_method/motion_planning.h>

namespace squeeze_motion_planner
{
  class SamplingBasedMethod : public Base
  {
  public:
    SamplingBasedMethod(){}
    ~SamplingBasedMethod() {}

    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp, boost::shared_ptr<aerial_robot_model::transformable::RobotModel> robot_model_ptr)
    {
      Base::initialize(nh, nhp, robot_model_ptr);

      planner_core_ = boost::shared_ptr<sampling_base::MotionPlanning>(new sampling_base::MotionPlanning(nh, nhp, robot_model_ptr_));

      planner_core_->setScene(setCollisionWorld());
    };

    void setInitState(const MultilinkState& state)  { planner_core_->setStartState(state); }

    moveit_msgs::CollisionObject setCollisionWorld()
    {
      moveit_msgs::CollisionObject collision_object;

      /* set env */
      collision_object.header.frame_id = "world";
      collision_object.id = "box";

      geometry_msgs::Pose wall_pose;
      wall_pose.orientation.w = 1.0;
      wall_pose.position.z = 0.0;
      shape_msgs::SolidPrimitive wall_primitive;
      wall_primitive.type = wall_primitive.BOX;
      wall_primitive.dimensions.resize(3);

      /* gap type 1: vertical gap */
      int gap_type ;
      nhp_.param("gap_type", gap_type, 0); //Type0: vertical; Type1: horizontal
      if(gap_type == HORIZONTAL_GAP)
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
      if(gap_type == VERTICAL_GAP)
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
              double ceiling_offset, side_wall_width;
              nhp_.param("ceiling_offset", ceiling_offset, 0.6);
              nhp_.param("side_wall_width", side_wall_width, 2.0);

              /* planning mode */
              /* ceil */
              wall_pose.position.x = 0;
              wall_pose.position.y = 0;
              wall_pose.position.z = ceiling_offset + gap_height;
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
              wall_pose.position.x = side_wall_width / 2;
              wall_pose.position.y = 0;
              wall_pose.position.z = gap_height / 2;
              wall_primitive.dimensions[0] = 0.05;
              wall_primitive.dimensions[1] = side_wall_width;
              wall_primitive.dimensions[2] = gap_height;
              collision_object.primitives.push_back(wall_primitive);
              collision_object.primitive_poses.push_back(wall_pose);

              wall_pose.position.x = - side_wall_width / 2;
              collision_object.primitives.push_back(wall_primitive);
              collision_object.primitive_poses.push_back(wall_pose);

              wall_pose.position.x = 0;
              wall_pose.position.y = side_wall_width / 2;
              wall_primitive.dimensions[0] = side_wall_width;
              wall_primitive.dimensions[1] = 0.05;
              collision_object.primitives.push_back(wall_primitive);
              collision_object.primitive_poses.push_back(wall_pose);

              wall_pose.position.x = 0;
              wall_pose.position.y = -side_wall_width / 2;
              collision_object.primitives.push_back(wall_primitive);
              collision_object.primitive_poses.push_back(wall_pose);
            }
        }
      collision_object.operation = collision_object.ADD;

      return collision_object;
    }

    const std::vector<MultilinkState>& getPathConst() const
    {
      return planner_core_->getPathConst();
    }

    const MultilinkState& getStateConst(int index) const
    {
      return planner_core_->getStateConst(index);
    }

    bool corePlan ()
    {
      return planner_core_->plan();
    }

    bool loadPath() { return planner_core_->loadPath();}

    void visualizeFunc() {}
    void checkCollision(MultilinkState state) { planner_core_->checkCollision(state); }

  private:
    boost::shared_ptr<sampling_base::MotionPlanning> planner_core_;
  };
};

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(squeeze_motion_planner::SamplingBasedMethod, squeeze_motion_planner::Base);
