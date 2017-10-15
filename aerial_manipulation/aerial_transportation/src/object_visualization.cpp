// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, JSK Lab
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

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>

using namespace std;

class ObjectVisualization
{
public:
  ObjectVisualization(ros::NodeHandle nh, ros::NodeHandle nhp):
    nh_(nh), nhp_(nhp)
  {
    string obj_marker_pub_name, uav_state_sub_name, object_pos_sub_name;

    nhp_.param("obj_marker_pub_name", obj_marker_pub_name, string("obj_marker"));
    nhp_.param("uav_state_sub_name", uav_state_sub_name, string("uav_state"));
    nhp_.param("object_pos_sub_name", object_pos_sub_name, string("object_pos"));

    nhp_.param("link_name", link_name_, string("link"));
    nhp_.param("obj_name", obj_name_, string("obj"));

    nhp_.param("obj_diameter", obj_diameter_, 0.25);

    obj_marker_pub_ = nh_.advertise<visualization_msgs::Marker>(obj_marker_pub_name, 1);
    uav_state_sub_ = nh_.subscribe(uav_state_sub_name, 1, &ObjectVisualization::uavStateCallback, this);
    object_pos_sub_ = nh_.subscribe(object_pos_sub_name, 1, &ObjectVisualization::objectPoseCallback, this);

  }

  ~ObjectVisualization(){}
private:

  ros::NodeHandle nh_, nhp_;
  tf::TransformBroadcaster br_;

  ros::Subscriber uav_state_sub_;
  ros::Subscriber object_pos_sub_;
  ros::Publisher obj_marker_pub_;

  string link_name_, obj_name_;
  tf::Transform w_uav_tf_;

  double obj_diameter_;

  ros::Time stamp_;
  void uavStateCallback(const nav_msgs::OdometryConstPtr & msg)
  {
    tf::poseMsgToTF(msg->pose.pose, w_uav_tf_);
    stamp_ = msg->header.stamp;
  }

  void objectPoseCallback(const geometry_msgs::Pose2DConstPtr & object_msg)
  {
    tf::Transform w_obj_tf(tf::createQuaternionFromYaw(object_msg->theta),
                           tf::Vector3(object_msg->x, object_msg->y, 0));

    tf::Transform uav_obj_tf = w_uav_tf_.inverse() * w_obj_tf;
    br_.sendTransform(tf::StampedTransform(uav_obj_tf, ros::Time::now(), link_name_, obj_name_));

    visualization_msgs::Marker marker_msg;
    marker_msg.header.stamp = stamp_;
    marker_msg.header.frame_id = link_name_;
    marker_msg.ns = "object";
    marker_msg.type = visualization_msgs::Marker::CYLINDER;
    marker_msg.action = visualization_msgs::Marker::ADD;
    marker_msg.pose.position.x = uav_obj_tf.getOrigin().x();
    marker_msg.pose.position.y = uav_obj_tf.getOrigin().y();
    marker_msg.pose.position.z = 0;
    marker_msg.pose.orientation.w = 1.0;
    marker_msg.scale.x = obj_diameter_;
    marker_msg.scale.y = obj_diameter_;
    marker_msg.scale.z = 0.3;
    marker_msg.color.r = 1.0;
    marker_msg.color.a = 0.5;
    obj_marker_pub_.publish(marker_msg);
  }
};

int main (int argc, char **argv)
{
  ros::init (argc, argv, "object_visualization");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");


  boost::shared_ptr<ObjectVisualization> obj_visualize_node(new ObjectVisualization(nh, nhp));

    ros::spin();
}
