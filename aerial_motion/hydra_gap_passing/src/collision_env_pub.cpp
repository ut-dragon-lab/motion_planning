#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
// MoveIt!
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>

int main(int argc, char **argv)
{
  ros::init (argc, argv, "collision_env_pub");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle;

  ros::Publisher collision_object_publisher = node_handle.advertise<moveit_msgs::CollisionObject>("collision_object", 1);
  while(collision_object_publisher.getNumSubscribers() < 1)
    {
      ros::WallDuration sleep_t(0.5);
      sleep_t.sleep();
    }
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = "world";
  collision_object.id = "box";
  /* A default pose */
  geometry_msgs::Pose pose;
  pose.position.x = 2.0;
  pose.position.y = 0.0;
  pose.position.y = 3.0;
  pose.orientation.w = 1.0;
  /* Define a box to be attached */
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 1;
  primitive.dimensions[1] = 1;
  primitive.dimensions[2] = 1;
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(pose);
  collision_object.operation = collision_object.ADD;
  ROS_INFO("Adding the object into the world at the location of the right wrist.");
  collision_object_publisher.publish(collision_object);
  ros::shutdown();
  return 0;
}
