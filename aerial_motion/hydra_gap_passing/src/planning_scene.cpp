
#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <tf/transform_broadcaster.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit/kinematic_constraints/utils.h>

class SinpleGapEnv
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  planning_scene::PlanningScene* planning_scene_;
  
};

class PlanningScene
{
private:

  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  ros::Publisher planning_scene_diff_pub_;
  ros::Subscriber pos_changed_sub_;
  float z_;

  robot_model_loader::RobotModelLoader *robot_model_loader;
  robot_model::RobotModelPtr kinematic_model;
  planning_scene::PlanningScene* planning_scene;

  moveit_msgs::CollisionObject collision_object;
  moveit_msgs::PlanningScene planning_scene_msg;
  collision_detection::AllowedCollisionMatrix acm;

  tf::TransformBroadcaster br;

  double tf_pub_rate_, collision_detection_rate_;
  ros::Timer collision_detection_timer_,tf_pub_timer_;

  void collisionFunc(const ros::TimerEvent &e)
    {
      collision_detection::CollisionRequest collision_request;
      collision_detection::CollisionResult collision_result;
      robot_state::RobotState& current_state = planning_scene->getCurrentStateNonConst();
      //robot_state::RobotState copied_state = planning_scene->getCurrentState();      
      planning_scene->checkCollision(collision_request, collision_result, current_state, acm);
      ROS_INFO_STREAM("Test 6: Current state is "
                      << (collision_result.collision ? "in" : "not in")
                      << " self collision");

      std::vector<double> joint_values;
      //const robot_state::JointStateGroup* joint_model_group = current_state.getJointStateGroup("body");
      //joint_model_group->getVariableValues(joint_values);
      current_state.getStateValues(joint_values);
      for(int i = 0; i < joint_values.size(); i++)
        {
          ROS_INFO("no%d, %f", i, joint_values[i]);
        }

      joint_values[0] += 0.01;
      current_state.setStateValues(joint_values);
      //joint_model_group->setVariableValues(joint_values);

      planning_scene->getPlanningSceneMsg(planning_scene_msg);
      planning_scene_msg.is_diff = true;
      planning_scene_diff_pub_.publish(planning_scene_msg);

    }

  void tfPubFunc(const ros::TimerEvent &e)
  {
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(0.0, 0.0, 1.0) );
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    transform.setRotation(q);

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "head_cylinder"));
  }

  void moveCallback(const std_msgs::Int8 msg)
  {
    if(msg.data == -1) z_ += 0.1;
    else if(msg.data == 1) z_ -= 0.1;
  }


public:

  PlanningScene(ros::NodeHandle nh, ros::NodeHandle nhp)
    :nh_(nh), nhp_(nhp)
  {
    planning_scene_diff_pub_ = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    pos_changed_sub_ = nh_.subscribe<std_msgs::Int8>("/teleop_command/joints_ctrl",1, &PlanningScene::moveCallback, this, ros::TransportHints().tcpNoDelay());

  while(planning_scene_diff_pub_.getNumSubscribers() < 1)
    {
      ros::WallDuration sleep_t(0.5);
      sleep_t.sleep();
    }
  ros::Duration sleep_time(1.0);
  sleep_time.sleep();


  robot_model_loader = new robot_model_loader::RobotModelLoader("hydra/robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader->getModel();
  planning_scene = new planning_scene::PlanningScene(kinematic_model);

  //pub the collision object
  collision_object.header.frame_id = "world";
  collision_object.id = "box";
  geometry_msgs::Pose pose;
  pose.position.x = 0.0;
  pose.position.y = 1.0;
  pose.position.z = 0.0;
  pose.orientation.w = 1.0;
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 1;
  primitive.dimensions[1] = 0.1;
  primitive.dimensions[2] = 1;
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(pose);
  collision_object.operation = collision_object.ADD;

  planning_scene->processCollisionObjectMsg(collision_object);

  z_ = 3.0;
  
  tf_pub_rate_ = 100;
  collision_detection_rate_ = 20;

  acm = planning_scene->getAllowedCollisionMatrix();
  //collision_request.contacts = true;
  //collision_request.max_contacts = 1000;

  collision_detection_timer_ = nhp_.createTimer(ros::Duration(1.0 / collision_detection_rate_), &PlanningScene::collisionFunc, this);

  tf_pub_timer_ = nhp_.createTimer(ros::Duration(1.0 / tf_pub_rate_), &PlanningScene::tfPubFunc, this);

  }


 
  ~PlanningScene()
  {
    delete robot_model_loader;
    delete planning_scene;
  }


};

int main(int argc, char **argv)
{
  ros::init (argc, argv, "hydra_collision_detection");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  
  PlanningScene *planning_scene = new PlanningScene(nh,nhp);
  ros::spin();
  delete planning_scene;


 return 0;
}
