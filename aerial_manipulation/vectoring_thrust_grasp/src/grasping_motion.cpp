#include <vectoring_thrust_grasp/grasping_motion.h>

namespace
{
  /* TODO: ad-hoc for quad dragon to grasp object */
  std::vector<int> joints_index_;
  double delta_angle_ = 0;
  double contact_torque_thresh_ = 0;
  double phase_init_time_ = 0;
  double extra_module_comepsention_delay_ = 0;
  bool once_flag_ = true;
  // sensor_msgs::JointState init_joint_angles_;
  double joint1_init_angle_ = 0;
  double joint3_init_angle_ = 0;

};

GraspingMotion::GraspingMotion(ros::NodeHandle nh, ros::NodeHandle nhp): nh_(nh), nhp_(nhp)
{
  robot_model_ = boost::make_shared<Dragon::HydrusLikeRobotModel>(true);
  planner_ = std::make_unique<GraspVectoringThrust>(nh_, nhp_, robot_model_, false /* realtime_control */);

  motion_phase_ = PHASE0;

  /* ros param */
  /** object inertial **/
  nhp_.param("object_mass", object_inertia_.m, 1.0);
  std::vector<double> inertial_tensor;
  nhp_.getParam("object_inertial_tensor", inertial_tensor);
  if(inertial_tensor.size() != 6) ROS_ERROR("The size of inertial tensor is wrong: %d vs 6", (int)inertial_tensor.size());
  object_inertia_.ixx = inertial_tensor.at(0);
  object_inertia_.ixy = inertial_tensor.at(1);
  object_inertia_.ixz = inertial_tensor.at(2);
  object_inertia_.iyy = inertial_tensor.at(3);
  object_inertia_.iyz = inertial_tensor.at(4);
  object_inertia_.izz = inertial_tensor.at(5);
  double motion_freq;
  nhp_.param("motion_freq", motion_freq, 20.0);

  /* publisher / subscriber */
  start_sub_ = nh_.subscribe("/motion_start", 1, &GraspingMotion::startCallback, this);
  release_sub_ = nh_.subscribe("/release_object", 1, &GraspingMotion::releaseCallback, this);
  joint_control_pub_ = nh_.advertise<sensor_msgs::JointState>("/joints_ctrl", 1);
  vectoring_force_pub_ = nh_.advertise<aerial_robot_msgs::ForceList>("/grasp_vectoring_force", 1);

  /* hard-codring */
  nhp_.param("delta_angle", delta_angle_, 0.02);
  //delta_angle_ /= motion_freq; // rad/iteration. not necessary!

  nhp_.param("contact_torque_thresh", contact_torque_thresh_, 2.0); // Nm
  nhp_.param("extra_module_comepsention_delay", extra_module_comepsention_delay_, 1.0); // m
  nhp_.param("joint1_init_angle", joint1_init_angle_, 1.0);
  nhp_.param("joint3_init_angle", joint3_init_angle_, 1.0);
  joints_index_.resize(3, -1);

  /* motion phase timer */
  motion_timer_ = nh_.createTimer(ros::Duration(1.0 / motion_freq), &GraspingMotion::stateMachine, this);
}


void GraspingMotion::startCallback(const std_msgs::Empty msg)
{
  ROS_INFO_STREAM("[Grasping Motion] start!!");
  motion_phase_ = PHASE1;
}

void GraspingMotion::releaseCallback(const std_msgs::Empty msg)
{
  if(motion_phase_ == PHASE2)
    {
      ROS_INFO_STREAM("[Grasping Motion] rlease!!");
      motion_phase_ = PHASE3;
    }
}

/* TODO: ad-hoc for quad dragon to grasp object */
void GraspingMotion::stateMachine(const ros::TimerEvent& event)
{
  if(robot_model_->getMass() == 0)
    {
      ROS_WARN_THROTTLE(1.0, "the robot model is not updated");
      return;
    }

  auto joint_angles = planner_->getCurrJointState();

  if(joints_index_.at(0) == -1)
    {
      for(int i = 0; i < joint_angles.name.size(); i++)
        {
          for(int j = 0; j < 3; j++)
            {
              if(joint_angles.name.at(i) == std::string("joint") + std::to_string(j+1) + std::string("_yaw")) joints_index_.at(j) = i;
            }
        }
    }

  ROS_INFO_THROTTLE(0.5, "joint torque: [%f, %f, %f]",
                    joint_angles.effort.at(joints_index_.at(0)),
                    joint_angles.effort.at(joints_index_.at(1)),
                    joint_angles.effort.at(joints_index_.at(2))); // debug

  switch(motion_phase_)
    {
    case PHASE0:
      {
        if(once_flag_)
          {
            ros::Duration(0.1).sleep(); 
            sensor_msgs::JointState joint_control_msg;
            joint_control_msg.header.stamp = ros::Time::now();
            joint_control_msg.name.push_back(std::string("joint1_yaw"));
            joint_control_msg.position.push_back(joint1_init_angle_);
            joint_control_msg.name.push_back(std::string("joint3_yaw"));
            joint_control_msg.position.push_back(joint3_init_angle_);
            joint_control_pub_.publish(joint_control_msg);

            ros::Duration(0.1).sleep();
            joint_control_pub_.publish(joint_control_msg);

            once_flag_ = false;
          }
        break;
      }
    case PHASE1:
      {
        sensor_msgs::JointState joint_control_msg;
        joint_control_msg.header.stamp = ros::Time::now();

        /* check the grasping */
        /* ad-hoc:
           Lev1: check center joint (e.g. joint2)
           Lev2: check the joint torque vector
        */
        if(joint_angles.effort.at(joints_index_.at(1)) >= contact_torque_thresh_)
          {
            ROS_WARN_STREAM("find large torque load in joint2_yaw: " << joint_angles.effort.at(joints_index_.at(1)) << ", detect contact at ends, hold object by vectoring force");

            /* relax the joint torque */
            joint_control_msg.name.push_back(std::string("joint1_yaw"));
            joint_control_msg.position.push_back(joint_angles.position.at(joints_index_.at(0)));
            joint_control_msg.name.push_back(std::string("joint3_yaw"));
            joint_control_msg.position.push_back(joint_angles.position.at(joints_index_.at(2)));
            joint_control_pub_.publish(joint_control_msg);

            /* do vectoring force planning and realtime control */
            planner_->setRealtimeControl(true);

            motion_phase_ = PHASE2;
            once_flag_ = true;
            phase_init_time_ = ros::Time().now().toSec();

            return;
          }

        /* change the joint angle */
        if(joint_angles.position.at(joints_index_.at(0)) < M_PI/2)
          {
            joint_control_msg.name.push_back(std::string("joint1_yaw"));
            joint_control_msg.position.push_back(joint_angles.position.at(joints_index_.at(0)) + delta_angle_);
          }
        if(joint_angles.position.at(joints_index_.at(2)) < M_PI/2)
          {
            joint_control_msg.name.push_back(std::string("joint3_yaw"));
            joint_control_msg.position.push_back(joint_angles.position.at(joints_index_.at(2)) + delta_angle_);
          }

        if(joint_control_msg.name.size() > 0) joint_control_pub_.publish(joint_control_msg);

        break;
      }
    case PHASE2:
      {
        if(once_flag_ && ros::Time::now().toSec() - phase_init_time_ >  extra_module_comepsention_delay_)
          {
            /* apply external wrench for the vertical contact force */
            ros::ServiceClient client = nh_.serviceClient<aerial_robot_model::AddExtraModule>("/dragon/add_extra_module");
            aerial_robot_model::AddExtraModule srv;
            srv.request.action = aerial_robot_model::AddExtraModule::Request::ADD;
            srv.request.module_name = std::string("target_object");
            srv.request.parent_link_name = std::string("link1");

            /* calculate the com of the object w.r.t. root link */
            auto seg_tf_map = robot_model_->fullForwardKinematics(joint_angles);
            tf::vectorKDLToMsg((seg_tf_map.at("tail_ball").p - seg_tf_map.at("head_ball").p) / 2, srv.request.transform.translation);
            srv.request.transform.rotation.w = 1;
            ROS_INFO_STREAM("Object CoM w.r.t root link: [" << srv.request.transform.translation.x << ", " << srv.request.transform.translation.y << ", " << srv.request.transform.translation.z << "]");
            srv.request.inertia = object_inertia_;

            client.call(srv);

            ROS_WARN("Apply target objecto to the flight control system");

            once_flag_ = false;
          }

        break;
      }
    case PHASE3:
      {
        /* relase object  */
        sensor_msgs::JointState joint_control_msg;
        joint_control_msg.header.stamp = ros::Time::now();
        joint_control_msg.name.push_back(std::string("joint1_yaw"));
        joint_control_msg.position.push_back(joint1_init_angle_);
        joint_control_msg.name.push_back(std::string("joint3_yaw"));
        joint_control_msg.position.push_back(joint3_init_angle_);
        joint_control_pub_.publish(joint_control_msg);

        /* clear the vectoring force for grasping */
        planner_->setRealtimeControl(false);
        aerial_robot_msgs::ForceList msg;
        msg.header.stamp = ros::Time::now();
        // empty force list means to clear the extra force
        vectoring_force_pub_.publish(msg);

        /* remove target object from flight control system */
        ros::ServiceClient client = nh_.serviceClient<aerial_robot_model::AddExtraModule>("/dragon/add_extra_module");
        aerial_robot_model::AddExtraModule srv;
        srv.request.action = aerial_robot_model::AddExtraModule::Request::REMOVE;
        srv.request.module_name = std::string("target_object");
        srv.request.parent_link_name = std::string("root");
        client.call(srv);


        motion_phase_ = PHASE0;

        break;
      }
    default:
      {
        break;
      }

    }
}
