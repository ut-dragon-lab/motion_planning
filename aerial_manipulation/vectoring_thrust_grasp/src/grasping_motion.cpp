#include <vectoring_thrust_grasp/grasping_motion.h>

GraspingMotion::GraspingMotion(ros::NodeHandle nh, ros::NodeHandle nhp): nh_(nh), nhp_(nhp), once_flag_(true), flight_state_(0)
{
  robot_model_ = boost::make_shared<Dragon::HydrusLikeRobotModel>(true);
  planner_ = std::make_unique<GraspVectoringThrust>(nh_, nhp_, robot_model_, false /* realtime_control */);

  motion_phase_ = PHASE0;

  /* ros param */
  /** object inertial **/
  nhp_.param("measure_object_mass", measure_object_mass_, true);
  // get the known object inertia
  if(!measure_object_mass_)
    {
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
    }
  else
    {
      object_inertia_.m = 0;
      object_inertia_.ixx = 0;
      object_inertia_.ixy = 0;
      object_inertia_.ixz = 0;
      object_inertia_.iyy = 0;
      object_inertia_.iyz = 0;
      object_inertia_.izz = 0;
    }

  /* publisher / subscriber */
  start_sub_ = nh_.subscribe("/motion_start", 1, &GraspingMotion::startCallback, this);
  release_sub_ = nh_.subscribe("/release_object", 1, &GraspingMotion::releaseCallback, this);
  external_wrench_sub_ = nh_.subscribe("estimated_external_wrench", 1, &GraspingMotion::estimatedWrenchCallback, this);
  flight_state_sub_ = nh_.subscribe("flight_state", 1, &GraspingMotion::flightStateCallback, this);
  joy_sub_ = nh_.subscribe("joy", 1, &GraspingMotion::joyCallback, this);
  joint_control_pub_ = nh_.advertise<sensor_msgs::JointState>("joints_ctrl", 1);
  vectoring_force_pub_ = nh_.advertise<aerial_robot_msgs::ForceList>("extra_vectoring_force", 1);
  extra_module_pub_ = nh_.advertise<aerial_robot_msgs::ExtraModule>("add_extra_module", 1);


  nhp_.param("delta_angle", delta_angle_, 0.02);

  nhp_.param("contact_torque_thresh", contact_torque_thresh_, 2.0); // Nm

  nhp_.param("extra_module_comepsention_delay", extra_module_comepsention_delay_, 1.0); // s
  nhp_.param("estimate_mass_du", estimate_mass_du_, 1.0); // s

  /* hard-coding */
  nhp_.param("joint1_init_angle", joint1_init_angle_, 1.0);
  nhp_.param("joint3_init_angle", joint3_init_angle_, 1.0);
  joints_index_.resize(3, -1);

  /* motion phase timer */
  double motion_freq;
  nhp_.param("motion_freq", motion_freq, 20.0);
  motion_timer_ = nh_.createTimer(ros::Duration(1.0 / motion_freq), &GraspingMotion::stateMachine, this);
}


void GraspingMotion::joyCallback(const sensor_msgs::Joy::ConstPtr msg)
{
  if (msg->buttons[4] == 1 && motion_phase_ == PHASE0) // L1
    {
      // start
      ROS_INFO_STREAM("[Grasping Motion] start!!");
      motion_phase_ = PHASE1;
    }

  if (msg->buttons[5] == 1 && motion_phase_ != PHASE3) // R1
    {
      // release
      ROS_INFO_STREAM("[Grasping Motion] rlease!!");
      motion_phase_ = PHASE3;
    }
}


void GraspingMotion::flightStateCallback(const std_msgs::UInt8::ConstPtr msg)
{
  flight_state_ = msg->data;
}

void GraspingMotion::startCallback(const std_msgs::Empty msg)
{
  ROS_INFO_STREAM("[Grasping Motion] start!!");
  motion_phase_ = PHASE1;
}

void GraspingMotion::releaseCallback(const std_msgs::Empty msg)
{
  if(motion_phase_ != PHASE3)
    {
      ROS_INFO_STREAM("[Grasping Motion] rlease!!");
      motion_phase_ = PHASE3;
    }
}


void GraspingMotion::estimatedWrenchCallback(const geometry_msgs::WrenchStampedConstPtr msg)
{
  estimated_wrench_ = msg->wrench;
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
        if(flight_state_ != aerial_robot_navigation::HOVER_STATE)
          break;

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

            offset_force_z_ = estimated_wrench_.force.z;

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
        if(ros::Time::now().toSec() - phase_init_time_ >  extra_module_comepsention_delay_)
          {
            aerial_robot_msgs::ExtraModule msg;
            msg.header.stamp = ros::Time::now();
            msg.action = aerial_robot_msgs::ExtraModule::ADD;
            msg.module_name = std::string("target_object");
            msg.parent_link_name = std::string("link1");

            if (once_flag_)
              {
                /* calculate the com of the object w.r.t. root link */
                auto seg_tf_map = robot_model_->fullForwardKinematics(joint_angles);
                tf::vectorKDLToMsg((seg_tf_map.at("tail_ball").p - seg_tf_map.at("head_ball").p) / 2, object_offset_);
                ROS_INFO_STREAM("Object CoM w.r.t root link: [" << object_offset_.x << ", " << object_offset_.y << ", " << object_offset_.z << "]");
              }

            msg.transform.translation = object_offset_;
            msg.transform.rotation.w = 1;

            if(measure_object_mass_)
              {
                if(ros::Time::now().toSec() -  phase_init_time_ < estimate_mass_du_)
                  {
                    // TODO1: use external wrench instead of extra module, becuase the mass of robot_model_for_control would change.
                    double mass  = -(estimated_wrench_.force.z - offset_force_z_) / 9.8;
                    if (mass > object_inertia_.m) object_inertia_.m = mass; // max
                    if (object_inertia_.m < 0) object_inertia_.m = 0;

                    ROS_INFO("estimate mass: %f, mass: %f, estimated_wrench_.force.z: %f, offset_force_z_: %f", object_inertia_.m, mass, estimated_wrench_.force.z, offset_force_z_);

                    // TODO2: estimate the inertia of the object

                    msg.inertia = object_inertia_;
                    extra_module_pub_.publish(msg);
                  }
              }
            else
              {
                if (once_flag_)
                  {
                    msg.inertia = object_inertia_;
                    extra_module_pub_.publish(msg);
                    ROS_INFO("send extra module inertia");
                  }
              }
            once_flag_ = false;
          }

        if(flight_state_ != aerial_robot_navigation::HOVER_STATE)
          {
            ROS_INFO("force shift to PHASE3, becuase the flight is not at hovering state");
            motion_phase_ = PHASE3;
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
        aerial_robot_msgs::ForceList force_msg;
        force_msg.header.stamp = ros::Time::now();
        // empty force list means to clear the extra force
        vectoring_force_pub_.publish(force_msg);

        /* remove target object from flight control system */
        aerial_robot_msgs::ExtraModule module_msg;
        module_msg.header.stamp = ros::Time::now();
        module_msg.action = aerial_robot_msgs::ExtraModule::CLEAR;
        extra_module_pub_.publish(module_msg);

        motion_phase_ = PHASE0;

        break;
      }
    default:
      {
        break;
      }

    }
}
