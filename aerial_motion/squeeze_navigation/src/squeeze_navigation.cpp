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

#include <squeeze_navigation/squeeze_navigation.h>
#include <tf/transform_broadcaster.h>

/* end effector IK */
#include <differential_kinematics/motion/end_effector_ik_solver_core.h>

SqueezeNavigation::SqueezeNavigation(ros::NodeHandle nh, ros::NodeHandle nhp):
  nh_(nh), nhp_(nhp), plugin_loader_("squeeze_navigation", "squeeze_motion_planner::Base"), real_machine_connect_(false)
{
  rosParamInit();

  /* ros pub/sub, srv */
  plan_flag_sub_ = nh_.subscribe("plan_start", 1, &SqueezeNavigation::planStartCallback, this);
  move_flag_sub_ = nh_.subscribe("move_start", 1, &SqueezeNavigation::moveStartCallback, this);
  return_flag_sub_ = nh_.subscribe("return", 1, &SqueezeNavigation::returnCallback, this);
  joy_stick_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 1, &SqueezeNavigation::joyStickControl, this);

  robot_baselink_odom_sub_ = nh_.subscribe("uav/baselink/odom", 1, &SqueezeNavigation::robotOdomCallback, this);
  robot_joint_states_sub_ = nh_.subscribe("joint_states", 1, &SqueezeNavigation::robotJointStatesCallback, this);
  control_terms_sub_ = nh_.subscribe("debug/pose/pid", 1, &SqueezeNavigation::controlTermsCallback, this);


  joints_ctrl_pub_ = nh_.advertise<sensor_msgs::JointState>("joints_ctrl", 1);
  flight_nav_pub_ = nh_.advertise<aerial_robot_msgs::FlightNav>("uav/nav", 1);
  se3_roll_pitch_nav_pub_ = nh_.advertise<spinal::DesireCoord>("desire_coordinate", 1);
  desired_path_pub_ = nh_.advertise<moveit_msgs::DisplayRobotState>("desired_robot_state", 1);
  debug_pub_ = nh_.advertise<moveit_msgs::DisplayRobotState>("debug_robot_state", 1);


  /* robot model */
  nhp_.param("motion_type", motion_type_, 0);
  nhp_.param("robot_type", robot_type_, std::string("hydrus"));

  if (robot_type_ == "hydrus") 
    robot_model_ptr_ = boost::shared_ptr<HydrusRobotModel>(new HydrusRobotModel(true));
  else if (robot_type_ == "hydrus_xi") 
    robot_model_ptr_ = boost::shared_ptr<HydrusTiltedRobotModel>(new HydrusTiltedRobotModel(true));
  else if (robot_type_ == "dragon") 
    robot_model_ptr_ = boost::shared_ptr<HydrusRobotModel>(new Dragon::HydrusLikeRobotModel(true));
  // if (motion_type_ == motion_type::SE2) //SE2
  //   robot_model_ptr_ = boost::shared_ptr<HydrusRobotModel>(new HydrusRobotModel(true));
  // else //SE3
  //   robot_model_ptr_ = boost::shared_ptr<HydrusRobotModel>(new Dragon::HydrusLikeRobotModel(true));
  /* set the joint angle limit */
  for(auto itr : robot_model_ptr_->getLinkJointNames())
    {
      auto joint_ptr = robot_model_ptr_->getUrdfModel().getJoint(itr);
      assert(joint_ptr != nullptr);

      angle_min_vec_.push_back(joint_ptr->limits->lower);
      angle_max_vec_.push_back(joint_ptr->limits->upper);
    }

  /* discrete path search */
  std::string discrete_path_search_method_name;
  nhp_.param("discrete_path_search_method_name", discrete_path_search_method_name, std::string("none"));
  path_planner_ = plugin_loader_.createInstance(discrete_path_search_method_name);
  path_planner_->initialize(nh_, nhp_, robot_model_ptr_);

  /* navigation timer */
  navigate_timer_ = nh_.createTimer(ros::Duration(1.0 / controller_freq_), &SqueezeNavigation::process, this);
  reset();
}

void SqueezeNavigation::rosParamInit()
{
  /* ros param */
  ros::NodeHandle nh_global("~");
  nhp_.param("headless", headless_, true);
  nhp_.param("debug_verbose", debug_verbose_, false);

  // discrete path
  nhp_.param("discrete_path_debug_flag", discrete_path_debug_flag_, false);

  // navigation
  nhp_.param("control_frequency", controller_freq_, 40.0);
  nhp_.param("joint_thresh", joint_thresh_, 0.05); // rad
  nhp_.param("pos_thresh", pos_thresh_, 0.1); // m
  nhp_.param("rot_thresh", rot_thresh_, 0.05); // rot

  double temp;
  nhp_.param("desired_state_color_r", temp, 0.0);
  desired_state_color_.r = temp;
  nhp_.param("desired_state_color_g", temp, 0.0);
  desired_state_color_.g = temp;
  nhp_.param("desired_state_color_b", temp, 1.0);
  desired_state_color_.b = temp;
  nhp_.param("desired_state_color_a", temp, 0.6);
  desired_state_color_.a = temp;

  nhp_.param("return_delay", return_delay_, 15.0);
}


void SqueezeNavigation::reset()
{
  plan_flag_ = false;
  move_flag_ = false;
  return_flag_ = false;
  state_index_ = 0;
  max_joint_vel_ = 0;

  path_planner_->reset();
}

void SqueezeNavigation::startNavigate()
{
 if(path_planner_->getDiscretePath().size() == 0)
    {
      ROS_ERROR("have not get planned path, cannot start");
      return;
    }

  move_flag_ = true;
  move_start_time_ = -10; // for check the convergence to the init state; -10 is for incremental count

  /* publish the init state */
  goToInitState(path_planner_->getDiscreteState(0));
}

void SqueezeNavigation::goToInitState(const MultilinkState& init_state)
{
  /* publish uav nav */
  aerial_robot_msgs::FlightNav nav_msg;
  nav_msg.header.frame_id = std::string("/world");
  nav_msg.header.stamp = ros::Time::now();

  /* x & y */
  nav_msg.control_frame = nav_msg.WORLD_FRAME;
  nav_msg.target = nav_msg.COG;
  nav_msg.pos_xy_nav_mode = nav_msg.POS_MODE;
  nav_msg.target_pos_x = init_state.getCogPoseConst().position.x;
  nav_msg.target_pos_y = init_state.getCogPoseConst().position.y;

  /* z */
  if(motion_type_ == motion_type::SE2)
    {
      nav_msg.pos_z_nav_mode = nav_msg.NO_NAVIGATION;
    }
  else if(motion_type_ == motion_type::SE3)
    {
      nav_msg.pos_z_nav_mode = nav_msg.POS_MODE;
      nav_msg.target_pos_z = init_state.getCogPoseConst().position.z;
    }

  /* orientation */
  tf::Matrix3x3 att(init_state.getBaselinkDesiredAttConst());
  double r, p, y; att.getRPY(r, p, y);

  /* yaw */
  nav_msg.yaw_nav_mode = nav_msg.POS_MODE;
  nav_msg.target_yaw = y;
  flight_nav_pub_.publish(nav_msg);

  /* roll & pitch */
  if(motion_type_ == motion_type::SE2)
    {
      spinal::DesireCoord att_msg;
      att_msg.roll = r;
      att_msg.pitch = p;
      se3_roll_pitch_nav_pub_.publish(att_msg);
    }

  /* joint states */
  sensor_msgs::JointState joints_msg;
  joints_msg.header = nav_msg.header;
  for(auto itr : robot_model_ptr_->getLinkJointIndices())
    joints_msg.position.push_back(init_state.getJointStateConst()(itr));
  joints_ctrl_pub_.publish(joints_msg);
}


void SqueezeNavigation::planStartCallback(const std_msgs::Empty msg)
{
  plan_flag_ = true;
}

void SqueezeNavigation::moveStartCallback(const std_msgs::Empty msg)
{
 if(path_planner_->getDiscretePath().size() == 0)
    {
      ROS_WARN("have no valid planned path");
      return;
    }
  ROS_INFO("[SqueezeNavigation] Receive move start topic.");

  /* start move */
  startNavigate();
}


void SqueezeNavigation::process(const ros::TimerEvent& event)
{
  if(!headless_) visualize();

  /* do planning */
  pathSearch();

  /* navigation */
  pathNavigate();
}

void SqueezeNavigation::pathSearch()
{
  if(plan_flag_)
    {
      plan_flag_ = false;
      move_flag_ = false; // disable the real motion

      bool start_squeeze_path_from_real_state;
      nhp_.param("start_squeeze_path_from_real_state", start_squeeze_path_from_real_state, false);

      /*-- 1. get discrete path for squeezing motion --*/
      bool load_squeeze_path_flag;
      nhp_.param("load_squeeze_path_flag", load_squeeze_path_flag, false);
      if(load_squeeze_path_flag)
        {
          path_planner_->loadPath();
          if(path_planner_->getDiscretePath().size() == 0)
            {
              ROS_ERROR("squeeze navigation: can not get valid discrete path from sampling based method");
              reset();
              return;
            }
        }
      else
        {
          /*-- do online planning --*/
          if(discrete_path_debug_flag_) start_squeeze_path_from_real_state = false;

          if(start_squeeze_path_from_real_state)
            {
              if(!real_machine_connect_)
                {
                  ROS_ERROR("have not received the real robot state, cannot start plan from the real state.");
                  reset();
                  return;
                }

              /* get from the real state */
              geometry_msgs::Pose root_pose;
              MultilinkState::convertBaselinkPose2RootPose(robot_model_ptr_, robot_baselink_odom_.pose.pose, joint_state_, root_pose);
              path_planner_->setInitState(MultilinkState(robot_model_ptr_, root_pose, joint_state_));

              /* assume the opening is right above the robot */
              bool overhead_opening;
              nhp_.param("overhead_opening", overhead_opening, false);
              if (overhead_opening)
                {
                  // get the pre-define robot z and the opening z
                  double robot_z;
                  nhp_.param("start_state_z", robot_z, 0.0);
                  double opening_z;
                  nhp_.param("openning_pos_z", opening_z, 0.0);
                  tf::Transform frame; frame.setIdentity();

                  frame.setOrigin(tf::Vector3(root_pose.position.x,
                                              root_pose.position.y,
                                              root_pose.position.z + opening_z - robot_z));
                  frame.setRotation(tf::createQuaternionFromYaw(tf::getYaw(root_pose.orientation)));
                  path_planner_->setOpenningCenterFrame(frame);
                }

            }

          /* do path search */
          if(!path_planner_->plan())
            {
              ROS_ERROR("squeeze navigation: cannot get valid planning result");
              reset();
              return;
            }
        }


      if (start_squeeze_path_from_real_state) startNavigate(); // autonatically start
    }
}

void SqueezeNavigation::visualize()
{
  path_planner_->visualizeFunc(); // visualize collision enviroment 
}

void SqueezeNavigation::returnCallback(const std_msgs::Empty msg)
{
  if(move_flag_)
    {
      ROS_WARN("still performing desired path motion");
      return;
    }
  return_flag_ = true;

  start_return_time_ = ros::Time::now().toSec();

  /* send init joint state */
  sensor_msgs::JointState joints_msg;
  joints_msg.header.stamp = ros::Time::now();
  for(int i = 0 ; i < robot_model_ptr_->getLinkJointNames().size(); i++)
    {
      joints_msg.name.push_back(robot_model_ptr_->getLinkJointNames().at(i));
      joints_msg.position.push_back(path_planner_->getDiscreteState(0).getJointStateConst()(robot_model_ptr_->getLinkJointIndices().at(i)));
    }

  joints_ctrl_pub_.publish(joints_msg);
}


void SqueezeNavigation::pathNavigate()
{
  pathNavigate(path_planner_->getDiscretePath(), path_planner_->getContinuousPath());
}

void SqueezeNavigation::pathNavigate(const std::vector<MultilinkState>& discrete_path, boost::shared_ptr<ContinuousPathGenerator> continuous_path)
{
  if(!move_flag_)
    {
      /* final process: return */
      if (return_flag_)
        {
          double t = ros::Time::now().toSec() - start_return_time_;
          if(t < return_delay_ * 2 / 3)
            {
              spinal::DesireCoord att_msg;
              double rate = 1 - t / (return_delay_ * 2 / 3);
              att_msg.roll = rate * continuous_path->getPositionVector(continuous_path->getPathDuration() + 1.0 / controller_freq_)[3];
              att_msg.pitch = rate * continuous_path->getPositionVector(continuous_path->getPathDuration() + 1.0 / controller_freq_)[4];
              se3_roll_pitch_nav_pub_.publish(att_msg);
              ROS_INFO_THROTTLE(1.0, "set robot level and the init joint state ");
            }

          if(t > return_delay_)
            {
              /* set SE2 goal (return) position */
              if (nhp_.hasParam("final_pos_x") && nhp_.hasParam("final_pos_y") && nhp_.hasParam("final_yaw"))
                {
                  double final_pos_x, final_pos_y, final_yaw;
                  nhp_.getParam("final_pos_x", final_pos_x);
                  nhp_.getParam("final_pos_y", final_pos_y);
                  nhp_.getParam("final_yaw", final_yaw);

                  aerial_robot_msgs::FlightNav nav_msg;
                  nav_msg.header.frame_id = std::string("/world");
                  nav_msg.header.stamp = ros::Time::now();
                  /* x & y */
                  nav_msg.control_frame = nav_msg.WORLD_FRAME;
                  nav_msg.target = nav_msg.COG;
                  nav_msg.pos_xy_nav_mode = nav_msg.POS_MODE;
                  nav_msg.target_pos_x = final_pos_x;
                  nav_msg.target_pos_y = final_pos_y;

                  /* yaw */
                  nav_msg.yaw_nav_mode = nav_msg.POS_MODE;
                  nav_msg.target_yaw = final_yaw;
                  flight_nav_pub_.publish(nav_msg);
                }

              reset();
            }
        }
      return;
    }

  int joint_num = robot_model_ptr_->getLinkJointIndices().size();
  moveit_msgs::DisplayRobotState display_robot_state;

  /* test (debug) the discrete path */
  if(discrete_path_debug_flag_)
    {
      if(state_index_ == discrete_path.size()) return; //debug

      display_robot_state.state.joint_state.header.seq = state_index_;
      display_robot_state.state.joint_state.header.stamp = ros::Time::now();
      display_robot_state.state = discrete_path.at(state_index_).getRootJointStateConst<moveit_msgs::RobotState>();

      desired_path_pub_.publish(display_robot_state);

      /* debug */
      {
        moveit_msgs::DisplayRobotState debug_robot_state;

        if(state_index_ +1 < discrete_path.size())
          {
            tf::Vector3 p1, p2;
            tf::pointMsgToTF(discrete_path.at(state_index_).getCogPoseConst().position, p1);
            tf::pointMsgToTF(discrete_path.at(state_index_ + 1).getCogPoseConst().position, p2);
            debug_robot_state.state.joint_state.position.push_back((p1-p2).length());

            debug_robot_state.state.joint_state.position.push_back((discrete_path.at(state_index_ + 1).getBaselinkDesiredAttConst() * discrete_path.at(state_index_).getBaselinkDesiredAttConst().inverse()).getAngle());

            double joint_angle_sum = 0;
            for(auto itr : robot_model_ptr_->getLinkJointIndices())
              {
                double delta_angle = discrete_path.at(state_index_ + 1).getJointStateConst()(itr) - discrete_path.at(state_index_).getJointStateConst()(itr);
                joint_angle_sum += (delta_angle * delta_angle);
              }
            debug_robot_state.state.joint_state.position.push_back(sqrt(joint_angle_sum));

            debug_pub_.publish(debug_robot_state);
          }
      }

      if(++state_index_ == discrete_path.size()) state_index_ = 0;

      return;
    }

  /* check the convergence to the init state */
  if(move_start_time_ < 0)
    {
      if(!real_machine_connect_)
        {
          ROS_WARN("skip the check of init state, and do not send navigation command to robot.");
          move_start_time_ = ros::Time::now().toSec();
        }
      else
        {
          bool convergence = true;

          /* check pose, directly from control terms */
          if (fabs(control_terms_.x.err_p) > pos_thresh_ ||
              fabs(control_terms_.y.err_p) > pos_thresh_ ||
              fabs(control_terms_.z.err_p) > pos_thresh_ ||
              fabs(control_terms_.roll.err_p) > rot_thresh_ ||
              fabs(control_terms_.pitch.err_p) > rot_thresh_ ||
              fabs(control_terms_.yaw.err_p) > rot_thresh_)
            {
              convergence = false;
            }

          ROS_DEBUG("pose errs: [%f, %f, %f], [%f, %f, %f]",
                    control_terms_.x.err_p,
                    control_terms_.y.err_p,
                    control_terms_.z.err_p,
                    control_terms_.roll.err_p,
                    control_terms_.pitch.err_p,
                    control_terms_.yaw.err_p);


          /* check joint */
          std::stringstream ss;
          MultilinkState init_state = discrete_path.at(0);
          for(auto itr : robot_model_ptr_->getLinkJointIndices())
            {
              double delta_angle = init_state.getJointStateConst()(itr) - joint_state_(itr);
              if(fabs(delta_angle) > joint_thresh_) convergence = false;
              ss << delta_angle << ", ";
            }
          ROS_DEBUG_STREAM("joint errs: " << ss.str());

          if(convergence) move_start_time_ ++; // successive convergence, move_start_time from a negative integar (e.g., -10)

          if(move_start_time_ == 0)
            {
              move_start_time_ = ros::Time::now().toSec();
              ROS_INFO("robot convergens to the initial state, start path following");
            }
        }
      return;
    }


  double cur_time = ros::Time::now().toSec() - move_start_time_;
  std::vector<double> des_pos = continuous_path->getPositionVector(cur_time + 1.0 / controller_freq_);
  std::vector<double> des_vel = continuous_path->getVelocityVector(cur_time);

  {
    // debug
    moveit_msgs::DisplayRobotState debug_robot_state;
    debug_robot_state.state.joint_state.position.push_back(tf::Vector3(des_vel[0], des_vel[1], des_vel[2]).length());
    debug_robot_state.state.joint_state.position.push_back(tf::Vector3(des_vel[3], des_vel[4], 0).length()); //no need to add yaw velocity

    double joint_angle_sum = 0;
    for(int i = 0; i < joint_num; i++)
      {
        joint_angle_sum += (des_vel[6+i] * des_vel[6+i]);
        if(fabs(des_vel[6+i]) > max_joint_vel_)
          {
            max_joint_vel_ = fabs(des_vel[6+i]);
            ROS_WARN("max joint vel: %f", max_joint_vel_);
          }
      }
    debug_robot_state.state.joint_state.position.push_back(sqrt(joint_angle_sum));

    debug_pub_.publish(debug_robot_state);
  }

  /* send general flight navigation command */
  if(real_machine_connect_)
    {
      aerial_robot_msgs::FlightNav nav_msg;
      nav_msg.header.frame_id = std::string("/world");
      nav_msg.header.stamp = ros::Time::now();
      /* x & y */
      nav_msg.control_frame = nav_msg.WORLD_FRAME;
      nav_msg.target = nav_msg.COG;
      nav_msg.pos_xy_nav_mode = nav_msg.POS_VEL_MODE;
      nav_msg.target_pos_x = des_pos[0];
      nav_msg.target_pos_y = des_pos[1];
      nav_msg.target_vel_x = des_vel[0];
      nav_msg.target_vel_y = des_vel[1];
      /* z axis */
      if(motion_type_ == motion_type::SE2)
        {
          nav_msg.pos_z_nav_mode = nav_msg.NO_NAVIGATION;
        }
      else if(motion_type_ == motion_type::SE3)
        {
          nav_msg.pos_z_nav_mode = nav_msg.POS_VEL_MODE;
          nav_msg.target_pos_z = des_pos[2];
          nav_msg.target_vel_z = des_vel[2];
        }
      /* yaw */
      nav_msg.yaw_nav_mode = nav_msg.POS_VEL_MODE;
      nav_msg.target_yaw = des_pos[5];
      nav_msg.target_omega_z = des_vel[5];
      flight_nav_pub_.publish(nav_msg);

      /* roll & pitch */
      if(motion_type_ == motion_type::SE3)
        {
          spinal::DesireCoord att_msg;
          att_msg.roll = des_pos[3];
          att_msg.pitch = des_pos[4];
          se3_roll_pitch_nav_pub_.publish(att_msg);
        }

      /* joint states */
      sensor_msgs::JointState joints_msg;
      joints_msg.header = nav_msg.header;
      for (int i = 0; i < joint_num; ++i)
        {
          joints_msg.name.push_back(robot_model_ptr_->getLinkJointNames().at(i));
          joints_msg.position.push_back(boost::algorithm::clamp(des_pos[6 + i], angle_min_vec_.at(i), angle_max_vec_.at(i)));
        }

      joints_ctrl_pub_.publish(joints_msg);
    }


  /* publish the target pose and form for visualize */
  /* cog pose */
  geometry_msgs::Pose cog_pose;
  cog_pose.position.x = des_pos.at(0);
  cog_pose.position.y = des_pos.at(1);
  cog_pose.position.z = des_pos.at(2);
  cog_pose.orientation = tf::createQuaternionMsgFromYaw(des_pos.at(5)); /* special: only get yaw angle */

  /* joint state:  */
  auto joint_vector = discrete_path.at(0).getJointStateConst();
  for(int i = 0; i < joint_num; i++)
    joint_vector(robot_model_ptr_->getLinkJointIndices().at(i)) = des_pos.at(6 + i);

  MultilinkState state_tmp(robot_model_ptr_,
                           tf::createQuaternionFromRPY(des_pos[3], des_pos[4], 0), cog_pose,
                           joint_vector);
  display_robot_state.state = state_tmp.getRootJointStateConst<moveit_msgs::RobotState>();
  display_robot_state.state.joint_state.header.stamp = ros::Time::now();

  /* set color */
  for(auto itr: robot_model_ptr_->getUrdfModel().links_)
    {
      moveit_msgs::ObjectColor object_color;
      object_color.id = itr.first;
      object_color.color = desired_state_color_;
      display_robot_state.highlight_links.push_back(object_color);
    }

  desired_path_pub_.publish(display_robot_state);

  /* check the end of navigation */
  if(cur_time > continuous_path->getPathDuration() + 1.0) // margin: 1.0 [sec]
    {
      ROS_INFO("[SqueezeNavigation] Finish Navigation");
      move_flag_ = false;
    }
}

void SqueezeNavigation::robotOdomCallback(const nav_msgs::OdometryConstPtr& msg)
{
  real_machine_connect_ = true;
  robot_baselink_odom_ = *msg;
}

void SqueezeNavigation::controlTermsCallback(const aerial_robot_msgs::PoseControlPidConstPtr& control_msg)
{
  real_machine_connect_ = true;
  control_terms_ = *control_msg;
}

void SqueezeNavigation::robotJointStatesCallback(const sensor_msgs::JointStateConstPtr& joints_msg)
{
  real_machine_connect_ = true;
  joint_state_ = robot_model_ptr_->jointMsgToKdl(*joints_msg);
}


void SqueezeNavigation::joyStickControl(const sensor_msgs::JoyConstPtr & joy_msg)
{
  using namespace aerial_robot_navigation;
  sensor_msgs::Joy joy_cmd;
  if(joy_msg->axes.size() == BaseNavigator::PS3_AXES && joy_msg->buttons.size() == BaseNavigator::PS3_BUTTONS)
    {
      joy_cmd = (*joy_msg);
    }
  else if(joy_msg->axes.size() == BaseNavigator::PS4_AXES && joy_msg->buttons.size() == BaseNavigator::PS4_BUTTONS)
    {
      joy_cmd = BaseNavigator::ps4joyToPs3joyConvert(*joy_msg);
    }
  else
    {
      ROS_WARN("the joystick type is not supported (buttons: %d, axes: %d)", (int)joy_msg->buttons.size(), (int)joy_msg->axes.size());
      return;
    }

  /* force landing */
  if(joy_cmd.buttons[BaseNavigator::PS3_BUTTON_SELECT] == 1 && move_flag_)
    {
      ROS_INFO("[SqueezeNavigation] Receive force landing command, stop navigation.");
      move_flag_ = false;
      return;
    }

  /* landing */
  if(joy_cmd.buttons[BaseNavigator::PS3_BUTTON_CROSS_RIGHT] == 1 || joy_cmd.buttons[BaseNavigator::PS3_BUTTON_ACTION_SQUARE] == 1 && move_flag_)
    {
      ROS_INFO("[SqueezeNavigation] Receive normal landing command, stop navigation.");
      move_flag_ = false;
      return;
    }
}
