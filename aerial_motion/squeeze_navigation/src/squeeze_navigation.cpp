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

namespace
{
  pluginlib::ClassLoader<squeeze_motion_planner::Base> plugin_loader("squeeze_navigation", "squeeze_motion_planner::Base");

  nav_msgs::Odometry robot_baselink_odom_;
  bool real_odom_flag_ = false;
  int state_index_ = 0;

  std_msgs::ColorRGBA desired_state_color_;

  ros::Publisher debug_pub_;

  double start_return_time_;

  double max_joint_vel = 0; //debug
}

SqueezeNavigation::SqueezeNavigation(ros::NodeHandle nh, ros::NodeHandle nhp):
  nh_(nh), nhp_(nhp), move_start_flag_(false), return_flag_(false), discrete_path_(0)
{
  rosParamInit();

  /* ros pub/sub, srv */
  plan_start_flag_sub_ = nh_.subscribe("/plan_start", 1, &SqueezeNavigation::planStartCallback, this);
  move_start_flag_sub_ = nh_.subscribe("/move_start", 1, &SqueezeNavigation::moveStartCallback, this);
  return_flag_sub_ = nh_.subscribe("/return", 1, &SqueezeNavigation::returnCallback, this);
  adjust_initial_state_sub_ = nh_.subscribe("/adjust_robot_initial_state", 1, &SqueezeNavigation::adjustInitalStateCallback, this);
  flight_config_sub_ = nh_.subscribe("/flight_config_cmd", 1,  &SqueezeNavigation::flightConfigCallback, this);

  std::string topic_name;
  nhp_.param("joint_control_topic_name", topic_name, std::string("joints_ctrl"));
  joints_ctrl_pub_ = nh_.advertise<sensor_msgs::JointState>(topic_name, 1);
  flight_nav_pub_ = nh_.advertise<aerial_robot_msgs::FlightNav>("/uav/nav", 1);
  se3_roll_pitch_nav_pub_ = nh_.advertise<spinal::DesireCoord>("/desire_coordinate", 1);
  desired_path_pub_ = nh_.advertise<moveit_msgs::DisplayRobotState>("/desired_robot_state", 1);
  debug_pub_ = nh_.advertise<moveit_msgs::DisplayRobotState>("/debug_robot_state", 1);

  if(!headless_)
    {
      std::string topic_name;
      robot_baselink_odom_sub_ = nh_.subscribe("/uav/baselink/odom", 1, &SqueezeNavigation::robotOdomCallback, this);
      nhp_.param("joint_state_topic_name", topic_name, std::string("joint_states"));
      robot_joint_states_sub_ = nh_.subscribe(topic_name, 1, &SqueezeNavigation::robotJointStatesCallback, this);
    }

  /* robot model */
  //////// TODO: temporary //////////////
  nhp_.param("motion_type", motion_type_, 0);
  if (motion_type_ == motion_type::SE2) //SE2
    robot_model_ptr_ = boost::shared_ptr<HydrusRobotModel>(new HydrusRobotModel(true));
  else //SE3
    robot_model_ptr_ = boost::shared_ptr<HydrusRobotModel>(new DragonRobotModel(true));
  /////////////////////////////////

  /* discrete path search */
  std::string discrete_path_search_method_name;
  nhp_.param("discrete_path_search_method_name", discrete_path_search_method_name, std::string("none"));
  discrete_path_planner_ = plugin_loader.createInstance(discrete_path_search_method_name);
  discrete_path_planner_->initialize(nh_, nhp_, robot_model_ptr_);
  /* continous path generator */
  bspline_ptr_ = boost::shared_ptr<BsplineRos>(new BsplineRos(nh_, nhp_));

  /* navigation timer */
  navigate_timer_ = nh_.createTimer(ros::Duration(1.0 / controller_freq_), &SqueezeNavigation::navigate, this);
}

void SqueezeNavigation::rosParamInit()
{
  /* ros param */
  nhp_.param("headless", headless_, true);
  nhp_.param("debug_verbose", debug_verbose_, false);

  // discrete path
  nhp_.param("load_path_flag", load_path_flag_, false);
  nhp_.param("discrete_path_debug_flag", discrete_path_debug_flag_, false);

  // continuous path
  nhp_.param("trajectory_period", trajectory_period_, 100.0);
  nhp_.param("bspline_degree", bspline_degree_, 5);

  // navigation
  nhp_.param("control_frequency", controller_freq_, 40.0);
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

void SqueezeNavigation::planStartCallback(const std_msgs::Empty msg)
{
  ROS_INFO("[SqueezeNavigation] Receive plan start topic.");
  state_index_ = 0; //reset

  /* discrete path */
  if(load_path_flag_)
    {
      discrete_path_planner_->loadPath();
      if(discrete_path_planner_->getPathConst().size() == 0)
        {
          ROS_WARN("squeeze navigation: can not get valid discrete path from sampling based method");
          return;
        }
    }
  else
    {
      if(!discrete_path_planner_->plan(debug_verbose_)) return;
    }

  discrete_path_ = discrete_path_planner_->getPathConst();

  /* low path filter */
  bool discrete_path_filter_flag;
  nhp_.param("discrete_path_filter_flag", discrete_path_filter_flag, false);
  if(discrete_path_filter_flag)
    {
      discrete_path_.clear();

      int joint_num = robot_model_ptr_->getLinkJointIndex().size();
      double filter_rate;
      nhp_.param("filter_rate", filter_rate, 0.1);

      states_lpf1_ = FirFilter(filter_rate, 3 + joint_num); //root position + joint_num
      states_lpf2_ = FirFilterQuaternion(filter_rate); //root orientation


      /* init state */
      auto init_state = discrete_path_planner_->getStateConst(0);
      Eigen::VectorXd init_state_vec = Eigen::VectorXd::Zero(3 + joint_num);
      init_state_vec.head(3) = Eigen::Vector3d(init_state.getRootPoseConst().position.x,
                                               init_state.getRootPoseConst().position.y,
                                               init_state.getRootPoseConst().position.z);
      for(int j = 0; j < joint_num; j++)
        init_state_vec(3 + j) = init_state.getActuatorStateConst()(robot_model_ptr_->getLinkJointIndex().at(j));
      states_lpf1_.setInitValues(init_state_vec); //init filter with the first value

      tf::Quaternion init_q;
      tf::quaternionMsgToTF(init_state.getRootPoseConst().orientation, init_q);
      states_lpf2_.setInitValues(init_q); //init filter with the first value

      /* do filtering */
      for(int index = 0; index < discrete_path_planner_->getPathConst().size() + 1 / states_lpf1_.getFilterFactor(); index++)
        {
          auto state_itr = discrete_path_planner_->getPathConst().back();
          if(index < discrete_path_planner_->getPathConst().size())
            state_itr = discrete_path_planner_->getPathConst().at(index);

          Eigen::VectorXd state_vec = Eigen::VectorXd::Zero(3 + joint_num);
          state_vec.head(3) = Eigen::Vector3d(state_itr.getRootPoseConst().position.x,
                                              state_itr.getRootPoseConst().position.y,
                                              state_itr.getRootPoseConst().position.z);

          for(int j = 0; j < joint_num; j++)
            state_vec(3 + j) = state_itr.getActuatorStateConst()(robot_model_ptr_->getLinkJointIndex().at(j));
          Eigen::VectorXd filtered_state =  states_lpf1_.filterFunction(state_vec);

          /* joint */
          auto filtered_actuator_vector = state_itr.getActuatorStateConst();
          for(int j = 0; j < joint_num; j++)
            filtered_actuator_vector(robot_model_ptr_->getLinkJointIndex().at(j)) = filtered_state(3 + j);

          /* root position */
          geometry_msgs::Pose filtered_root_pose;
          filtered_root_pose.position.x = filtered_state(0);
          filtered_root_pose.position.y = filtered_state(1);
          filtered_root_pose.position.z = filtered_state(2);

          /* root orientation */
          tf::Quaternion raw_q;
          tf::quaternionMsgToTF(state_itr.getRootPoseConst().orientation, raw_q);
          tf::quaternionTFToMsg(states_lpf2_.filterFunction(raw_q), filtered_root_pose.orientation);

          discrete_path_.push_back(MultilinkState(robot_model_ptr_, filtered_root_pose, filtered_actuator_vector));
        }
    }

  bool resampling_flag;
  nhp_.param("resampling_flag", resampling_flag, false);
  /* resampling */
  if(resampling_flag)
    {
      std::vector<MultilinkState> unsampling_path = discrete_path_;
      discrete_path_.clear();

      double resampling_angular_rate;
      double resampling_seg_diff_thresh;
      nhp_.param("resampling_angular_rate", resampling_angular_rate, 1.0);
      nhp_.param("resampling_seg_diff_thresh", resampling_seg_diff_thresh, 0.2);
      double path_length = 0;

      for(int i = 0; i < unsampling_path.size() - 1; i++)
        {
          /* translation motion */
          tf::Vector3 p1, p2;
          tf::pointMsgToTF(unsampling_path.at(i).getCogPoseConst().position, p1);
          tf::pointMsgToTF(unsampling_path.at(i + 1).getCogPoseConst().position, p2);
          path_length += (p1-p2).length();

          /* rotational motion */
          double angle_diff = fabs(unsampling_path.at(i).getBaselinkDesiredAttConst().angleShortestPath(unsampling_path.at(i+1).getBaselinkDesiredAttConst()));
          path_length += resampling_angular_rate * angle_diff;
          //ROS_WARN("%d: angle_diff: %f", i, angle_diff);
        }

      double ave_delta_trans = path_length / unsampling_path.size();
      ROS_WARN("path length is %f, average delta trans is %f", path_length, ave_delta_trans);

      double delta_trans = 0;
      bool not_enough = false;
      discrete_path_.push_back(unsampling_path.front());

      for(int i = 1; i < unsampling_path.size();)
        {
          tf::Vector3 prev_p;
          tf::pointMsgToTF(unsampling_path.at(i - 1).getCogPoseConst().position, prev_p);
          tf::Quaternion prev_q = unsampling_path.at(i - 1).getBaselinkDesiredAttConst();
          tf::Vector3 new_p;
          tf::pointMsgToTF(unsampling_path.at(i).getCogPoseConst().position, new_p);
          tf::Quaternion new_q = unsampling_path.at(i).getBaselinkDesiredAttConst();
          tf::Vector3 ref_p;
          tf::pointMsgToTF(discrete_path_.back().getCogPoseConst().position, ref_p);
          tf::Quaternion ref_q = discrete_path_.back().getBaselinkDesiredAttConst();

          if(delta_trans == 0) delta_trans = (ref_p - new_p).length() + resampling_angular_rate * fabs(new_q.angleShortestPath(ref_q));
          if(not_enough)
            {
              delta_trans += ((prev_p - new_p).length() + resampling_angular_rate * fabs(new_q.angleShortestPath(prev_q)));
              not_enough = false;
            }

          if(debug_verbose_)
            {
              double r,p,y;
              tf::Matrix3x3(discrete_path_.back().getBaselinkDesiredAttConst()).getRPY(r,p,y);
              ROS_INFO("ref pose [%f, %f, %f] [%f, %f, %f]", ref_p.x(), ref_p.y() ,ref_p.z(), r, p, y);
              std::cout << "new point vs resample ref point: [" << i << ", " << discrete_path_.size() << "], inrcement vs delta_trans vs ave: [" << (ref_p - new_p).length() + resampling_angular_rate * fabs(new_q.angleShortestPath(ref_q)) << ", " << delta_trans << ", " << ave_delta_trans << "]. ";
            }

          if(fabs(delta_trans - ave_delta_trans) < resampling_seg_diff_thresh * ave_delta_trans)
            {
              if(debug_verbose_)
                {
                  std::cout << "add unsampling state directly since convergence. rate: " << fabs(delta_trans - ave_delta_trans) / ave_delta_trans << ". " << std::endl;
                  ROS_WARN("delta_trans: %f", delta_trans);
                }
              discrete_path_.push_back(unsampling_path.at(i));
              delta_trans = 0;
              i++;
            }
          else if (delta_trans >  ave_delta_trans)
            {
              // interpolation
              double interpolate_rate = 1 - (delta_trans - ave_delta_trans) / ((prev_p - new_p).length() + resampling_angular_rate * fabs(prev_q.angleShortestPath(new_q)));
              if(debug_verbose_)
                std::cout << " interpolation since too big delta trans. interpolate rate: " << interpolate_rate << ". "  << std::endl;


              /* joint */
              auto actuator_vector = unsampling_path.at(i - 1).getActuatorStateConst();
              for(auto index: robot_model_ptr_->getLinkJointIndex())
                actuator_vector(index) = unsampling_path.at(i - 1).getActuatorStateConst()(index) * (1 - interpolate_rate) + unsampling_path.at(i).getActuatorStateConst()(index) * interpolate_rate;


              /* cog pose */
              geometry_msgs::Pose cog_pose;
              tf::pointTFToMsg(prev_p * (1 - interpolate_rate) + new_p * interpolate_rate, cog_pose.position);
              cog_pose.orientation.w = 1;

              /* baselink attitude and  root pose */
              tf::Quaternion desired_baselink_q = unsampling_path.at(i - 1).getBaselinkDesiredAttConst().slerp(unsampling_path.at(i).getBaselinkDesiredAttConst(), interpolate_rate);

              discrete_path_.push_back(MultilinkState(robot_model_ptr_,
                                                       desired_baselink_q, cog_pose,
                                                       actuator_vector));
              tf::Vector3 new_ref_p;
              tf::pointMsgToTF(discrete_path_.back().getCogPoseConst().position, new_ref_p);
              tf::Quaternion new_ref_q = discrete_path_.back().getBaselinkDesiredAttConst();
              delta_trans = (new_ref_p - new_p).length() + resampling_angular_rate * fabs(new_q.angleShortestPath(new_ref_q));

              if(debug_verbose_) ROS_WARN("delta_trans: %f", delta_trans);
            }
          else
            {
              not_enough = true;
              i++;
              if(debug_verbose_) std::cout << "  not enough delta trans. "  << std::endl;
            }
        }

    }

  if (discrete_path_debug_flag_)
    {
      move_start_flag_ = true;
      return;
    }

  /* continuous path */
  continuousPath(discrete_path_);

  /* set the joint angle limit */
  for(auto itr : robot_model_ptr_->getLinkJointNames())
    {
      auto joint_ptr = robot_model_ptr_->getUrdfModel().getJoint(itr);
      assert(joint_ptr != nullptr);

      angle_min_vec_.push_back(joint_ptr->limits->lower);
      angle_max_vec_.push_back(joint_ptr->limits->upper);
    }
}


void SqueezeNavigation::adjustInitalStateCallback(const std_msgs::Empty msg)
{
  if(discrete_path_.size() == 0)
    {
      ROS_ERROR("have not get planned path");
      return;
    }

  auto initial_state = discrete_path_.at(0);
  auto cog_pos = initial_state.getCogPoseConst().position;
  tf::Matrix3x3 att(initial_state.getBaselinkDesiredAttConst());
  double euler_r, euler_p, euler_y; att.getRPY(euler_r, euler_p, euler_y);

  /* publish uav nav */
  aerial_robot_msgs::FlightNav nav_msg;
  nav_msg.header.frame_id = std::string("/world");
  nav_msg.header.stamp = ros::Time::now();

  /* x & y */
  nav_msg.control_frame = nav_msg.WORLD_FRAME;
  nav_msg.target = nav_msg.COG;
  nav_msg.pos_xy_nav_mode = nav_msg.POS_MODE;
  nav_msg.target_pos_x = cog_pos.x;
  nav_msg.target_pos_y = cog_pos.y;

  /* z */
  if(motion_type_ == motion_type::SE2)
    {
      nav_msg.pos_z_nav_mode = nav_msg.NO_NAVIGATION;
    }
  else if(motion_type_ == motion_type::SE3)
    {
      nav_msg.pos_z_nav_mode = nav_msg.POS_MODE;
      nav_msg.target_pos_z = cog_pos.z;
    }

  /* yaw */
  nav_msg.psi_nav_mode = nav_msg.POS_MODE;
  nav_msg.target_psi = euler_y;
  flight_nav_pub_.publish(nav_msg);

  /* joint states */
  sensor_msgs::JointState joints_msg;
  joints_msg.header = nav_msg.header;
  for(auto itr : robot_model_ptr_->getLinkJointIndex())
    joints_msg.position.push_back(initial_state.getActuatorStateConst()(itr));
  joints_ctrl_pub_.publish(joints_msg);

  /* se3: roll & pitch */
  spinal::DesireCoord att_msg;
  att_msg.roll = euler_r;
  att_msg.pitch = euler_p;
  se3_roll_pitch_nav_pub_.publish(att_msg);
}

void SqueezeNavigation::moveStartCallback(const std_msgs::Empty msg)
{
  if(discrete_path_.size() == 0)
    {
      ROS_ERROR("have not get planned path");
      return;
    }

  move_start_flag_ = true;
  move_start_time_ = ros::Time::now().toSec();
  ROS_INFO("[SqueezeNavigation] Receive move start topic.");
}

void SqueezeNavigation::returnCallback(const std_msgs::Empty msg)
{
  if(move_start_flag_)
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
      joints_msg.position.push_back(discrete_path_planner_->getPathConst().at(0).getActuatorStateConst()(robot_model_ptr_->getLinkJointIndex().at(i)));
    }

  joints_ctrl_pub_.publish(joints_msg);
}

void SqueezeNavigation::flightConfigCallback(const spinal::FlightConfigCmdConstPtr msg)
{
  if(msg->cmd == spinal::FlightConfigCmd::FORCE_LANDING_CMD)
    {
      ROS_INFO("[SqueezeNavigation] Receive force landing command, stop navigation.");
      move_start_flag_ = false;
    }
}

void SqueezeNavigation::navigate(const ros::TimerEvent& event)
{
  /* some special visualize process, such as ENV */
  if(!headless_) discrete_path_planner_->visualizeFunc();

  int joint_num = robot_model_ptr_->getLinkJointIndex().size();

  if (!move_start_flag_)
    {
      if(return_flag_)
        {
          double t = ros::Time::now().toSec() - start_return_time_;
          if(t < return_delay_ * 2 / 3)
            {
              spinal::DesireCoord att_msg;
              double rate = 1 - t / (return_delay_ * 2 / 3);
              att_msg.roll = rate * bspline_ptr_->evaluate(trajectory_period_ + 1.0 / controller_freq_)[3];
              att_msg.pitch = rate * bspline_ptr_->evaluate(trajectory_period_ + 1.0 / controller_freq_)[4];
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
                  nav_msg.psi_nav_mode = nav_msg.POS_MODE;
                  nav_msg.target_psi = final_yaw;
                  flight_nav_pub_.publish(nav_msg);
                }

              return_flag_ = false;
            }
        }

      return;
    }

  moveit_msgs::DisplayRobotState display_robot_state;

  /* test (debug) the discrete path */
  if(discrete_path_debug_flag_)
    {
      if(state_index_ == discrete_path_.size()) return; //debug

      display_robot_state.state.joint_state.header.seq = state_index_;
      display_robot_state.state.joint_state.header.stamp = ros::Time::now();
      display_robot_state.state = discrete_path_.at(state_index_).getRootActuatorStateConst<moveit_msgs::RobotState>();
      desired_path_pub_.publish(display_robot_state);

      /* debug */
      {
        moveit_msgs::DisplayRobotState debug_robot_state;

        if(state_index_ +1 < discrete_path_.size())
          {
            tf::Vector3 p1, p2;
            tf::pointMsgToTF(discrete_path_.at(state_index_).getCogPoseConst().position, p1);
            tf::pointMsgToTF(discrete_path_.at(state_index_ + 1).getCogPoseConst().position, p2);
            debug_robot_state.state.joint_state.position.push_back((p1-p2).length());

            debug_robot_state.state.joint_state.position.push_back((discrete_path_.at(state_index_ + 1).getBaselinkDesiredAttConst()*discrete_path_.at(state_index_).getBaselinkDesiredAttConst().inverse()).getAngle());

            double joint_angle_sum = 0;
            for(auto itr : robot_model_ptr_->getLinkJointIndex())
              {
                double delta_angle = discrete_path_.at(state_index_ + 1).getActuatorStateConst()(itr) - discrete_path_.at(state_index_).getActuatorStateConst()(itr);
                joint_angle_sum += (delta_angle * delta_angle);
              }
            debug_robot_state.state.joint_state.position.push_back(sqrt(joint_angle_sum));

            debug_pub_.publish(debug_robot_state);
          }
      }

      if(++state_index_ == discrete_path_.size()) state_index_ = 0;

      return;
    }

  double cur_time = ros::Time::now().toSec() - move_start_time_;
  std::vector<double> des_pos = bspline_ptr_->evaluate(cur_time + 1.0 / controller_freq_);
  std::vector<double> des_vel = bspline_ptr_->evaluate(cur_time, 1);
  std::vector<double> des_vel_debug = bspline_ptr_->evaluateDerive(cur_time);
  ROS_INFO("vel debug: [%f, %f, %f], [%f, %f, %f]", des_vel.at(0), des_vel.at(1), des_vel.at(2),
           des_vel_debug.at(0), des_vel_debug.at(1), des_vel_debug.at(2));

  {
    // debug
    moveit_msgs::DisplayRobotState debug_robot_state;
    debug_robot_state.state.joint_state.position.push_back(tf::Vector3(des_vel[0], des_vel[1], des_vel[2]).length());
    debug_robot_state.state.joint_state.position.push_back(tf::Vector3(des_vel[3], des_vel[4], 0).length()); //no need to add yaw velocity

    double joint_angle_sum = 0;
    for(int i = 0; i < joint_num; i++)
      {
        joint_angle_sum += (des_vel[6+i] * des_vel[6+i]);
        if(fabs(des_vel[6+i]) > max_joint_vel)
          {
            max_joint_vel = fabs(des_vel[6+i]);
            ROS_WARN("max joint vel: %f", max_joint_vel);
          }
      }
    debug_robot_state.state.joint_state.position.push_back(sqrt(joint_angle_sum));

    debug_pub_.publish(debug_robot_state);
  }

  /* send general flight navigation command (pos + yaw) */
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
  nav_msg.psi_nav_mode = nav_msg.POS_VEL_MODE;
  nav_msg.target_psi = des_pos[5];
  nav_msg.target_vel_psi = des_vel[5];
  flight_nav_pub_.publish(nav_msg);

  /* joint states */
  sensor_msgs::JointState joints_msg;
  joints_msg.header = nav_msg.header;
  for (int i = 0; i < joint_num; ++i)
    {
      joints_msg.name.push_back(robot_model_ptr_->getLinkJointNames().at(i));
      joints_msg.position.push_back(boost::algorithm::clamp(des_pos[6 + i], angle_min_vec_.at(i), angle_max_vec_.at(i)));
    }

  joints_ctrl_pub_.publish(joints_msg);

  if (debug_verbose_)
    {
      std::cout << "joints: ";
      for (int i = 0; i < joint_num; ++i) std::cout << joints_msg.position[i] << ", ";
      std::cout << "\n\n";
    }

  /* se3: roll & pitch */
  spinal::DesireCoord att_msg;
  att_msg.roll = des_pos[3];
  att_msg.pitch = des_pos[4];
  se3_roll_pitch_nav_pub_.publish(att_msg);

  //TODO: set forward roll, pitch
  /* check the end of navigation */
  if(cur_time > trajectory_period_ + 1.0) // margin: 1.0 [sec]
    {
      ROS_INFO("[SqueezeNavigation] Finish Navigation");
      move_start_flag_ = false;
    }

  /* publish the target pose and form for visualize */
  /* cog pose */
  geometry_msgs::Pose cog_pose;
  cog_pose.position.x = des_pos.at(0);
  cog_pose.position.y = des_pos.at(1);
  cog_pose.position.z = des_pos.at(2);
  cog_pose.orientation = tf::createQuaternionMsgFromYaw(des_pos.at(5)); /* special: only get yaw angle */

  /* joint state:  */
  auto actuator_vector = discrete_path_planner_->getPathConst().at(0).getActuatorStateConst();
  for(int i = 0; i < joint_num; i++)
    actuator_vector(robot_model_ptr_->getLinkJointIndex().at(i)) = des_pos.at(6 + i);

  MultilinkState state_tmp(robot_model_ptr_,
                           tf::createQuaternionFromRPY(att_msg.roll, att_msg.pitch, 0), cog_pose,
                           actuator_vector);
  display_robot_state.state = state_tmp.getRootActuatorStateConst<moveit_msgs::RobotState>();
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
}

void SqueezeNavigation::continuousPath(const std::vector<MultilinkState>& discrete_path)
{
  assert(discrete_path.size() > 0);

  int joint_num = robot_model_ptr_->getLinkJointIndex().size();

  /* insert data */
  std::vector<std::vector<double> > control_point_list;
  for (int i = -1; i < (int)discrete_path.size() + 1; i++)
    {
      /* add one more start & end keypose to guarantee speed 0 */
      if (i < 0) i = 0;
      if (i >= discrete_path.size()) i = discrete_path.size() - 1;

      std::vector<double> control_point;

      // cog position
      const auto cog_pos = discrete_path.at(i).getCogPoseConst().position;
      control_point.push_back(cog_pos.x);
      control_point.push_back(cog_pos.y);
      control_point.push_back(cog_pos.z);

      // euler enagles
      tf::Matrix3x3 att(discrete_path.at(i).getBaselinkDesiredAttConst());
      double r, p, y; att.getRPY(r, p, y);
      control_point.push_back(r);
      control_point.push_back(p);
      /* TODO: use quaternion bspline: keep yaw euler angle continous */
      control_point.push_back(generateContinousEulerAngle(y, i));

      /* set joint state */
      for(auto itr : robot_model_ptr_->getLinkJointIndex())
        control_point.push_back(discrete_path.at(i).getActuatorStateConst()(itr));

      control_point_list.push_back(control_point);
    }

  bspline_ptr_->initialize(true, 0, trajectory_period_, bspline_degree_, control_point_list);
  std::vector<int> pos_indicies{0,1,2};
  bspline_ptr_->display3dPath(pos_indicies);

  ROS_INFO("Spline display finished.");
}

void SqueezeNavigation::robotOdomCallback(const nav_msgs::OdometryConstPtr& msg)
{
  robot_baselink_odom_ = *msg;
  real_odom_flag_ = true;
}

void SqueezeNavigation::robotJointStatesCallback(const sensor_msgs::JointStateConstPtr& joints_msg)
{
  if(!real_odom_flag_) return;

  if(discrete_path_.size() == 0) return; // do not have valid path

  auto actuator_vector = robot_model_ptr_->jointMsgToKdl(*joints_msg);
  geometry_msgs::Pose root_pose;
  MultilinkState::convertBaselinkPose2RootPose(robot_model_ptr_, robot_baselink_odom_.pose.pose, actuator_vector, root_pose);

  /* check collision */
  if(discrete_path_search_method_type_ == 0) // sampling base
    discrete_path_planner_->checkCollision(MultilinkState(robot_model_ptr_, root_pose, actuator_vector));
}
