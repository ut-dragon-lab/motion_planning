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

  std::vector<IirFilter> low_path_filter_vec_;
  std::vector<MultilinkState> fft_path_;
  std_msgs::ColorRGBA desired_state_color_;

  std::vector<float> timevec_test_;
}


SqueezeNavigation::SqueezeNavigation(ros::NodeHandle nh, ros::NodeHandle nhp):
  nh_(nh), nhp_(nhp), move_start_flag_(false)
{
  rosParamInit();

  /* ros pub/sub, srv */
  plan_start_flag_sub_ = nh_.subscribe("/plan_start", 1, &SqueezeNavigation::planStartCallback, this);
  move_start_flag_sub_ = nh_.subscribe("/move_start", 1, &SqueezeNavigation::moveStartCallback, this);
  adjust_initial_state_sub_ = nh_.subscribe("/adjust_robot_initial_state", 1, &SqueezeNavigation::adjustInitalStateCallback, this);
  flight_config_sub_ = nh_.subscribe("/flight_config_cmd", 1,  &SqueezeNavigation::flightConfigCallback, this);

  std::string topic_name;
  nhp_.param("joint_control_topic_name", topic_name, std::string("joints_ctrl"));
  joints_ctrl_pub_ = nh_.advertise<sensor_msgs::JointState>(topic_name, 1);
  flight_nav_pub_ = nh_.advertise<aerial_robot_msgs::FlightNav>("/uav/nav", 1);
  se3_roll_pitch_nav_pub_ = nh_.advertise<spinal::DesireCoord>("/desire_coordinate", 1);
  desired_path_pub_ = nh_.advertise<moveit_msgs::DisplayRobotState>("/desired_robot_state", 1);

  if(!headless_)
    {
      std::string topic_name;
      robot_baselink_odom_sub_ = nh_.subscribe("/uav/baselink/odom", 1, &SqueezeNavigation::robotOdomCallback, this);
      nhp_.param("joint_state_topic_name", topic_name, std::string("joint_states"));
      robot_joint_states_sub_ = nh_.subscribe(topic_name, 1, &SqueezeNavigation::robotJointStatesCallback, this);
    }

  /* robot model */
  //////// temporary //////////////
  nhp_.param("motion_type", motion_type_, 0);
  if (motion_type_ == motion_type::SE2) //SE2
    robot_model_ptr_ = boost::shared_ptr<TransformController>(new TransformController(nh, nhp, false));
  else //SE3
    robot_model_ptr_ = boost::shared_ptr<TransformController>(new DragonTransformController(nh, nhp, false));
  /////////////////////////////////

  /* discrete path search */
  std::string discrete_path_search_method_name;
  nhp_.param("discrete_path_search_method_name", discrete_path_search_method_name, std::string("none"));
  discrete_path_planner_ = plugin_loader.createInstance(discrete_path_search_method_name);
  discrete_path_planner_->initialize(nh_, nhp_, robot_model_ptr_);
  //////// temporary //////////////
  joint_num_ = robot_model_ptr_->getActuatorJointMap().size();
  assert(joint_num_ > 0);

  /* continous path generator */
  /* e.g.: bspline */
  bspline_ptr_ = boost::shared_ptr<TinysplineInterface>(new TinysplineInterface(nh_, nhp_));
  control_pts_ptr_ = boost::shared_ptr<bspline_generator::ControlPoints>(new bspline_generator::ControlPoints());
  control_pts_ptr_->num = 0;
  control_pts_ptr_->degree = bspline_degree_;
  control_pts_ptr_->control_pts.layout.dim.push_back(std_msgs::MultiArrayDimension());
  control_pts_ptr_->control_pts.layout.dim.push_back(std_msgs::MultiArrayDimension());
  control_pts_ptr_->control_pts.layout.dim[0].label = "height";
  control_pts_ptr_->control_pts.layout.dim[1].label = "width";

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
      if(!discrete_path_planner_->plan()) return;
    }

  /* debug: low path filter */
  double rx_freq, cutoff_freq;
  nhp_.param("rx_freq", rx_freq, 100.0);
  nhp_.param("cutoff_freq", cutoff_freq, 20.0);
  low_path_filter_vec_.clear();
  low_path_filter_vec_.push_back(IirFilter(rx_freq, cutoff_freq));


  /* filtering by FFT */
  double cutoff_rate, cutoff_band;
  nhp_.param("cutoff_rate", cutoff_rate, 0.5);
  nhp_.param("cutoff_band", cutoff_band, 0.1);

#if 0
  double start_time = ros::Time::now().toSec();
  std::vector< std::vector<float> > timevec_array(6 + joint_num_);
  for(auto itr: discrete_path_planner_->getPathConst())
    {
      /* root position */
      timevec_array.at(0).push_back(itr.getRootPoseConst().position.x);
      timevec_array.at(1).push_back(itr.getRootPoseConst().position.y);
      timevec_array.at(2).push_back(itr.getRootPoseConst().position.z);

      /* attitdue, TODO: use Quaternion FFT */
      tf::Quaternion q;
      tf::quaternionMsgToTF(itr.getRootPoseConst().orientation, q);
      double r, p, y;
      tf::Matrix3x3(q).getRPY(r, p, y);
      timevec_array.at(3).push_back(r);
      timevec_array.at(4).push_back(p);
      timevec_array.at(5).push_back(y);

      /* joint state */
      for(int j = 0; j < joint_num_; j++)
        {
          int index = robot_model_ptr_->getActuatorJointMap().at(j);
          timevec_array.at(6 + j).push_back(itr.getActuatorStateConst().position.at(index));
        }
    }
  for(auto itr = timevec_array.begin(); itr != timevec_array.end(); itr++)
    {
      Eigen::FFT<float> fft;
      std::vector<std::complex<float> > freqvec;
      fft.fwd(freqvec, *itr);

      for(int index = 0; index < freqvec.size(); index++)
        {
          if(index > freqvec.size() * cutoff_rate)
            {
              freqvec.at(index).real(0);
              freqvec.at(index).imag(0);
            }
        }
      fft.inv(*itr, freqvec);
    }
  for(int index = 0; index < discrete_path_planner_->getPathConst().size(); index++)
    {
      MultilinkState robot_state = discrete_path_planner_->getStateConst(0);
      robot_state.getRootPoseNonConst().position.x = timevec_array.at(0).at(index);
      robot_state.getRootPoseNonConst().position.y = timevec_array.at(1).at(index);
      robot_state.getRootPoseNonConst().position.z = timevec_array.at(2).at(index);

      robot_state.getCogPoseNonConst().orientation = tf::createQuaternionMsgFromRollPitchYaw(timevec_array.at(3).at(index), timevec_array.at(4).at(index), timevec_array.at(5).at(index));

      sensor_msgs::JointState actuator_state = robot_state.getActuatorStateConst();
      for(int j = 0; j < joint_num_; j++)
        {
          actuator_state.position.at(robot_model_ptr_->getActuatorJointMap().at(j)) = timevec_array.at(6 + j).at(index);
        }

      robot_state.targetRootPose2TargetBaselinkPose(robot_model_ptr_);
      fft_path_.push_back(robot_state);
    }

  ROS_ERROR("fft time: %f", ros::Time::now().toSec() - start_time);

#endif


  Eigen::FFT<float> fft;
  std::vector<std::complex<float> > freqvec;
  for(auto itr: discrete_path_planner_->getPathConst())
    timevec_test_.push_back(itr.getRootPoseConst().position.y);
  fft.fwd(freqvec, timevec_test_);

  for(int index = 0; index < freqvec.size(); index++)
    {
      if(index < freqvec.size() * cutoff_rate + cutoff_band / 2  && index > freqvec.size() * cutoff_rate + cutoff_band / 2)
        {
        freqvec.at(index).real(0);
          freqvec.at(index).imag(0);
        }
      //timevec_test_.at(index) = std::abs(freqvec.at(index));
    }
  fft.inv(timevec_test_, freqvec);

  for(int i = 0; i < 100; i++)
    {
      double temp;
      low_path_filter_vec_.at(0).filterFunction(discrete_path_planner_->getStateConst(0).getRootPoseConst().position.y, temp);
    }

  if (discrete_path_debug_flag_)
    {
      move_start_flag_ = true;
      return;
    }

  /* continuous path */
  continuousPath(discrete_path_planner_->getPathConst());
}


void SqueezeNavigation::adjustInitalStateCallback(const std_msgs::Empty msg)
{
  if(control_pts_ptr_->num == 0)
    {
      ROS_ERROR("have not get planned path");
      return;
    }

  std::vector<double> initial_state = getKeypose(0);

  /* publish uav nav */
  aerial_robot_msgs::FlightNav nav_msg;
  nav_msg.header.frame_id = std::string("/world");
  nav_msg.header.stamp = ros::Time::now();

  /* x & y */
  nav_msg.control_frame = nav_msg.WORLD_FRAME;
  nav_msg.target = nav_msg.COG;
  nav_msg.pos_xy_nav_mode = nav_msg.POS_MODE;
  nav_msg.target_pos_x = initial_state[0];
  nav_msg.target_pos_y = initial_state[1];

  /* z */
  if(motion_type_ == motion_type::SE2)
    {
      nav_msg.pos_z_nav_mode = nav_msg.NO_NAVIGATION;
    }
  else if(motion_type_ == motion_type::SE3)
    {
      nav_msg.pos_z_nav_mode = nav_msg.POS_MODE;
      nav_msg.target_pos_z = initial_state[2];
    }

  /* yaw */
  nav_msg.psi_nav_mode = nav_msg.POS_MODE;
  nav_msg.target_psi = initial_state[5];
  flight_nav_pub_.publish(nav_msg);

  /* joint states */
  sensor_msgs::JointState joints_msg;
  joints_msg.header = nav_msg.header;
  for (int i = 0; i < joint_num_; ++i) joints_msg.position.push_back(initial_state[6 + i]);
  joints_ctrl_pub_.publish(joints_msg);

  /* se3: roll & pitch */
  spinal::DesireCoord att_msg;
  att_msg.roll = initial_state[3];
  att_msg.pitch = initial_state[4];
  se3_roll_pitch_nav_pub_.publish(att_msg);
}

void SqueezeNavigation::moveStartCallback(const std_msgs::Empty msg)
{
  if(control_pts_ptr_->num == 0)
    {
      ROS_ERROR("have not get planned path");
      return;
    }

  move_start_flag_ = true;
  move_start_time_ = ros::Time::now().toSec();
  ROS_INFO("[SqueezeNavigation] Receive move start topic.");
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
  if (!move_start_flag_) return;

  moveit_msgs::DisplayRobotState display_robot_state;

  /* some special visualize process, such as ENV */
  if(!headless_) discrete_path_planner_->visualizeFunc();

  /* test (debug) the discrete path */
  if(discrete_path_debug_flag_)
    {
#if 0
      display_robot_state.state = discrete_path_planner_->getStateConst(state_index_++).getVisualizeRobotStateConst();

      if(state_index_ == discrete_path_planner_->getPathConst().size()) state_index_ = 0;
      desired_path_pub_.publish(display_robot_state);
#endif
      /* debug */
#if 1
      if(state_index_ == discrete_path_planner_->getPathConst().size()) return;

      display_robot_state.state.joint_state.header.seq = state_index_;

      /*
      display_robot_state.state.joint_state.name.push_back("root_x");
      display_robot_state.state.joint_state.position.push_back(discrete_path_planner_->getStateConst(state_index_).getRootPoseConst().position.x);
      display_robot_state.state.joint_state.name.push_back("root_x_fft");
      display_robot_state.state.joint_state.position.push_back(fft_path_.at(state_index_).getRootPoseConst().position.x);
      display_robot_state.state.joint_state.name.push_back("root_y");
      display_robot_state.state.joint_state.position.push_back(discrete_path_planner_->getStateConst(state_index_).getRootPoseConst().position.y);
      display_robot_state.state.joint_state.name.push_back("root_y_fft");
      display_robot_state.state.joint_state.position.push_back(fft_path_.at(state_index_).getRootPoseConst().position.y);
      display_robot_state.state.joint_state.name.push_back("root_z");
      display_robot_state.state.joint_state.position.push_back(discrete_path_planner_->getStateConst(state_index_).getRootPoseConst().position.z);
      display_robot_state.state.joint_state.name.push_back("root_z_fft");
      display_robot_state.state.joint_state.position.push_back(fft_path_.at(state_index_).getRootPoseConst().position.z);
      */
      /*
      display_robot_state.state.joint_state.name.push_back("cog_x");
      display_robot_state.state.joint_state.position.push_back(discrete_path_planner_->getStateConst(state_index_).getCogPoseConst().position.x);
      display_robot_state.state.joint_state.name.push_back("cog_x_fft");
      display_robot_state.state.joint_state.position.push_back(fft_path_.at(state_index_).getCogPoseConst().position.x);
      display_robot_state.state.joint_state.name.push_back("cog_y");
      display_robot_state.state.joint_state.position.push_back(discrete_path_planner_->getStateConst(state_index_).getCogPoseConst().position.y);
      display_robot_state.state.joint_state.name.push_back("cog_y_fft");
      display_robot_state.state.joint_state.position.push_back(fft_path_.at(state_index_).getCogPoseConst().position.y);
      display_robot_state.state.joint_state.name.push_back("cog_z");
      display_robot_state.state.joint_state.position.push_back(discrete_path_planner_->getStateConst(state_index_).getCogPoseConst().position.z);
      display_robot_state.state.joint_state.name.push_back("cog_z_fft");
      display_robot_state.state.joint_state.position.push_back(fft_path_.at(state_index_).getCogPoseConst().position.z);
      */

      /*
      double filtered_value;
      low_path_filter_vec_.at(0).filterFunction(discrete_path_planner_->getStateConst(state_index_).getRootPoseConst().position.y, filtered_value);
      display_robot_state.state.joint_state.name.push_back("root_z");
      display_robot_state.state.joint_state.position.push_back(discrete_path_planner_->getStateConst(state_index_).getRootPoseConst().position.y);
      display_robot_state.state.joint_state.name.push_back("cog_x_filtered");
      display_robot_state.state.joint_state.position.push_back(filtered_value);
      */

      display_robot_state.state.joint_state.name.push_back("joint1_pitch");
      display_robot_state.state.joint_state.position.push_back(discrete_path_planner_->getStateConst(state_index_).getRootPoseConst().position.y);
      display_robot_state.state.joint_state.name.push_back("joint1_yaw");
      display_robot_state.state.joint_state.position.push_back(timevec_test_.at(state_index_));

      /*
      double r,p,y;
      tf::Matrix3x3(discrete_path_planner_->getStateConst(state_index_).getBaselinkDesiredAttConst()).getRPY(r,p,y);
      display_robot_state.state.joint_state.name.push_back("baselink_r");
      display_robot_state.state.joint_state.position.push_back(r);
      display_robot_state.state.joint_state.name.push_back("baselink_p");
      display_robot_state.state.joint_state.position.push_back(p);
      display_robot_state.state.joint_state.name.push_back("baselink_y");
      display_robot_state.state.joint_state.position.push_back(y);
      */

      /*
      for(auto itr : robot_model_ptr_->getActuatorJointMap())
        {
          display_robot_state.state.joint_state.name.push_back(discrete_path_planner_->getStateConst(state_index_).getActuatorStateConst().name.at(itr));
          display_robot_state.state.joint_state.position.push_back(discrete_path_planner_->getStateConst(state_index_).getActuatorStateConst().position.at(itr));
        }
      */

      desired_path_pub_.publish(display_robot_state);

      state_index_++;
#endif
      return;
    }

  double cur_time = ros::Time::now().toSec() - move_start_time_;
  std::vector<double> des_pos = bspline_ptr_->evaluate(cur_time + 1.0 / controller_freq_);
  std::vector<double> des_vel = bspline_ptr_->evaluateDerive(cur_time);

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
  for (int i = 0; i < joint_num_; ++i)
    joints_msg.position.push_back(boost::algorithm::clamp(des_pos[6 + i], robot_model_ptr_->joint_angle_min_, robot_model_ptr_->joint_angle_max_));
  joints_ctrl_pub_.publish(joints_msg);

  if (debug_verbose_)
    {
      std::cout << "joints: ";
      for (int i = 0; i < joint_num_; ++i) std::cout << joints_msg.position[i] << ", ";
      std::cout << "\n\n";
    }

  /* se3: roll & pitch */
  spinal::DesireCoord att_msg;
  att_msg.roll = des_pos[3];
  att_msg.pitch = des_pos[4];
  se3_roll_pitch_nav_pub_.publish(att_msg);

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
  sensor_msgs::JointState joint_state = discrete_path_planner_->getPathConst().at(0).getActuatorStateConst();
  for(int i = 0; i < joint_num_; i++)
    joint_state.position.at(robot_model_ptr_->getActuatorJointMap().at(i)) = des_pos.at(6 + i);

  MultilinkState state_tmp;
  state_tmp.setCogPose(cog_pose);
  state_tmp.setActuatorState(joint_state);
  state_tmp.setBaselinkDesiredAtt(tf::createQuaternionFromRPY(att_msg.roll, att_msg.pitch, 0));
  state_tmp.cogPose2RootPose(robot_model_ptr_);
  display_robot_state.state = state_tmp.getVisualizeRobotStateConst();

  /* set color */
  for(auto itr: robot_model_ptr_->getRobotModel().links_)
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


  /* insert data */
  control_pts_ptr_->num = discrete_path.size() + 2;
  control_pts_ptr_->dim = 6 + joint_num_;

  control_pts_ptr_->is_uniform = true; // TODO: SHI FAN
  control_pts_ptr_->start_time = 0.0;
  control_pts_ptr_->end_time = trajectory_period_;
  control_pts_ptr_->control_pts.layout.dim[0].size = control_pts_ptr_->num;
  control_pts_ptr_->control_pts.layout.dim[1].size = control_pts_ptr_->dim;
  control_pts_ptr_->control_pts.layout.dim[0].stride = control_pts_ptr_->num * control_pts_ptr_->dim;
  control_pts_ptr_->control_pts.layout.dim[1].stride = control_pts_ptr_->dim;
  control_pts_ptr_->control_pts.layout.data_offset = 0;

  control_pts_ptr_->control_pts.data.resize(0);
  for (int i = -1; i < (int)discrete_path.size() + 1; i++)
    {
      /* add one more start & end keypose to guarantee speed 0 */
      int id = i;
      if (id < 0)
        id = 0;
      else if (id >= discrete_path.size())
        id = discrete_path.size() - 1;
      int index_s = id * control_pts_ptr_->dim;

      /* general state: pos_x, pos_y, pos_z, roll, pitch, yaw, joints */
      {
        // cog position
        control_pts_ptr_->control_pts.data.push_back(discrete_path.at(id).getCogPoseConst().position.x);
        control_pts_ptr_->control_pts.data.push_back(discrete_path.at(id).getCogPoseConst().position.y);
        control_pts_ptr_->control_pts.data.push_back(discrete_path.at(id).getCogPoseConst().position.z);

        // convert to euler enagles
        tf::Matrix3x3 att(discrete_path.at(id).getBaselinkDesiredAttConst());
        double r, p, y; att.getRPY(r, p, y);
        control_pts_ptr_->control_pts.data.push_back(r);
        control_pts_ptr_->control_pts.data.push_back(p);
        /* TODO: use quaternion bspline: keep yaw euler angle continous */
        control_pts_ptr_->control_pts.data.push_back(generateContinousEulerAngle(y, id));
      }

      /* set joint state */
      for(auto itr : robot_model_ptr_->getActuatorJointMap())
        control_pts_ptr_->control_pts.data.push_back(discrete_path.at(id).getActuatorStateConst().position.at(itr));
    }

  bspline_ptr_->bsplineParamInput(control_pts_ptr_);
  bspline_ptr_->getDerive();
  bspline_ptr_->splinePathDisplay();
  bspline_ptr_->controlPolygonDisplay();
  ROS_INFO("Spline display finished.");
}

std::vector<double> SqueezeNavigation::getKeypose(int id)
{
  std::vector<double> keypose;
  int start_index = id * control_pts_ptr_->dim;
  if (start_index < 0){
    ROS_ERROR("[BsplineGenerator] visited keypose id%d is less than 0.", id);
    start_index = 0;
  }
  else if (start_index > control_pts_ptr_->control_pts.data.size() - control_pts_ptr_->dim){
    ROS_ERROR("[BsplineGenerator] visited keypose id%d is larger than bound.", id);
    start_index = control_pts_ptr_->control_pts.data.size() - control_pts_ptr_->dim;
  }
  for (int i = 0; i < control_pts_ptr_->dim; ++i)
    keypose.push_back(control_pts_ptr_->control_pts.data[start_index + i]);

  return keypose;
}

void SqueezeNavigation::robotOdomCallback(const nav_msgs::OdometryConstPtr& msg)
{
  robot_baselink_odom_ = *msg;
  real_odom_flag_ = true;
}

void SqueezeNavigation::robotJointStatesCallback(const sensor_msgs::JointStateConstPtr& joints_msg)
{
  if(!real_odom_flag_) return;

  if(control_pts_ptr_->num == 0) return; // do not have valid path

  /* joint state */
  MultilinkState state_tmp;

  state_tmp.setActuatorState(*joints_msg);
  state_tmp.baselinkPose2RootPose(robot_baselink_odom_.pose.pose, robot_model_ptr_);

  /* check collision */
  if(discrete_path_search_method_type_ == 0) // sampling base
    discrete_path_planner_->checkCollision(state_tmp);
}
