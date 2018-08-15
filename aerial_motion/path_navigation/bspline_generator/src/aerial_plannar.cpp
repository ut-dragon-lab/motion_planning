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

#include <bspline_generator/aerial_plannar.h>
#include <tf/transform_broadcaster.h>

AerialPlannar::AerialPlannar(ros::NodeHandle nh, ros::NodeHandle nhp):
  nh_(nh), nhp_(nhp),
  move_start_flag_(false),
  motion_type_(gap_passing::PlanningMode::SE2)
{
  bspline_ptr_ = boost::shared_ptr<TinysplineInterface>(new TinysplineInterface(nh_, nhp_));
  control_pts_ptr_ = new bspline_generator::ControlPoints();
  control_pts_ptr_->control_pts.layout.dim.push_back(std_msgs::MultiArrayDimension());
  control_pts_ptr_->control_pts.layout.dim.push_back(std_msgs::MultiArrayDimension());
  control_pts_ptr_->control_pts.layout.dim[0].label = "height";
  control_pts_ptr_->control_pts.layout.dim[1].label = "width";
  nhp_.param("control_frequency", controller_freq_, 40.0);
  nhp_.param("trajectory_period", trajectory_period_, 100.0);
  nhp_.param("bspline_degree", bspline_degree_, 5);

  nhp_.param("debug_verbose", debug_verbose_, false);
  nhp_.param("joint_upperbound", joint_upperbound_, 1.57);
  nhp_.param("joint_lowerbound", joint_lowerbound_, -1.57);

  move_start_flag_sub_ = nh_.subscribe("/move_start", 1, &AerialPlannar::moveStartCallback, this);
  adjust_initial_state_sub_ = nh_.subscribe("/adjust_robot_initial_state", 1, &AerialPlannar::adjustInitalStateCallback, this);
  flight_config_sub_ = nh_.subscribe("/flight_config_cmd", 1,  &AerialPlannar::flightConfigCallback, this);
  desired_state_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/desired_state", 1);
  std::string topic_name;
  nhp_.param("joint_control_topic_name", topic_name, std::string("joints_ctrl"));
  joints_ctrl_pub_ = nh_.advertise<sensor_msgs::JointState>(topic_name, 1);
  flight_nav_pub_ = nh_.advertise<aerial_robot_msgs::FlightNav>("/uav/nav", 1);
  se3_roll_pitch_nav_pub_ = nh_.advertise<spinal::DesireCoord>("/desire_coordinate", 1);
  spline_init_thread_ = boost::thread(boost::bind(&AerialPlannar::splineInitThread, this));
  navigate_timer_ = nh_.createTimer(ros::Duration(1.0 / controller_freq_), &AerialPlannar::navigate, this);
}

AerialPlannar::~AerialPlannar()
{
  spline_init_thread_.interrupt();
  spline_init_thread_.join();
  delete control_pts_ptr_;
}

void AerialPlannar::splineInitThread()
{
  ROS_INFO("bspline initalized.");
  waitForKeyposes();
}
void AerialPlannar::adjustInitalStateCallback(const std_msgs::Empty msg)
{
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
  if(motion_type_ == gap_passing::PlanningMode::SE2)
    {
      nav_msg.pos_z_nav_mode = nav_msg.NO_NAVIGATION;
    }
  else if(motion_type_ == gap_passing::PlanningMode::SE3)
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

void AerialPlannar::moveStartCallback(const std_msgs::Empty msg)
{
  move_start_flag_ = true;
  move_start_time_ = ros::Time::now().toSec();
  ROS_INFO("[AerialPlannar] Receive move start topic.");
}

void AerialPlannar::flightConfigCallback(const spinal::FlightConfigCmdConstPtr msg)
{
  if(msg->cmd == spinal::FlightConfigCmd::FORCE_LANDING_CMD)
    {
      ROS_INFO("[AerialPlannar] Receive force landing command, stop navigation.");
      move_start_flag_ = false;
    }
}

void AerialPlannar::navigate(const ros::TimerEvent& event)
{
  if (!move_start_flag_) return;

  double cur_time = ros::Time::now().toSec() - move_start_time_;
  std::vector<double> des_pos = bspline_ptr_->evaluate(cur_time + 1.0 / controller_freq_);
  std::vector<double> des_vel = bspline_ptr_->evaluateDerive(cur_time);

  /* send the desired continous path for display (visualize) */
  std_msgs::Float64MultiArray desired_state;
  desired_state.data = des_pos;
  desired_state_pub_.publish(desired_state);

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
  if(motion_type_ == gap_passing::PlanningMode::SE2)
    {
      nav_msg.pos_z_nav_mode = nav_msg.NO_NAVIGATION;
    }
  else if(motion_type_ == gap_passing::PlanningMode::SE3)
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
    joints_msg.position.push_back(boost::algorithm::clamp(des_pos[6 + i], joint_lowerbound_, joint_upperbound_));
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
      ROS_INFO("[AerialPlannar] Finish Navigation");
      move_start_flag_ = false;
    }


}

void AerialPlannar::waitForKeyposes()
{
  ros::ServiceClient sampling_plannar_client = nh_.serviceClient<gap_passing::Keyposes>("keyposes_server");

  gap_passing::Keyposes keyposes_srv;
  while (!sampling_plannar_client.call(keyposes_srv)){
    ROS_INFO_THROTTLE(1.0, "waiting for the discrete key poses");
  }
  if (!keyposes_srv.response.available_flag){
    ROS_WARN("keyposes is not available, try again");
    waitForKeyposes();
  }
  else if (keyposes_srv.response.states_cnt == 0){
    ROS_WARN("keyposes size is 0, try again");
    waitForKeyposes();
  }
  else{
    joint_num_ = keyposes_srv.response.dim - 6; // set joint_num
    motion_type_ = keyposes_srv.response.motion_type;
    control_pts_ptr_->num = keyposes_srv.response.states_cnt + 2;
    control_pts_ptr_->dim = keyposes_srv.response.dim;
    control_pts_ptr_->degree = bspline_degree_;
    control_pts_ptr_->is_uniform = true; // TODO: shi
    control_pts_ptr_->start_time = 0.0;
    control_pts_ptr_->end_time = trajectory_period_;
    control_pts_ptr_->control_pts.layout.dim[0].size = control_pts_ptr_->num;
    control_pts_ptr_->control_pts.layout.dim[1].size = control_pts_ptr_->dim;
    control_pts_ptr_->control_pts.layout.dim[0].stride = control_pts_ptr_->num * control_pts_ptr_->dim;
    control_pts_ptr_->control_pts.layout.dim[1].stride = control_pts_ptr_->dim;
    control_pts_ptr_->control_pts.layout.data_offset = 0;

    control_pts_ptr_->control_pts.data.resize(0);
    for (int i = -1; i < (int)keyposes_srv.response.states_cnt + 1; i++)
      {
        // add one more start & end keypose to guarantee speed 0
        int id = i;
        if (id < 0)
          id = 0;
        else if (id >= keyposes_srv.response.states_cnt)
          id = keyposes_srv.response.states_cnt - 1;
        int index_s = id * keyposes_srv.response.dim;

        /* general state: pos_x, pos_y, pos_z, roll, pitch, yaw, joints */
        for (int j = 0; j < 5; ++j)
          control_pts_ptr_->control_pts.data.push_back(keyposes_srv.response.data.data[index_s + j]);
        //ROS_INFO("roll: %f, pitch: %f", keyposes_srv.response.data.data[index_s + 3], keyposes_srv.response.data.data[index_s + 4]);
        /* keep yaw euler angle continous */
        control_pts_ptr_->control_pts.data.push_back(generateContinousEulerAngle(keyposes_srv.response.data.data[index_s + 5], id));
        for (int j = 6; j < keyposes_srv.response.dim; ++j)
          control_pts_ptr_->control_pts.data.push_back(keyposes_srv.response.data.data[index_s + j]);
    }

    bspline_ptr_->bsplineParamInput(control_pts_ptr_);
    bspline_ptr_->getDerive();
    bspline_ptr_->splinePathDisplay();
    bspline_ptr_->controlPolygonDisplay();
    ROS_INFO("Spline display finished.");
  }
}

std::vector<double> AerialPlannar::getKeypose(int id)
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

