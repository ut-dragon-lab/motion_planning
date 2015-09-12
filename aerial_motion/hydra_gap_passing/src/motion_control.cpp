#include <hydra_gap_passing/motion_control.h>


MotionControl::MotionControl(ros::NodeHandle nh, ros::NodeHandle nhp, boost::shared_ptr<TransformController>  transform_controller): nh_(nh), nhp_(nhp)
{
  nhp_.param("log_flag", log_flag_, false);
  nhp_.param("play_log_path", play_log_path_, false);

  nhp_.param("joint_cmd_rate", joint_cmd_rate_, 50.0);
  nhp_.param("move_cmd_rate", move_cmd_rate_, 40.0);
  nhp_.param("gain_cmd_rate", gain_cmd_rate_, 40.0); //old system:20(hz)

  nhp_.param("backward_offset", backward_offset_, 100);
  nhp_.param("forward_offset", forward_offset_, 100);

  if(play_log_path_) log_flag_ = false;
  nhp_.param("file_name", file_name_, std::string("planning_log.txt"));

  //subscriber
  control_flag_sub_ = nh_.subscribe<std_msgs::UInt8>("hydra/motion_control", 1, &MotionControl::controlFlagCallback, this, ros::TransportHints().tcpNoDelay());

  robot_states_sub_ = nh_.subscribe<aerial_robot_base::States>("ground_truth/pose", 1, &MotionControl::robotStateCallback, this, ros::TransportHints().tcpNoDelay());

  joint_values_sub_ = nh_.subscribe<sensor_msgs::JointState>("joint_states", 1, &MotionControl::jointStateCallback, this, ros::TransportHints().tcpNoDelay());

  //publisher
  joint_cmd_pub_ = nh_.advertise<sensor_msgs::JointState>("hydra/joints_ctrl", 1);
  /*TODO*/
  move_cmd_pub_ = nh_.advertise<aerial_robot_base::FlightNav>("hoge", 1);

  transform_controller_ = transform_controller;

  //init
  planning_path_.resize(0);
  planning_mode_ = hydra_gap_passing::PlanningMode::ONLY_JOINTS_MODE;

  minimum_x_performance_ = 1e6;
  minimum_y_performance_ = 1e6;
  minimum_x_performance_state_ = 0;
  minimum_y_performance_state_ = 0;
  semi_stable_states_ = 0;


  if(play_log_path_) planFromFile();

  real_states_.resize(3 + transform_controller_->getLinkNum() );

  control_flag_ = false;

  //control sub thread
  if(planning_mode_ != hydra_gap_passing::PlanningMode::ONLY_BASE_MODE)
    {
      joint_cmd_thread_ = boost::thread(boost::bind(&MotionControl::jointCmd, this));
      gain_cmd_thread_ = boost::thread(boost::bind(&MotionControl::gainCmd, this));

    }
  if(planning_mode_ != hydra_gap_passing::PlanningMode::ONLY_JOINTS_MODE)
    move_cmd_thread_ = boost::thread(boost::bind(&MotionControl::moveCmd, this));
}

MotionControl::~MotionControl()
{
  if(planning_mode_ != hydra_gap_passing::PlanningMode::ONLY_BASE_MODE)
    {
      joint_cmd_thread_.interrupt();
      gain_cmd_thread_.interrupt();
      joint_cmd_thread_.join();
      gain_cmd_thread_.join();
    }
  if(planning_mode_ != hydra_gap_passing::PlanningMode::ONLY_JOINTS_MODE)
    {
      move_cmd_thread_.interrupt();
      move_cmd_thread_.join();
    }
}

void MotionControl::getRealStates(std::vector<double>& real_states)
{
  boost::lock_guard<boost::mutex> lock(real_state_mutex_);
  for(int i = 0; i < (int)real_states_.size(); i++)
    real_states[i] = real_states_[i];
}
void MotionControl::setMoveBaseStates(std::vector<double> move_base_states)
{
  boost::lock_guard<boost::mutex> lock(real_state_mutex_);
  for(int i = 0; i < 3; i++)
    real_states_[i] = move_base_states[i];//x,y, theta
}
void MotionControl::setJointStates(std::vector<double> joint_states)
{
  boost::lock_guard<boost::mutex> lock(real_state_mutex_);
  for(int i = 0; i <  (int)joint_states.size(); i ++)
    real_states_[i + 3] = joint_states[i];
}

void MotionControl::planStoring(const ompl::base::StateStorage* plan_states, int planning_mode, const std::vector<double>& start_state, const std::vector<double>& goal_state,  double best_cost, double calculation_time)
{
  best_cost_ = best_cost;
  calculation_time_ = calculation_time;
  planning_mode_ = planning_mode;

  conf_values state;
  //use start state to initialize the state
  state.state_values.resize(start_state.size());
  state.state_values = start_state;
  state.stable_mode = TransformController::LQI_FOUR_AXIS_MODE;
  state.dist_thre_value = 1;
  //state.k = Eigen::MatrixXd::Zero(4, 12);
  state.angle_cos = 1;
  state.angle_sin = 0;
  state.control_mode = hydra_gap_passing::PlanningMode::POSITION_MODE;


  int state_list = (int)plan_states->size();

  if(planning_mode != hydra_gap_passing::PlanningMode::ORIGINAL_MODE)
    {
      for(int i = 0; i < state_list; i++)
        {
          if(planning_mode == hydra_gap_passing::PlanningMode::ONLY_JOINTS_MODE || planning_mode == hydra_gap_passing::PlanningMode::JOINTS_AND_BASE_MODE)
            {
              if(planning_mode == hydra_gap_passing::PlanningMode::ONLY_JOINTS_MODE)
                {
              //joints value
              state.state_values[3] = plan_states->getState(i)->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
              state.state_values[4] = plan_states->getState(i)->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];
              state.state_values[5] = plan_states->getState(i)->as<ompl::base::RealVectorStateSpace::StateType>()->values[2];
                }
              else if(planning_mode == hydra_gap_passing::PlanningMode::JOINTS_AND_BASE_MODE)
                {
                  const ompl::base::CompoundState* state_tmp = dynamic_cast<const ompl::base::CompoundState*>(plan_states->getState(i));
                  state.state_values[0] = state_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->getX();
                  state.state_values[1] = state_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->getY();
                  state.state_values[2] = state_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->getYaw();
                  state.state_values[3] = state_tmp->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[0];
                  state.state_values[4] = state_tmp->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[1];
                  state.state_values[5] = state_tmp->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[2];
                }

              // dist thre
              bool dist_thre = transform_controller_->distThreCheckFromJointValues(state.state_values, 3);
              if(!dist_thre) 
                {
                  state.dist_thre_value = 0;
                  ROS_ERROR("(singular pose, can not resolve the lqi control problem");
                }
              else state.dist_thre_value = 1;

              //minimum dist(x_axis, y_axis)
              std::vector<Eigen::Vector3d> links_origin_from_cog(3); //links_num
              transform_controller_->getLinksOriginFromCog(links_origin_from_cog);

              float x_minus_max_dist = 0, x_plus_max_dist = 0;
              float y_minus_max_dist = 0, y_plus_max_dist = 0;

              //std::cout << "state: " << i << std::endl;
              for(int l = 0; l < (int)links_origin_from_cog.size(); l ++)
                {
                  //std::cout << "link" << l +1 << ":\n"<< links_origin_from_cog[l] << std::endl;
                  //x
                  if(links_origin_from_cog[l](0) > 0 && links_origin_from_cog[l](0) > x_plus_max_dist)
                    x_plus_max_dist = links_origin_from_cog[l](0);
                  if(links_origin_from_cog[l](0) < 0 && links_origin_from_cog[l](0) < x_minus_max_dist)
                    x_minus_max_dist = links_origin_from_cog[l](0);
                  //y
                  if(links_origin_from_cog[l](1) > 0 && links_origin_from_cog[l](1) > y_plus_max_dist)
                    y_plus_max_dist = links_origin_from_cog[l](1);
                  if(links_origin_from_cog[l](1) < 0 && links_origin_from_cog[l](1) < y_minus_max_dist)
                    y_minus_max_dist = links_origin_from_cog[l](1);
                }
              //std::cout << " " << std::endl;

              if(x_plus_max_dist < minimum_x_performance_)
                {
                  minimum_x_performance_ = x_plus_max_dist;
                  minimum_x_performance_state_ = i;
                }
              if(-x_minus_max_dist < minimum_x_performance_)
                {
                  minimum_x_performance_ = -x_minus_max_dist;
                  minimum_x_performance_state_ = i;
                }

              if(y_plus_max_dist < minimum_y_performance_)
                {
                  minimum_y_performance_ = y_plus_max_dist;
                  minimum_y_performance_state_ = i;
                }
              if(-y_minus_max_dist < minimum_y_performance_)
                {
                  minimum_y_performance_ = -y_minus_max_dist;
                  minimum_y_performance_state_ = i;
                }


              //stability
              if(!transform_controller_->stabilityCheck()) 
                {//semi stable
                  semi_stable_states_ ++;
                  state.stable_mode = TransformController::LQI_THREE_AXIS_MODE;
                }
              else
                state.stable_mode = TransformController::LQI_FOUR_AXIS_MODE;

              //gains
              transform_controller_->hamiltonMatrixSolver(state.stable_mode);
              state.k = transform_controller_->getK();

              // if(state.stable_mode == TransformController::LQI_THREE_AXIS_MODE)
              //   std::cout << "K: " << state.k << std::endl;

              //imu-cog rotate angle
              transform_controller_->getRotateAngle(state.angle_cos, state.angle_sin);
            }
          else if(planning_mode == hydra_gap_passing::PlanningMode::ONLY_BASE_MODE || planning_mode == hydra_gap_passing::PlanningMode::ONLY_BASE_MODE + hydra_gap_passing::PlanningMode::ORIGINAL_MODE)
            {

              state.state_values[0] = plan_states->getState(i)->as<ompl::base::SE2StateSpace::StateType>()->getX();
              state.state_values[1] = plan_states->getState(i)->as<ompl::base::SE2StateSpace::StateType>()->getY();
              state.state_values[2] = plan_states->getState(i)->as<ompl::base::SE2StateSpace::StateType>()->getYaw();

            }
          //interation
          planning_path_.push_back(state);
        }
    }

  //file log
  if(log_flag_)
    {
      std::ofstream ofs;
      ofs.open( "planning_log.txt" );
      ofs << "start_state: " << start_state[0] << " " <<start_state[1] << " " << 
        start_state[2] << " " << start_state[3] <<  " " << start_state[4] <<
        " " << start_state[5] << std::endl;
      ofs << "goal_state: " << goal_state[0] << " " <<goal_state[1] << " " <<
        goal_state[2] << " " << goal_state[3] << " " <<goal_state[4] << " " << 
        goal_state[5] << std::endl;

      ofs << "states: " << state_list  << std::endl;
      ofs << "planning_mode: " << planning_mode << std::endl;
      ofs << "planning_time: " << calculation_time << std::endl;
      ofs << "motion_cost: " << best_cost << std::endl;
      ofs << "minimum_x_dist: " << minimum_x_performance_ << std::endl;
      ofs << "minimum_x_dist_state_entry: " << minimum_x_performance_state_  << std::endl;
      ofs << "minimum_y_dist: " << minimum_y_performance_ << std::endl;
      ofs << "minimum_y_dist_state_entry: " << minimum_y_performance_state_  << std::endl;
      ofs << "semi_stable_states: " << semi_stable_states_  << std::endl;

      for(int k = 0; k < state_list;  k++)
        {
          ofs << "state" << k << ": " << planning_path_[k].state_values[0] << " " 
              << planning_path_[k].state_values[1] << " " << planning_path_[k].state_values[2] 
              << " " <<planning_path_[k].state_values[3] << " " <<planning_path_[k].state_values[4] 
              << " " <<planning_path_[k].state_values[5] << " " <<planning_path_[k].stable_mode 
              << " " <<planning_path_[k].dist_thre_value << std::endl;
          ofs << "gains: ";
          for(int x = 0; x < planning_path_[k].k.rows(); x++)
            for(int y = 0; y < planning_path_[k].k.cols(); y++)
              ofs << planning_path_[k].k(x,y) << " ";
          ofs << std::endl;
          ofs << "rotate: " << planning_path_[k].angle_cos << " " << planning_path_[k].angle_sin << std::endl;
        }
      ofs << "end"  << std::endl;
      ofs.close();
    }
}

void MotionControl::planFromFile()
{
  std::ifstream ifs(file_name_.c_str());

  if(ifs.fail()) 
    {
    ROS_ERROR("File do not exist");
    return;
    }

  //hard code
  std::vector<double> start_state(6,0);
  std::vector<double> goal_state(6,0);
  int state_list;
  std::stringstream ss[11];
  std::string str;
  std::string header;
  //1 start and goal state
  std::getline(ifs, str);
  ss[0].str(str);
  ss[0] >> header >> start_state[0] >> start_state[1] >> start_state[2] 
     >> start_state[3] >> start_state[4] >> start_state[5];
  std::cout << header << std::endl;
  std::getline(ifs, str);
  ss[1].str(str);
  ss[1] >> header >> goal_state[0] >> goal_state[1] >> goal_state[2] 
     >> goal_state[3] >> goal_state[4] >> goal_state[5];
  std::cout << header << std::endl;
  ROS_WARN("from (%f, %f, %f, %f, %f, %f) to (%f, %f, %f, %f, %f, %f)",
           start_state[0], start_state[1], start_state[2],
           start_state[3], start_state[4], start_state[5],
           goal_state[0], goal_state[1], goal_state[2],
           goal_state[3], goal_state[4], goal_state[5]);

  //states size, planning time, motion cost
  std::getline(ifs, str);
  ss[2].str(str);
  ss[2] >> header >> state_list;
  std::cout << header << state_list <<std::endl;
  std::getline(ifs, str);
  ss[3].str(str);
  ss[3] >> header >> planning_mode_;
  std::cout << header << planning_mode_ <<std::endl;
  std::getline(ifs, str);
  ss[4].str(str);
  ss[4] >> header >> calculation_time_;
  std::cout << header << calculation_time_ <<std::endl;
  std::getline(ifs, str);
  ss[5].str(str);
  ss[5] >> header >> best_cost_;
  std::cout << header << best_cost_ <<std::endl;
  std::getline(ifs, str);
  ss[6].str(str);
  ss[6] >> header >> minimum_x_performance_;
  std::cout << header << minimum_x_performance_ << std::endl;
  std::getline(ifs, str);
  ss[7].str(str);
  ss[7] >> header >> minimum_x_performance_state_;
  std::cout << header << minimum_x_performance_state_  <<std::endl;
  std::getline(ifs, str);
  ss[8].str(str);
  ss[8] >> header >> minimum_y_performance_;
  std::cout << header << minimum_y_performance_ <<std::endl;
  std::getline(ifs, str);
  ss[9].str(str);
  ss[9] >> header >> minimum_y_performance_state_;
  std::cout << header << minimum_y_performance_state_  <<std::endl;
  std::getline(ifs, str);
  ss[10].str(str);
  ss[10] >> header >> semi_stable_states_;
  std::cout << header << semi_stable_states_  <<std::endl;

  planning_path_.resize(0);
  for(int k = 0; k < state_list;  k++)
    {
      std::stringstream ss_tmp[3];
      conf_values state;
      std::getline(ifs, str);
      ss_tmp[0].str(str);

      ss_tmp[0] >> header >> state.state_values[0] >> state.state_values[1]
       >> state.state_values[2] >> state.state_values[3] >>state.state_values[4]
       >> state.state_values[5] >> state.stable_mode >> state.dist_thre_value;

      std::getline(ifs, str);
      ss_tmp[1].str(str);
      ss_tmp[1] >> header;
      int rows =0, cols =0;
      if(state.stable_mode == TransformController::LQI_FOUR_AXIS_MODE)
        state.k = Eigen::MatrixXd::Zero(4, 12);
      if(state.stable_mode == TransformController::LQI_THREE_AXIS_MODE)
        state.k = Eigen::MatrixXd::Zero(4, 9);

      for(int x = 0; x < state.k.rows(); x++)
        for(int y = 0; y < state.k.cols(); y++)
          ss_tmp[1] >> state.k(x,y);

      std::getline(ifs, str);
      ss_tmp[2].str(str);
      ss_tmp[2] >> header >> state.angle_cos >> state.angle_sin;

      planning_path_.push_back(state);

      //debug
      //ROS_INFO("state%d: joint1: %f",k , planning_path_[k].state_values[3]);
      // std::cout << "state: " << k << std::endl;
      // if(!transform_controller_->distThreCheckFromJointValues(state.state_values, 3))
      //   ROS_ERROR("(singular pose, can not resolve the lqi control problem");

    }
}

void MotionControl::controlFlagCallback(const std_msgs::UInt8ConstPtr& control_msg)
{
  if(control_msg->data == 0) 
    {
      ROS_WARN("stop motion control");
      control_flag_ = false;
    }
  if(control_msg->data == 1) 
    {
      ROS_WARN("start motion control");
      control_flag_ = true;
    }
}

void MotionControl::jointStateCallback(const sensor_msgs::JointStateConstPtr& joint_state_msg)
{
  //debug
  //static int cnt = 0;

  std::vector<double> joint_state;
  joint_state.resize(0);
  for(int i = 0; i < (int)joint_state_msg->position.size(); i++)
    joint_state.push_back(joint_state_msg->position[i]);
  setJointStates(joint_state);

  // std::cout << "state: " << cnt << std::endl;
  // //test!!
  //  if(!transform_controller_->distThreCheckFromJointValues(joint_state))
  //    ROS_ERROR("(singular pose, can not resolve the lqi control problem");
  //  cnt++;
}
void MotionControl::robotStateCallback(const aerial_robot_base::StatesConstPtr& pose_state)
{
  std::vector<double> move_base_state;
  move_base_state.resize(0);
  move_base_state.push_back(pose_state->states[0].pos); //x
  move_base_state.push_back(pose_state->states[1].pos); //y
  move_base_state.push_back(pose_state->states[3].pos); //yaw
  setMoveBaseStates(move_base_state);
}


void MotionControl::moveCmd()
{
  //TODO
}

void MotionControl::jointCmd()
{
  ros::Rate loop_rate(joint_cmd_rate_);
  int joint_index = 0;

  while(ros::ok())
    {
      if(control_flag_) 
        {
          sensor_msgs::JointState ctrl_joint_msg;
          int temp = planning_path_[joint_index].state_values.size();
          ctrl_joint_msg.position.resize(0);
          for(int i = 3; i < temp; i ++)
            ctrl_joint_msg.position.push_back(planning_path_[joint_index].state_values[i]);
          joint_cmd_pub_.publish(ctrl_joint_msg);
          joint_index ++;
          //end, stop
          if(joint_index == planning_path_.size()) break;
        }
      loop_rate.sleep();
    }

}

void MotionControl::gainCmd()
{
  ros::Rate loop_rate(gain_cmd_rate_);
  int control_index = 0;

  while(ros::ok())
    {
      if(control_flag_) 
        {
          //find the index for real robot state
          std::vector<double> real_states(6,0);
          getRealStates(real_states);
          int start_index = ((control_index - backward_offset_ < 0)?  0 : control_index - backward_offset_);
          int goal_index =  ((control_index + forward_offset_ > planning_path_.size())? planning_path_.size()  : control_index + forward_offset_);
          double min = 1e6;

          for(int i = start_index ; i < goal_index; i++)
            {
              double diff = 0;
              for(int j = 3; j < 3+3; j++) //3 is link num
                diff += ((real_states[i] - planning_path_[i].state_values[i]) *
                         (real_states[i] - planning_path_[i].state_values[i]));
              if(diff < min) 
                {
                  min =diff;
                  control_index = i;
                }
            }
          ///debug
          ROS_INFO("control index is %d", control_index);

          //send gain and rotate angles 
          transform_controller_->setK(planning_path_[control_index].k, planning_path_[control_index].stable_mode);
          transform_controller_->setRotateAngle(planning_path_[control_index].angle_cos,planning_path_[control_index].angle_sin);

          transform_controller_->param2contoller();
          //end, stop?
          if(control_index == planning_path_.size() - 1) break;
        }
      loop_rate.sleep();
    }
}
