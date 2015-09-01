#include <hydra_gap_passing/motion_control.h>


MotionControl::MotionControl(ros::NodeHandle nh, ros::NodeHandle nhp, boost::shared_ptr<TransformController>  transform_controller): nh_(nh), nhp_(nhp)
{
  nhp_.param("log_flag", log_flag_, false);
  nhp_.param("play_log_path", play_log_path_, false);

  if(play_log_path_) log_flag_ = false;
  nhp_.param("file_name", file_name_, std::string("planning_log.txt"));


  transform_controller_ = transform_controller;

  //init
  planning_path_.resize(0);

  minimum_x_performance_ = 1e6;
  minimum_y_performance_ = 1e6;
  minimum_x_performance_state_ = 0;
  minimum_y_performance_state_ = 0;
  semi_stable_states_ = 0;

  if(play_log_path_) planFromFile();
}

void MotionControl::planStoring(const ompl::base::StateStorage* plan_states, int planning_mode, const std::vector<double>& start_state, const std::vector<double>& goal_state,  double best_cost, double calculation_time)
{
  best_cost_ = best_cost;
  calculation_time_ = calculation_time;

  conf_values state;
  //use start state to initialize the state
  state.state_values.resize(start_state.size());
  state.state_values = start_state;
  state.stable_mode = TransformController::LQI_FOUR_AXIS_MODE;
  state.dist_thre_value = 1;
  state.k = Eigen::MatrixXd::Zero(4, 12);
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
              if(!dist_thre) state.dist_thre_value = 0;
              else state.dist_thre_value = 1;

              //minimum dist(x_axis, y_axis)
              std::vector<Eigen::Vector3d> links_origin_from_cog(3); //links_num
              transform_controller_->getLinksOriginFromCog(links_origin_from_cog);
              float max_x_length = 0, max_y_length = 0;
              for(int l = 0; l < (int)links_origin_from_cog.size(); l ++)
                {
                  if(max_x_length < fabs(links_origin_from_cog[l](0)))
                    max_x_length = fabs(links_origin_from_cog[l](0));
                  if(max_y_length < fabs(links_origin_from_cog[l](1)))
                    max_y_length = fabs(links_origin_from_cog[l](1));
                }

              if(max_x_length < minimum_x_performance_)
                {
                  minimum_x_performance_ = max_x_length;
                  minimum_x_performance_state_ = i;
                }
              if(max_y_length < minimum_y_performance_)
                {
                  minimum_y_performance_ = max_y_length;
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
  std::stringstream ss;
  std::string str;
  std::string header;
  //1 start and goal state
  std::getline(ifs, str);
  ss.str(str);
  ss >> header >> start_state[0] >> start_state[1] >> start_state[2] 
     >> start_state[3] >> start_state[4] >> start_state[5];
  std::getline(ifs, str);
  ss.str(str);
  ss >> header >> goal_state[0] >> goal_state[1] >> goal_state[2] 
     >> goal_state[3] >> goal_state[4] >> goal_state[5];

  ROS_WARN("from (%f, %f, %f, %f, %f, %f) to (%f, %f, %f, %f, %f, %f)",
           start_state[0], start_state[1], start_state[2],
           start_state[3], start_state[4], start_state[5],
           goal_state[0], goal_state[1], goal_state[2],
           goal_state[3], goal_state[4], goal_state[5]);

  //states size, planning time, motion cost
  std::getline(ifs, str);
  ss.str(str);
  ss >> header >> state_list;
  std::getline(ifs, str);
  ss.str(str);
  ss >> header >> calculation_time_;
  std::getline(ifs, str);
  ss.str(str);
  ss >> header >> best_cost_;
  std::getline(ifs, str);
  ss.str(str);
  ss >> header >> minimum_x_performance_;
  std::getline(ifs, str);
  ss.str(str);
  ss >> header >> minimum_x_performance_state_;
  std::getline(ifs, str);
  ss.str(str);
  ss >> header >> minimum_y_performance_;
  std::getline(ifs, str);
  ss.str(str);
  ss >> header >> minimum_y_performance_state_;
  std::getline(ifs, str);
  ss.str(str);
  ss >> header >> semi_stable_states_;

  planning_path_.resize(0);
  for(int k = 0; k < state_list;  k++)
    {
      conf_values state;
      std::getline(ifs, str);
      ss.str(str);
      ss >> header >> state.state_values[0] >> state.state_values[1] 
         >> state.state_values[2] >> state.state_values[3] >> state.state_values[4] 
         >> state.state_values[5] >> state.stable_mode >> state.dist_thre_value;
      std::getline(ifs, str);
      ss.str(str);
      ss >> header;
      for(int x = 0; x < state.k.rows(); x++)
        for(int y = 0; y < state.k.cols(); y++)
          ss>> state.k(x,y);

      std::getline(ifs, str);
      ss.str(str);
      ss >> header >> state.angle_cos >> state.angle_sin;

      planning_path_.push_back(state);
    }
}
