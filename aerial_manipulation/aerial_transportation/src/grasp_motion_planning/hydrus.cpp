#include <aerial_transportation/grasp_control/hydrus.h>

namespace aerial_transportation
{
  void Hydrus::initialize(ros::NodeHandle nh, ros::NodeHandle nhp)
  {
    /* super class init */
    baseInit(nh, nhp);

    /* ros param init */
    rosParamInit();

    /* ros pub sub init */
    joint_ctrl_pub_ = nh_.advertise<sensor_msgs::JointState>(joint_ctrl_pub_name_, 1);
    aerial_grasping_flight_velocity_control_pub_ = nh_.advertise<std_msgs::UInt8>(aerial_grasping_flight_velocity_control_pub_name_, 1);
    joint_motors_sub_ = nh_.subscribe<dynamixel_msgs::MotorStateList>(joint_motors_sub_name_, 1, &Hydrus::jointMotorStatusCallback, this);
    joint_states_sub_ = nh_.subscribe<sensor_msgs::JointState>(joint_states_sub_name_, 1, &Hydrus::jointStatesCallback, this);

    if(!control_cheat_mode_)
      {/* plugin init */
        try
          {
            grasp_planning_loader_ = boost::shared_ptr< pluginlib::ClassLoader<grasp_planning::Base> > (new pluginlib::ClassLoader<grasp_planning::Base>("aerial_transportation", "grasp_planning::Base"));
            grasp_planning_method_ = grasp_planning_loader_->createInstance(grasp_planning_plugin_name_);
            grasp_planning_method_->initialize(nh_, nhp_);
            ROS_INFO("Load grasp planning plugin");
          }
        catch(pluginlib::PluginlibException& ex)
          {
            ROS_ERROR("The grasp planning plugin failed to load for some reason. Error: %s", ex.what());
          }
      }
    else
      ROS_WARN("Use Control Cheat Mode");

    {
      TODO from grasp form searching
        /****************************************************************/
        /* test the control approach part */
        /* claculate the joint angles for grasp from graspl_planner */
        if(control_test_flag_)
          {
            std::vector<float> v_hold_angle(contact_num_ - 1);
            std::vector<float> v_tighten_angle(contact_num_ - 1);
            std::vector<float> v_approach_angle(contact_num_ - 1);

            float approach_delta_angle = -0.4;//-0.4; // about 30deg
            float tighten_delta_angle =  0.09; // about 5deg

            getObjectGraspAngles(tighten_delta_angle, approach_delta_angle, contact_num_, v_hold_angle, v_tighten_angle, v_approach_angle);

            for(int i = 0; i < contact_num_ -1; i++)
              {
                ROS_INFO("grasp planner&control test: joint%d, approach_angle: %f, hold_angle: %f, tighten_angle: %f", i+1, v_approach_angle[i], v_hold_angle[i], v_tighten_angle[i]);
                v_best_theta_[i](0) = v_approach_angle[i];
              }

            /* calculate object_approach_offset[x,y,yaw] */
            object_approach_offset_x_ = 0;
            object_approach_offset_y_ = 0;
            object_approach_offset_yaw_ = 0;
            getObjectApproachOffset(v_approach_angle, object_approach_offset_x_, object_approach_offset_y_, object_approach_offset_yaw_);
            ROS_INFO("grasp planner&control test: base link: %d, object_approach_offset_x_: %f, object_approach_offset_y_: %f, object_approach_offset_yaw_: %f", approach_base_link_, object_approach_offset_x_, object_approach_offset_y_, object_approach_offset_yaw_);

          }

    
      /* base variables init */
      joint_num_ = 0;
      sub_phase_ = SUB_PHASE1;
      force_closure_ = false;
      envelope_closure_ = false;
      one_time_tighten_ = false;
      modification_start_time_ = ros::Time::now();


      TODO form grasp_form_search
        /****************************************************************/
        /* test the control approach part */
        /* claculate the joint angles for grasp from graspl_planner */
        if(control_test_flag_)
          {
            std::vector<float> v_hold_angle(contact_num_ - 1);
            std::vector<float> v_tighten_angle(contact_num_ - 1);
            std::vector<float> v_approach_angle(contact_num_ - 1);

            float approach_delta_angle = -0.4;//-0.4; // about 30deg
            float tighten_delta_angle =  0.09; // about 5deg

            getObjectGraspAngles(tighten_delta_angle, approach_delta_angle, contact_num_, v_hold_angle, v_tighten_angle, v_approach_angle);

            for(int i = 0; i < contact_num_ -1; i++)
              {
                ROS_INFO("grasp planner&control test: joint%d, approach_angle: %f, hold_angle: %f, tighten_angle: %f", i+1, v_approach_angle[i], v_hold_angle[i], v_tighten_angle[i]);
                v_best_theta_[i](0) = v_approach_angle[i];
              }

            /* calculate object_approach_offset[x,y,yaw] */
            object_approach_offset_x_ = 0;
            object_approach_offset_y_ = 0;
            object_approach_offset_yaw_ = 0;
            getObjectApproachOffset(v_approach_angle, object_approach_offset_x_, object_approach_offset_y_, object_approach_offset_yaw_);
            ROS_INFO("grasp planner&control test: base link: %d, object_approach_offset_x_: %f, object_approach_offset_y_: %f, object_approach_offset_yaw_: %f", approach_base_link_, object_approach_offset_x_, object_approach_offset_y_, object_approach_offset_yaw_);

          }
      /****************************************************************/

      /* the grasp approaching control variables */
      nhp_.param("approach_base_link", approach_base_link_, 0);
      nhp_.param("approach_pos_weight_rate", approach_pos_weight_rate_, 2.0);
      nhp_.param("approach_angle_weight_rate", approach_angle_weight_rate_, 2.0);
      std::cout << "approach_base_link: " << approach_base_link_ << ", "
                << "approach_pos_weight_rate: " << approach_pos_weight_rate_ << ", "
                << "approach_angle_weight_rate: " << approach_angle_weight_rate_
                << std::endl;
    }
  }

  void Hydrus::graspPhase()
  {
    static bool once_flag = true;
    static int cnt = 0;

    if(joint_num_ == 0) return;

    if(sub_phase_ == SUB_PHASE1)
      {// change pose
        if(once_flag)
          {
            once_flag = false;
            /* send joint angles */
            sensor_msgs::JointState joint_ctrl_msg;
            joint_ctrl_msg.header.stamp = ros::Time::now();
            for(int i = 0; i < joint_num_; i++)
              joint_ctrl_msg.position.push_back(joints_control_[i].approach_angle);
            joint_ctrl_pub_.publish(joint_ctrl_msg);

            /* send the overload check to inactive */
            ros::ServiceClient overload_check_activate_client = nh_.serviceClient<std_srvs::SetBool>(overload_check_activate_srv_name_);
            std_srvs::SetBool srv;
            srv.request.data = false;

            if (overload_check_activate_client.call(srv))
              ROS_INFO("set the overload check to be inactive");
            else
              ROS_ERROR("Failed to call service %s", overload_check_activate_srv_name_.c_str());
          }

        /* phase shift condition */
        bool pose_fixed_flag_ = true;
        /* specified for dynamixel motor */
        for(int i = 0; i < joint_num_; i++)
          if(joints_control_[i].moving) pose_fixed_flag_ = false;

        if(pose_fixed_flag_)
          {
            if(++cnt > (pose_fixed_count_ * func_loop_rate_))
              {
                ROS_INFO("Grasping Sub Phase: Succeed to change to sub phase 2");
                sub_phase_ ++;
                cnt = 0; // convergence reset
                once_flag = true;

                /* send nav msg */
                aerial_robot_base::FlightNav nav_msg;
                nav_msg.header.stamp = ros::Time::now();
                nav_msg.pos_xy_nav_mode = aerial_robot_base::FlightNav::POS_MODE;
                nav_msg.target_pos_x = object_position_.x + object_offset_.x();
                nav_msg.target_pos_y = object_position_.y + object_offset_.y();
                nav_msg.pos_z_nav_mode = aerial_robot_base::FlightNav::POS_MODE;
                nav_msg.target_pos_z = object_height_;
                nav_msg.psi_nav_mode = aerial_robot_base::FlightNav::POS_MODE;
                nav_msg.target_psi = object_position_.theta + object_offset_.z();
                if(nav_msg.target_psi > M_PI) nav_msg.target_psi -= (2 * M_PI);
                if(nav_msg.target_psi < -M_PI) nav_msg.target_psi += (2 * M_PI);
                uav_nav_pub_.publish(nav_msg);
              }
          }
      }
    else if(sub_phase_ == SUB_PHASE2)
      {// descending
        if(fabs(object_height_ - uav_position_.z()) < grasping_height_threshold_)
          {
            ROS_INFO("Grasping Sub Phase: Succeed to change to sub phase 3");
            sub_phase_++;

            /* set init target angle */
            for(int i = 0; i < joint_num_; i++)
              joints_control_[i].target_angle = joints_control_[i].approach_angle;

          }
      }
    else if(sub_phase_ == SUB_PHASE3)
      {// grasping, most important

        /* procedure: !envelope_closure_ -> envelope_closure_ -> force_closure_ */
        if(envelope_closure_)
          {/* if we reach envelop status, we move directly to tigthen angles, no feed back from contact point */

            //4.1 envelope closure -> force closure iteration
            /* one_time tighten, not iteration*/
            if(!one_time_tighten_)
              {
                one_time_tighten_ = true;
                for(int i = 0; i < joint_num_; i++)
                  joints_control_[i].target_angle = joints_control_[i].tighten_angle;
                ROS_WARN("force-closure: once tighten process");
              }
          }
        else
          {
            /* calculate the joint command */
            for(int i = 0; i < joint_num_; i++)
              {
                if(joints_control_[i].target_angle != joints_control_[i].hold_angle)
                  {
                    float incre_angle = (joints_control_[i].hold_angle - joints_control_[i].approach_angle) / func_loop_rate_;
                    float rate = 1.0 / (grasping_rate_ * (float)abs(i - joint_num_/2) + 1);
                    joints_control_[i].target_angle += (incre_angle * rate);
                  }
                //limitation
                if(joints_control_[i].target_angle > joints_control_[i].hold_angle
                   && joints_control_[i].holding_rotation_direction == 1)
                  joints_control_[i].target_angle = joints_control_[i].hold_angle;
                if(joints_control_[i].target_angle < joints_control_[i].hold_angle
                   && joints_control_[i].holding_rotation_direction == -1)
                  joints_control_[i].target_angle = joints_control_[i].hold_angle;
              }
          }

        /* send joint angles */
        sensor_msgs::JointState joint_ctrl_msg;
        joint_ctrl_msg.header.stamp = ros::Time::now();
        for(int i = 0; i < joint_num_; i++)
          joint_ctrl_msg.position.push_back(joints_control_[i].target_angle);
        joint_ctrl_pub_.publish(joint_ctrl_msg);
      }
  }

  void Hydrus::dropPhase()
  {
    static bool once_flag = true;
    static int cnt = 0;

    if(joint_num_ == 0) return;

    if(once_flag)
      {
        once_flag = false;
        /* send joint angles */
        sensor_msgs::JointState joint_ctrl_msg;
        joint_ctrl_msg.header.stamp = ros::Time::now();
        for(int i = 0; i < joint_num_; i++)
          joint_ctrl_msg.position.push_back(joints_control_[i].approach_angle);
        joint_ctrl_pub_.publish(joint_ctrl_msg);
      }

    /* phase shift condition */
    bool pose_fixed_flag_ = true;
    /* specified for dynamixel motor */
    for(int i = 0; i < joint_num_; i++)
      if(joints_control_[i].moving) pose_fixed_flag_ = false;

    if(pose_fixed_flag_)
      {
        if(++cnt > (pose_fixed_count_ * func_loop_rate_))
          {
            ROS_WARN("Object dropped!! Shift to RETURN_PHASE");
            phase_ = RETURN_PHASE;
            cnt = 0; // convergence reset
            once_flag = true;

            /* reset for base variables */
            sub_phase_ = SUB_PHASE1;
            force_closure_ = false;
            modification_start_time_ = ros::Time::now();
            envelope_closure_ = false;
            one_time_tighten_ = false;

            /* send the overload check to inactive */
#if 0
            ros::ServiceClient overload_check_activate_client = nh_.serviceClient<std_srvs::SetBool>(overload_check_activate_srv_name_);
            std_srvs::SetBool srv;
            srv.request.data = true;

            if (overload_check_activate_client.call(srv))
              ROS_INFO("set the overload check to be active");
            else
              ROS_ERROR("Failed to call service %s", overload_check_activate_srv_name_.c_str());
#endif
          }
      }
  }

  void Hydrus::objectPoseApproachOffsetCal()
  {
    object_offset_.setValue(object_approach_offset_x_ * cos(object_position_.theta)
                            -object_approach_offset_y_ * sin(object_position_.theta),
                            object_approach_offset_x_ *sin(object_position_.theta)
                            +object_approach_offset_y_ *cos(object_position_.theta),
                            object_approach_offset_yaw_);
  }

  void Hydrus::jointStatesCallback(const sensor_msgs::JointStateConstPtr& joint_states_msg)
  {
    if(joint_num_ == 0)
      {
        joint_num_ = joint_states_msg->position.size();
        jointControlParamInit();
      }

    float max_delta_angle = 1e-6;

    for(int i = 0; i < joint_num_; i++)
      {
        joints_control_[i].current_angle = joint_states_msg->position[i];
        float delta_angle = fabs(joints_control_[i].current_angle - joints_control_[i].hold_angle);
        if(max_delta_angle < delta_angle) max_delta_angle = delta_angle;
      }

    /* 3.1 check the enveloping grasp */
    if(phase_ == GRASPING_PHASE && sub_phase_ == SUB_PHASE3 &&
       !envelope_closure_ && !control_cheat_mode_)
      {
        // for(int i = 0; i < joint_num_; i++)
        //   ROS_WARN(" joints_control_[%d].current_angle:%f, joints_control_[%d].hold_angle: %f", i, joints_control_[i].current_angle, i, joints_control_[i].hold_angle);

        // ROS_WARN("the max_dlta_angle is %f, envelope_joint_angle_thre_ is %f", max_delta_angle, envelope_joint_angle_thre_);
        /* check the enveloping grasp in terms of joint angles */
        if(max_delta_angle < envelope_joint_angle_thre_)
          {
            //test, keep position control until the envelope closure reaches

            /* send nav msg: shift to vel control mode */
            aerial_robot_base::FlightNav nav_msg;
            nav_msg.header.stamp = ros::Time::now();
            nav_msg.pos_xy_nav_mode = aerial_robot_base::FlightNav::VEL_MODE;
            nav_msg.target_pos_x = 0;
            nav_msg.target_pos_y = 0;
            nav_msg.pos_z_nav_mode = aerial_robot_base::FlightNav::NO_NAVIGATION;
            nav_msg.psi_nav_mode = aerial_robot_base::FlightNav::NO_NAVIGATION;

            uav_nav_pub_.publish(nav_msg);

            if(!control_cheat_mode_)
              {
                std_msgs::UInt8 low_flight_velocity_control;
                low_flight_velocity_control.data= 1;
                aerial_grasping_flight_velocity_control_pub_.publish(low_flight_velocity_control);
              }

            envelope_closure_ = true;
            ROS_WARN("GRASPING_PHASE, shift to tighten angles because reaching to the hold angles");
          }
      }
  }

  void Hydrus::jointMotorStatusCallback(const dynamixel_msgs::MotorStateListConstPtr& joint_motors_msg)
  {
    bool force_closure = true;
    bool overload_flag = false; /* new, 2017 IJRR */

    if(joint_num_ == 0)
      {
        joint_num_ = joint_motors_msg->motor_states.size();
        jointControlParamInit();
      }

    for(int i = 0; i < joint_num_; i++)
      {
        joints_control_[i].moving = joint_motors_msg->motor_states[i].moving;
        joints_control_[i].load_rate = joint_motors_msg->motor_states[i].load;
        joints_control_[i].temperature = joint_motors_msg->motor_states[i].temperature;
        joints_control_[i].angle_error = joint_motors_msg->motor_states[i].error;

        /* if all torque load is bigger than the certain threshold, then the force closure is achieved */
        if(joints_control_[i].load_rate < torque_min_threshold_) force_closure = false;

        // 1. the torque exceeds the max threshold(overload)
        // 2. in the holding phase(force_closure_ == true)
        // 3. the modification is not susseccive
        /* CAUTION: only all the joints have the load(force_closure!!!), we recognize the overload flag, otherwise, we ignore the loadover. this is very dangerous!!!!!! */
        /* do not modified once grasped the object */
        if(joints_control_[i].load_rate > torque_max_threshold_ /* (joints_control_[i].angle_error & OVERLOAD_FLAG) */
           && force_closure
           && phase_ == GRASPING_PHASE
           && sub_phase_ == SUB_PHASE3
           && (ros::Time::now().toSec() - modification_start_time_.toSec() > modification_duration_))
          {
            ROS_WARN("jointMotorStatusCallback: overload in joint%d: %f; error code: %d", i + 1, joints_control_[i].load_rate, joints_control_[i].angle_error);
            overload_flag = true;

            if(control_cheat_mode_)
              {
                joints_control_[i].target_angle -= (joints_control_[i].holding_rotation_direction * modification_delta_angle_);
                joints_control_[i].hold_angle -= (joints_control_[i].holding_rotation_direction * modification_delta_angle_);
              }
          }
      }

    /* 3.2 check the enveloping grasp in terms of joint torques */
    /* TODO: the torque to control the angle change already reach the condition */
#if 0
    if(force_closure &&
       phase_ == GRASPING_PHASE &&
       !envelope_closure_ &&
       !control_cheat_mode_)
      {
        // shift to tighten angles
        envelope_closure_ = true;
        ROS_WARN("jointMotorStatusCallback: GRASPING_PHASE, shift to tighten angles because all joint torques satisfy the threshold force closure condition");
      }
#endif

    /* process while overlaod_flag is true */
    if(overload_flag)
      {
        ROS_WARN("Reset holding start time");
        holding_start_time_ = ros::Time::now(); //reset!!
        modification_start_time_ = ros::Time::now(); //reset!!

        /* release the joint angles */
        //4.2 modification in force-closure phase
        if(!control_cheat_mode_)
          {
            for(int i = 0; i < joint_num_; i++)
              joints_control_[i].target_angle += ((joints_control_[i].tighten_angle - joints_control_[i].hold_angle) / tighten_delta_angle_ * release_delta_angle_);
          }

        /* send modified angle */
        sensor_msgs::JointState joint_ctrl_msg;
        joint_ctrl_msg.header.stamp = ros::Time::now();
        for(int i = 0; i < joint_num_; i++)
          joint_ctrl_msg.position.push_back(joints_control_[i].target_angle);
        joint_ctrl_pub_.publish(joint_ctrl_msg);
      }

    /* the condition to get into hold phase */
    if(!force_closure_ && force_closure)
      {
        ROS_INFO("phase: %d, sub_phase: %d", phase_, sub_phase_);
        if(phase_ == GRASPING_PHASE && sub_phase_ == SUB_PHASE3  && envelope_closure_)
          {
            force_closure_ = true;
            ROS_WARN("GRASPING_PHASE: Force_Closure with object");
            holding_start_time_ = ros::Time::now(); //reset!!
          }
      }

    /* the condition to shift to transportation phase */
    if(force_closure_ && ros::Time::now().toSec() - holding_start_time_.toSec() > hold_count_)
      {
        if(phase_ == GRASPING_PHASE)
          {
            ROS_WARN("Pick the object up!! Shift to GRSPED_PHASE");
            phase_ = GRASPED_PHASE;
            sub_phase_ = SUB_PHASE1;

            if(!control_cheat_mode_)
              {
                std_msgs::UInt8 low_flight_velocity_control;
                low_flight_velocity_control.data= 0;
                aerial_grasping_flight_velocity_control_pub_.publish(low_flight_velocity_control);
              }
          }
      }
  }

  void Hydrus::joyStickAdditionalCallback(const sensor_msgs::JoyConstPtr & joy_msg)
  {
    if(joy_debug_)
      {
        if(joy_msg->buttons[9] == 1) // RIGHT TOP TRIGGER(R1)
          {
          }
        if(joy_msg->buttons[11] == 1) // RIGHT DOWM TRIGGER(R1)
          {

            ROS_WARN("Debug: Shift to GRASPING_PHASE & Sub_Phase3");
            phase_ = GRASPING_PHASE;
            sub_phase_ = SUB_PHASE3;

            /* set init target angle */
            for(int i = 0; i < joint_num_; i++)
              joints_control_[i].target_angle = joints_control_[i].approach_angle;

            /* send nav msg: shift to vel control mode */
            aerial_robot_base::FlightNav nav_msg;
            nav_msg.header.stamp = ros::Time::now();
            nav_msg.pos_xy_nav_mode = aerial_robot_base::FlightNav::VEL_MODE;
            nav_msg.target_pos_x = 0;
            nav_msg.target_pos_y = 0;
            nav_msg.pos_z_nav_mode = aerial_robot_base::FlightNav::NO_NAVIGATION;
            nav_msg.psi_nav_mode = aerial_robot_base::FlightNav::NO_NAVIGATION;
            uav_nav_pub_.publish(nav_msg);
          }
      }
  }

  void Hydrus::rosParamInit()
  {
    std::string ns = nhp_.getNamespace();
    nhp_.param("grasp_planning_plugin_name", grasp_planning_plugin_name_, std::string("grasp_planning/full_search"));
    nhp_.param("joint_ctrl_pub_name", joint_ctrl_pub_name_, std::string("/hydrus/joints_ctrl"));
    nhp_.param("aerial_grasping_flight_velocity_control_pub_name", aerial_grasping_flight_velocity_control_pub_name_, std::string("velocity_control_gain_change"));
    nhp_.param("joint_states_sub_name", joint_states_sub_name_, std::string("/joint_states"));
    nhp_.param("joint_motors_sub_name", joint_motors_sub_name_, std::string("/joint_motors"));

    if(control_cheat_mode_)
      {
        nhp_.param("object_approach_offset_x", object_approach_offset_x_, 0.0);
        nhp_.param("object_approach_offset_y", object_approach_offset_y_, 0.25);
        nhp_.param("object_approach_offset_yaw", object_approach_offset_yaw_, 0.0);
      }

    nhp_.param("pose_fixed_count", pose_fixed_count_, 2.0); //sec
    nhp_.param("grasping_height_threshold", grasping_height_threshold_, 0.01); //m

    nhp_.param("grasping_duration", grasping_duration_, 2.0); 
    nhp_.param("torque_min_threshold", torque_min_threshold_, 0.2); //20%
    nhp_.param("torque_max_threshold", torque_max_threshold_, 0.5); //50%
    nhp_.param("modification_delta_angle", modification_delta_angle_, 0.015);
    nhp_.param("modification_duration", modification_duration_, 0.5);
    nhp_.param("hold_count", hold_count_, 1.0); 
    nhp_.param("grasping_rate", grasping_rate_, 1.0); 

    /* grasp planning  */
    nhp_.param("envelope_joint_angle_thre", envelope_joint_angle_thre_, 0.1); //[rad]
    nhp_.param("approach_delta_angle", approach_delta_angle_, -0.45); // about 25deg
    nhp_.param("tighten_delta_angle", tighten_delta_angle_, 0.09); // about 5deg
    nhp_.param("release_delta_angle", release_delta_angle_, -0.01); // about 0.5deg

    nhp_.param("overload_check_activate_srv_name", overload_check_activate_srv_name_, std::string("overload_check_activate"));
  }

  void  Hydrus::jointControlParamInit()
  {
    joints_control_.resize(joint_num_);

    if(control_cheat_mode_)
      {
        for(int i = 0; i < joint_num_; i++)
          {
            std::stringstream joint_no;
            joint_no << i + 1;

            nhp_.param(std::string("j") + joint_no.str() + std::string("/approach_angle"), joints_control_[i].approach_angle, 1.2);
            nhp_.param(std::string("j") + joint_no.str() + std::string("/hold_angle"), joints_control_[i].hold_angle, 1.5);
            nhp_.param(std::string("j") + joint_no.str() + std::string("/tighten_angle"), joints_control_[i].tighten_angle, 0.0);

            joints_control_[i].holding_rotation_direction = (joints_control_[i].hold_angle - joints_control_[i].approach_angle) / fabs(joints_control_[i].hold_angle - joints_control_[i].approach_angle);

            ROS_INFO("control cheat mode: joint%d, approach_angle: %f, hold_angle: %f, tighten_angle: %f, holding_rotation_direction: %d", i+1, joints_control_[i].approach_angle, joints_control_[i].hold_angle, joints_control_[i].tighten_angle, joints_control_[i].holding_rotation_direction);
          }
      }
    else
      {
        int contact_num;
        std::vector<float> v_hold_angle(joint_num_);
        std::vector<float> v_tighten_angle(joint_num_);
        std::vector<float> v_approach_angle(joint_num_);

        //1. hold_angles, tighten_angles, approach_angles
        /* claculate the joint angles for grasp from graspl_planner */
        grasp_planning_method_->getObjectGraspAngles(tighten_delta_angle_, approach_delta_angle_, contact_num, v_hold_angle, v_tighten_angle, v_approach_angle);

        int holding_rotation_direction = -approach_delta_angle_ / fabs(approach_delta_angle_);

        for(int i = 0; i < joint_num_; i++)
          {
            joints_control_[i].approach_angle = v_approach_angle[i];
            joints_control_[i].tighten_angle = v_tighten_angle[i];
            joints_control_[i].hold_angle = v_hold_angle[i];
            joints_control_[i].holding_rotation_direction = holding_rotation_direction;
            ROS_INFO("grasp planner: joint%d, approach_angle: %f, hold_angle: %f, tighten_angle: %f, holding_rotation_direction: %d", i+1, joints_control_[i].approach_angle, joints_control_[i].hold_angle, joints_control_[i].tighten_angle, joints_control_[i].holding_rotation_direction);
          }

        /* rewrite joint_num to contact_num -1, redundant joint is not used in this grasp control  */
        joint_num_ = contact_num -1 ;
        ROS_INFO("grasp planner:  change joint_num_ to contact_num -1 :%d", joint_num_);

        /* calculate object_approach_offset[x,y,yaw] */
        std::vector<float> approach_angles = getApproachAngles();
        double object_approach_offset_x = 0;
        double object_approach_offset_y = 0;
        double object_approach_offset_yaw = 0;
        grasp_planning_method_->getObjectApproachOffset(approach_angles, object_approach_offset_x, object_approach_offset_y, object_approach_offset_yaw);
        object_approach_offset_x_ = object_approach_offset_x;
        object_approach_offset_y_ = object_approach_offset_y;
        object_approach_offset_yaw_ = object_approach_offset_yaw;
      }
  }


    void Base::getObjectGraspAngles(float tighten_delta_angle, float approach_delta_angle, int& contact_num, std::vector<float>& v_hold_angle, std::vector<float>& v_tighten_angle, std::vector<float>& v_approach_angle)
  {
    /* tighten delta angles based on the tau */
    float max_element = v_best_tau_.maxCoeff();
    contact_num = contact_num_;

    ROS_WARN("max_element: %f", max_element);

    for(int i = 0; i < contact_num_ - 1; i ++)
      {
        /* weight calc */
        int index = (i - (contact_num_ -1) / 2 >= 0)? i - (contact_num_ -1)/2:(contact_num_ /2-1) - i;
        /* temporary for the rate */
        float approach_delta_angle_i = (index /  approach_angle_weight_rate_ + 1 ) * approach_delta_angle;

        /* hold angle */
        v_hold_angle[i] = v_best_theta_[i](0);

        /* approach angle */
        v_approach_angle[i] = v_hold_angle[i] + approach_delta_angle_i;

        /* tighten angle */
        //v_tighten_angle[i] = v_hold_angle[i] + v_best_tau_(i,0) / max_element * tighten_delta_angle;
        v_tighten_angle[i] = v_hold_angle[i] + tighten_delta_angle;
      }
  }

  void Base::getObjectApproachOffset(std::vector<float> v_theta, double& object_approach_offset_x, double& object_approach_offset_y, double& object_approach_offset_yaw)
  {
    /* check the health */
    if(v_theta.size() < contact_num_ - 1)
      {
        ROS_FATAL("getObjectApproachOffest: the joint number from v_theta %d is smaller than contact_num - 1: %d", (int)v_theta.size(), contact_num_ -1);
        return;
      }

    /* change the approach theta to abs theta */
    std::vector<float> v_abs_theta(contact_num_);
    float abs_theta = 0;
    for(int i = 0; i < contact_num_; i ++)
      {
        v_abs_theta[i] = abs_theta;
        abs_theta += v_theta[i];
      }

    /* the direction of best_link in terms of best base side */
    float base_link_best_base_side_direction = v_best_phy_[approach_base_link_] + v_psi_[approach_base_link_](1);
    ROS_INFO("base_link_best_base_side_direction: %f", base_link_best_base_side_direction);

    /* 1. x and y */
    /* 1.1 calculate o_b: the best approach position of origin of base link in terms of  best_base_side_ frame */
    float E_beta1 = 0, E_beta2 = 0, E_beta1_beta1 = 0, E_beta2_beta2 = 0, E_beta1_beta2 = 0, E_beta1_beta1_beta1 = 0, E_beta2_beta2_beta2 = 0, E_beta1_beta2_beta2 = 0, E_beta1_beta1_beta2 = 0;
    Vector3d grasping_cog = Vector3d(0, 0, 0); // the cog of hydrus when totally grasp
    float weight_rate_sum = (pow(approach_pos_weight_rate_, contact_num_/2) - 1) / (approach_pos_weight_rate_ -1);
    ROS_WARN("weight_rate_sum: %f", weight_rate_sum);

    for(int i = 0; i < contact_num_; i++)
      {
        ROS_INFO("contact%d", i);

        Vector3d v_base_link = Vector3d(0, 0, 0);
        Vector3d v_intermediate_link = Vector3d(0, 0, 0);
        Vector3d v_end_link = Vector3d(0, 0, 0);
        Vector3d v_propeller = Vector3d(0, 0, 0);

        int index = (i - contact_num_ /2 >= 0)? i - contact_num_ /2 + 1:contact_num_ /2 - i;
        float weight_rate = pow(approach_pos_weight_rate_, index -1) / weight_rate_sum * 0.5;
        ROS_WARN("weight_rate: %f", weight_rate);

        if(i < approach_base_link_)
          {
            v_base_link = Vector3d(0, 0, 0);
            v_end_link = AngleAxisd(M_PI + v_abs_theta[i] - v_abs_theta[approach_base_link_] + v_best_theta_[approach_base_link_](1), Vector3d::UnitZ()) * Vector3d(link_length_ /2, 0, 0);
            for(int j = i + 1; j < approach_base_link_; j ++)
              {
                Vector3d  v_intermediate_link_tmp = v_intermediate_link;
                v_intermediate_link = v_intermediate_link_tmp +
                  AngleAxisd(M_PI + v_abs_theta[j] - v_abs_theta[approach_base_link_] + v_best_theta_[approach_base_link_](1), Vector3d::UnitZ()) * Vector3d(link_length_, 0, 0);
                if(debug_)
                  {
                    ROS_INFO("getObjectApproachOffest: j:%d, base_link:%d", j, approach_base_link_);
                    std::cout << "v_intermediate_link: " << v_intermediate_link.transpose() << std::endl;
                  }
              }
          }
        else if(i > approach_base_link_)
          {
            v_base_link = AngleAxisd(v_best_theta_[approach_base_link_](1), Vector3d::UnitZ()) * Vector3d(link_length_, 0, 0);
            v_end_link = AngleAxisd(v_abs_theta[i] - v_abs_theta[approach_base_link_] + v_best_theta_[approach_base_link_](1), Vector3d::UnitZ()) * Vector3d(link_length_ /2, 0, 0);
            for(int j = approach_base_link_ + 1; j < i; j ++)
              {
                Vector3d  v_intermediate_link_tmp = v_intermediate_link;
                v_intermediate_link = v_intermediate_link_tmp +
                  AngleAxisd(v_abs_theta[j] - v_abs_theta[approach_base_link_] + v_best_theta_[approach_base_link_](1), Vector3d::UnitZ()) * Vector3d(link_length_, 0, 0);
                if(debug_)
                  {
                    ROS_INFO("getObjectApproachOffest: j:%d, base_link:%d", j, approach_base_link_);
                    std::cout << "v_intermediate_link: " << v_intermediate_link.transpose() << std::endl;
                  }
              }
          }

        v_propeller = AngleAxisd(M_PI/2 + v_psi_[i](1), Vector3d::UnitZ()) * Vector3d(link_radius_, 0, 0);

        /* calculate the cog of hydrus in totally grasp state */
        Vector3d link_p = v_best_contact_p_[i] - v_propeller;
        Vector3d grasping_cog_tmp = grasping_cog;
        grasping_cog = grasping_cog_tmp + link_p;

        //Vector3d beta = v_base_link + v_intermediate_link + v_end_link + v_propeller - v_best_contact_p_[i];
        Vector3d beta = v_base_link + v_intermediate_link + v_end_link;
        std::cout << "v_base_link: " << v_base_link.transpose() << std::endl;
        std::cout << "v_end_link: " << v_end_link.transpose() << std::endl;
        std::cout << "v_intermediate_link: " << v_intermediate_link.transpose() << std::endl;
        std::cout << "v_propeller: " << v_propeller.transpose() << std::endl;
        std::cout << "v_best_contact_p_[" <<i << "]: " << v_best_contact_p_[i].transpose() << std::endl;
        std::cout << "link_p: " << link_p.transpose() << std::endl;
        std::cout << "beta: " << beta.transpose() << std::endl;

        E_beta1 += (beta(0) * weight_rate);
        E_beta2 += (beta(1) * weight_rate);

      }

    grasping_cog /= contact_num_;

    Vector2d O_b_best_base_side;
    O_b_best_base_side << grasping_cog.x() - E_beta1, grasping_cog.y() - E_beta2;
    std::cout << "grasping_cog: " << grasping_cog.transpose() << std::endl;

    ROS_WARN("O_b_best_base_side:");
    std::cout << O_b_best_base_side.transpose() << std::endl;

    /* 1.2 change from best_base_side-x-axis/vertex-origin frame to first_side-x-axis/cog-origin frame */
    Vector3d O_b_first_link_cog;
    if(object_type_ == CONVEX_POLYGONAL_COLUMN)
      O_b_first_link_cog = AngleAxisd(v_orig_psi_[best_base_side_](1), Vector3d::UnitZ())
        * (Vector3d(O_b_best_base_side(0),
                    O_b_best_base_side(1),
                    0) - cog_object_ );
    else if(object_type_ == CYLINDER) /* no best_base_side */
      O_b_first_link_cog = Vector3d(O_b_best_base_side(0),
                                    O_b_best_base_side(1),
                                    0);
    object_approach_offset_x = O_b_first_link_cog(0);
    object_approach_offset_y = O_b_first_link_cog(1);

    ROS_WARN("O_b_first_link_cog:");
    std::cout << O_b_first_link_cog.transpose() << std::endl;

    /* 2. calculate phy_b: the best approach direction of base link */
    if(object_type_ == CONVEX_POLYGONAL_COLUMN)
      object_approach_offset_yaw =  v_best_theta_[approach_base_link_](1) + v_orig_psi_[best_base_side_](1);
    else if(object_type_ == CYLINDER) /* no best_base_side */
      object_approach_offset_yaw = v_best_theta_[approach_base_link_](1);
  }
};

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_transportation::Hydrus, aerial_transportation::Base);
