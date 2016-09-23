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
    joint_motors_sub_ = nh_.subscribe<dynamixel_msgs::MotorStateList>(joint_motors_sub_name_, 1, &Hydrus::jointMotorStatusCallback, this); //do not use udp option !!!
    joint_states_sub_ = nh_.subscribe<sensor_msgs::JointState>(joint_states_sub_name_, 1, &Hydrus::jointStatesCallback, this); //do not use udp option !!!

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

    /* base variables init */
    joint_num_ = 0;
    sub_phase_ = SUB_PHASE1;
    force_closure_ = false;
    envelope_closure_ = false;
    one_time_tighten_ = false;
    modification_start_time_ = ros::Time::now();
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
        if(fabs(object_height_ - uav_position_.position.z) < grasping_height_threshold_)
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

            //4.1 DONE: envelope closure -> force closure iteration
            /* TODO: one_time tighten, not iteration*/
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
                    float rate = 1.0 / (grasping_rate_ * (float)(abs(i - joint_num_/2) + 1));
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

            //reset for base variables
            sub_phase_ = SUB_PHASE1;
            force_closure_ = false;
            modification_start_time_ = ros::Time::now();
            envelope_closure_ = false;
            one_time_tighten_ = false;
          }
      }
  }

  void Hydrus::objectPoseApproachOffsetCal() 
  {
    object_offset_.setValue(object_approach_offset_x_ *cos(object_position_.theta)
                            -object_approach_offset_y_ *sin(object_position_.theta),
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

    /* 3.1 DONE: check the enveloping grasp */
    if(phase_ == GRASPING_PHASE && sub_phase_ == SUB_PHASE3 &&
       !envelope_closure_ && !control_cheat_mode_)
      {
        // for(int i = 0; i < joint_num_; i++)
        //   ROS_WARN(" joints_control_[%d].current_angle:%f, joints_control_[%d].hold_angle: %f", i, joints_control_[i].current_angle, i, joints_control_[i].hold_angle);

        // ROS_WARN("the max_dlta_angle is %f, envelope_joint_angle_thre_ is %f", max_delta_angle, envelope_joint_angle_thre_);
        /* check the enveloping grasp in terms of joint angles */
        if(max_delta_angle < envelope_joint_angle_thre_)
          {
            //test, keep position control until the envelope closure reachs

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
    bool torque_load_exceed_flag = false;

    if(joint_num_ == 0)
      {
        joint_num_ = joint_motors_msg->motor_states.size();
        jointControlParamInit();
      }

    for(int i = 0; i < joint_num_; i++)
      {
        joints_control_[i].moving = joint_motors_msg->motor_states[i].moving;  
        joints_control_[i].load_rate = joint_motors_msg->motor_states[i].load;
        //ROS_INFO("DEBUG: joint%d, torque:%f",i+1,joints_control_[i].load_rate);
        joints_control_[i].temperature = joint_motors_msg->motor_states[i].temperature;
        joints_control_[i].angle_error = joint_motors_msg->motor_states[i].error;

        /* if all torque load is bigger than the certain threshold, then the force closure is achieved */
        if(joints_control_[i].load_rate < torque_min_threshold_) force_closure = false;

        // 1. the torque exceeds the max threshold
        // 2. in the holding phase(force_closure_ == true)
        // 3. the modification is not susseccive
        /* CAUTION: only all the joints have the load(force_closure!!!), we recognize the loadover flag, otherwise, we ignore the loadover. this is very dangerous!!!!!! */
        /* do not modified once grasped the object */
        if(joints_control_[i].load_rate > torque_max_threshold_ && force_closure
           && phase_ == GRASPING_PHASE
           && ros::Time::now().toSec() - modification_start_time_.toSec() > modification_duration_)
          {
            ROS_WARN("jointMotorStatusCallback: Holding load exceeds in joint%d: %f", i + 1, joints_control_[i].load_rate);
            torque_load_exceed_flag = true;

            if(control_cheat_mode_)
              {
                joints_control_[i].target_angle -= (joints_control_[i].holding_rotation_direction * modification_delta_angle_);
                joints_control_[i].hold_angle -= (joints_control_[i].holding_rotation_direction * modification_delta_angle_);
              }
          }
      }

    /* 3.2 DONE: check the enveloping grasp in terms of joint torques */
    if(force_closure && phase_ == GRASPING_PHASE && !envelope_closure_ && !control_cheat_mode_)
      {
        // shift to tighten angles
        envelope_closure_ = true;
        ROS_WARN("jointMotorStatusCallback: GRASPING_PHASE, shift to tighten angles because the all joits torque satisfy the threshold condition");
      }

    /* process while torque_load_exceed_flag occurs */
    if(torque_load_exceed_flag)
      {
        ROS_WARN("Reset holding start time");
        holding_start_time_ = ros::Time::now(); //reset!!
        modification_start_time_ = ros::Time::now(); //reset!!

        /* release the joint angles */
        //4.2 TODO: modification in force-closure phase
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
        if(phase_ == GRASPING_PHASE && sub_phase_ == SUB_PHASE3 )
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
    nhp_.param("torque_max_threshold", torque_max_threshold_, 0.5); //20%
    nhp_.param("modification_delta_angle", modification_delta_angle_, 0.015);
    nhp_.param("modification_duration", modification_duration_, 0.5);
    nhp_.param("hold_count", hold_count_, 1.0); 
    nhp_.param("grasping_rate", grasping_rate_, 1.0); 

    /* grasp planning  */
    nhp_.param("base_link", base_link_, 1); // second link
    nhp_.param("envelope_joint_angle_thre", envelope_joint_angle_thre_, 0.1); //[rad]
    nhp_.param("approach_delta_angle", approach_delta_angle_, -0.45); // about 25deg
    nhp_.param("tighten_delta_angle", tighten_delta_angle_, 0.09); // about 5deg
    nhp_.param("release_delta_angle", release_delta_angle_, -0.01); // about 0.5deg
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

        //2.DONE hold_angles, tighten_angles, approach_angles
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
        //1.DONE: apporach offset [x,y,yaw]
        std::vector<float> approach_angles = getApproachAngles();
        double object_approach_offset_x = 0;
        double object_approach_offset_y = 0;
        double object_approach_offset_yaw = 0;
        grasp_planning_method_->getObjectApproachOffest(base_link_, approach_angles, object_approach_offset_x, object_approach_offset_y, object_approach_offset_yaw);
        object_approach_offset_x_ = object_approach_offset_x;
        object_approach_offset_y_ = object_approach_offset_y;
        object_approach_offset_yaw_ = object_approach_offset_yaw;
        /* reverse the object_approach_offset_yaw_, because the direction of base_link and mocap&imu frame is opposite */
        object_approach_offset_yaw_ += M_PI;
      }
  }

};

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_transportation::Hydrus, aerial_transportation::Base);
