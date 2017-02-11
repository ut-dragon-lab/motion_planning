#include <aerial_transportation/grasp_planning/grasp_planner.h>

using namespace Eigen;
USING_NAMESPACE_QPOASES

namespace grasp_planning
{
  void Base::baseInit(ros::NodeHandle nh, ros::NodeHandle nhp)
  {
    nh_ = nh;
    nhp_ = nhp;

    baseRosParamInit();

    /* pub & sub */
    if(play_file_flag_)
      {
        getResultFromFile();

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
            object_approach_offset_yaw_ += M_PI;
            ROS_INFO("grasp planner&control test: base link: %d, object_approach_offset_x_: %f, object_approach_offset_y_: %f, object_approach_offset_yaw_: %f", approach_base_link_, object_approach_offset_x_, object_approach_offset_y_, object_approach_offset_yaw_);

          }
        /****************************************************************/
      }
    else
      {
        convex_polygonal_column_info_sub_ = nh_.subscribe<geometry_msgs::PolygonStamped>(convex_polygonal_column_info_sub_name_, 1, &Base::convexPolygonalColumnInfoCallback, this);
        cylinder_info_sub_ = nh_.subscribe<visualization_msgs::Marker>(cylinder_info_sub_name_, 1, &Base::cylinderInfoCallback, this);

        if(file_log_flag_) log_ofs_.open( "grasp_planning_search_result.txt" );

        /* variables init */
        object_type_ = CONVEX_POLYGONAL_COLUMN;
        best_base_side_ = 0;
        planning_flag_ = -1;
      }

    joint_states_pub_ = nh_.advertise<sensor_msgs::JointState>(joint_states_pub_name_, 1);


    /* timer init */
    func_timer_ = nhp_.createTimer(ros::Duration(1.0 / func_loop_rate_), &Base::mainFunc,this);

  }

  void Base::baseRosParamInit()
  {
    std::string ns = nhp_.getNamespace();

    nhp_.param("debug", debug_, false);
    nhp_.param("test_grasp_flag", test_grasp_flag_, false);
    nhp_.param("play_file_flag", play_file_flag_, false);
    nhp_.param("file_log_flag", file_log_flag_, false);
    nhp_.param("file_result_flag", file_result_flag_, false);
    nhp_.param("control_test_flag", control_test_flag_, false);
    //ROS_WARN("debug: %s, test grasp flag: %s", debug_?std::string("true").c_str():std::string("false").c_str(), test_grasp_flag_?std::string("true").c_str():std::string("false").c_str());

    nhp_.param("convex_polygonal_column_info_sub_name", convex_polygonal_column_info_sub_name_, std::string("/convex_polygonal_column_config"));
    nhp_.param("cylinder_info_sub_name", cylinder_info_sub_name_, std::string("/cylinder_config"));
    nhp_.param("joint_states_pub_name", joint_states_pub_name_, std::string("/planning/joint_states"));
    nhp_.param("planning_file_name", planning_file_name_, std::string("planning_log.txt"));
    ROS_INFO("planning file name: %s", planning_file_name_.c_str());
    nhp_.param("func_loop_rate", func_loop_rate_, 10.0);

    nhp_.param("link_length", link_length_, 0.0);
    nhp_.param("link_radius", link_radius_, 0.0);
    nhp_.param("link_num", link_num_, 0);

    nhp_.param("object_mass", object_mass_, 0.5); //should not be here!!
    nhp_.param("fric_x_mu", fric_x_mu_, 0.1); //should not be here!!
    nhp_.param("fric_z_mu", fric_z_mu_, 0.1); //should not be here!!
    nhp_.param("joint_angle_limit", joint_angle_limit_, 1.65);

    /* tf */
    nhp_.param("link1_frame_name", link1_frame_name_, std::string("link1")); //should not be here!!
    nhp_.param("object_frame_name", object_frame_name_, std::string("object")); //should not be here!!

    /* qp problem */
    nhp_.param("n_wsr", n_wsr_, 10);

    /* the grasp approaching control variables */
    nhp_.param("approach_base_link", approach_base_link_, 0);
    nhp_.param("approach_pos_weight_rate", approach_pos_weight_rate_, 2.0);
    nhp_.param("approach_angle_weight_rate", approach_angle_weight_rate_, 2.0);
    std::cout << "approach_base_link: " << approach_base_link_ << ", "
              << "approach_pos_weight_rate: " << approach_pos_weight_rate_ << ", "
              << "approach_angle_weight_rate: " << approach_angle_weight_rate_
              << std::endl;

  }

  void Base::mainFunc(const ros::TimerEvent & e)
  {
    if(planning_flag_ < 0) return;

    if(planning_flag_ == 0)
      {
        planning_flag_ ++;
        /* test the jointCalc and linkStatics method */
        if(test_grasp_flag_) graspPlanningTest();
        /* planning the grasp pose */
        else graspPlanning();
        /* show the result */
        ROS_WARN("finish!");
        showResult();

        if(file_result_flag_) resultRecord2File();

      }

    /* publish the final result */
    sensor_msgs::JointState planning_joint_states;
    planning_joint_states.header.stamp = ros::Time::now();
    planning_joint_states.position.resize(link_num_ - 1, 0);
    planning_joint_states.name.resize(link_num_ - 1);
    for(int i = 0; i < contact_num_ - 1; i++)
      {
        std::stringstream joint_no;
        joint_no << i + 1;
        planning_joint_states.position[i] = v_best_theta_[i](0);
        planning_joint_states.name[i] = std::string("joint") + joint_no.str();
      }
    joint_states_pub_.publish(planning_joint_states);

    /* broadcast tf from object coord to rkbkt */
    tf::Transform transform;
    if(object_type_ == CONVEX_POLYGONAL_COLUMN)
      transform.setOrigin( tf::Vector3(v_best_contact_d_[0], -link_radius_, 0.0) );
    else
      transform.setOrigin( tf::Vector3(0, - cylinder_radius_ - link_radius_, 0.0) );
    tf::Quaternion q;
    q.setRPY(0, 0, M_PI + v_best_phy_[0]);
    transform.setRotation(q);

    /* hardcoding for control testing: not good */
    if(control_test_flag_)
      {
        transform.setOrigin( tf::Vector3(object_approach_offset_x_ + cog_object_.x(), object_approach_offset_y_ + cog_object_.y(), 0.0) );
        q.setRPY(0, 0, object_approach_offset_yaw_);
        transform.setRotation(q);
        std::stringstream link_no;
        link_no << approach_base_link_ + 1;

        br_.sendTransform(tf::StampedTransform(transform.inverse(), ros::Time::now(), std::string("link") + link_no.str(), object_frame_name_));
      }
    else
      br_.sendTransform(tf::StampedTransform(transform.inverse(), ros::Time::now(), link1_frame_name_, object_frame_name_));
  }

  void Base::convexPolygonalColumnInfoCallback(const geometry_msgs::PolygonStampedConstPtr & msg)
  {
    if(planning_flag_ >= 0) return;

    /* variables init */
    object_type_ = CONVEX_POLYGONAL_COLUMN;

    /* calculate the object geometric information */
    bool health_flag = convexPolygonalColumnCalc(*msg);
    if(!health_flag) return; // something wrong

    /* init */
    v_best_theta_.resize(contact_num_, Vector2d::Zero()); //x: contact_num -1, y: contact_num
    v_best_phy_.resize(contact_num_);
    v_best_contact_d_.resize(contact_num_);
    /* based on the first link coordinate */
    v_best_contact_p_.resize(contact_num_);
    v_best_joint_p_.resize(contact_num_ - 1);
    v_best_f_fc_ = VectorXd::Constant(contact_num_ * 3, 1e6);
    v_best_tau_ = VectorXd::Constant(contact_num_ - 1, 1e6);

    qpInit();

    /* guarantee the onetime planning */
    planning_flag_ = 0;
    convex_polygonal_column_info_sub_.shutdown();
  }

  void Base::cylinderInfoCallback(const visualization_msgs::MarkerConstPtr & msg)
  {
    if(planning_flag_ >= 0) return;

    /* variables init */
    object_type_ = CYLINDER;
    cylinder_radius_  = msg->scale.x / 2; /* assume scale.x = scale.y */

    bool health_flag = cylinderCalc();
    if(!health_flag) return; // something wrong

    /* init */
    v_best_theta_.resize(contact_num_, Vector2d::Zero());
    v_best_phy_.resize(contact_num_);
    v_best_contact_d_.resize(contact_num_);
    /* based on the first link coordinate */
    v_best_contact_p_.resize(contact_num_);
    v_best_joint_p_.resize(contact_num_ - 1);
    v_best_f_fc_ = VectorXd::Constant(contact_num_ * 3, 1e6);
    v_best_tau_ = VectorXd::Constant(contact_num_ - 1, 1e6); //same number with joints

    qpInit();

    /* guarantee the onetime planning */
    planning_flag_ = 0;
  }

  bool Base::graspPlanningTest()
  {
    /* 1. calculate the joint angles */
    if(object_type_ == CONVEX_POLYGONAL_COLUMN)
      {
#if 1 // center contact test
        /* test the link statics  */
        float d = v_vertex_p_[1](0) / 2;
        float phy = 0;
        if(debug_) ROS_INFO("test grasp planner: first contact distance: %f", d);
#else // random in the valid range
        boost::random::mt19937 engine;
        boost::random::uniform_real_distribution<> dist(0.0, 1.0);
        float d =  v_side_length_[1] * dist(engine);
        float phy = (atan2(link_radius_, v_side_length_[1] -d ) + atan2(link_radius_, d))  * dist(engine) - atan2(link_radius_, d) ;
        if(debug_) ROS_INFO("test grasp planner: first contact distance: %f, phy: %f", d, phy);
#endif

        if(!convexPolygonBasedJointsCalc(d, phy, v_best_theta_, v_best_phy_, v_best_contact_d_, v_best_contact_p_, v_best_joint_p_))
          {
            ROS_ERROR("graspPlanningTest: no joint angles solution");
            return false;
          }
      }
    else if(object_type_ == CYLINDER)
      {
#if 1 // center contact test
        /* test the link statics  */
        float phy = 0.0;
        if(debug_) ROS_INFO("test grasp planner: perpendicular to the norm of contact point");
#else // random in the valid range
        boost::random::mt19937 engine;
        boost::random::uniform_real_distribution<> dist(0.0, 1.0);
        float phy_dash = M_PI / 2  - acos((link_length_ / 2) / (link_radius_ + cylinder_radius_));
        float phy = 2 * phy_dash * dist(engine) - phy_dash ;
        if(debug_) ROS_INFO("test grasp planner: first contact phy: %f", phy);
#endif
        if(!circleBasedJointsCalc(phy, v_best_theta_, v_best_phy_, v_best_contact_p_, v_best_joint_p_))
          {
            ROS_ERROR("graspPlanningTest: no joint angles solution");
            return false;
          }
      }

    /* 2. calculate the joint angles */
     v_best_tau_ = VectorXd::Zero(contact_num_ - 1);
     linkStatics(v_best_contact_p_, v_best_joint_p_, v_best_tau_, v_best_f_fc_);

     ROS_WARN("graspPlanningTest: the test reuslt is :");
     std::cout << " best tau_: [" << v_best_tau_.transpose() << "]" << std::endl;
     std::cout << " tau norm: " << v_best_tau_.norm() << std::endl;
     std::cout << " best min f fc: [" << v_best_f_fc_.transpose() << "]"  << std::endl;

    return true;
  }

  bool Base::convexPolygonalColumnCalc(geometry_msgs::PolygonStamped object_msg)
  {
    side_num_ = object_msg.polygon.points.size();
    cog_object_ = Vector3d(0,0,0);

    if(side_num_ > link_num_)
      {
        ROS_WARN("the num of the vertex is more than that of link");
        contact_num_ = link_num_;
      }
    else if(side_num_ < link_num_)
      {
        ROS_WARN("the num of the vertex is less than that of link");
        contact_num_ = side_num_;
      }
    else
      {
        ROS_WARN("the num of the vertex is equal with that of link");
        contact_num_ = side_num_;
      }

    /* init about the config of vertex */
    v_orig_psi_.resize(side_num_);
    v_orig_vertex_p_.resize(side_num_);

    v_psi_.resize(side_num_);
    v_vertex_p_.resize(side_num_);
    v_side_length_.resize(side_num_);

    float abs_psi = 0;
    for(int i = 0; i < side_num_; i++)
      {
        int p1 = i % side_num_;
        int p2 = (i + 1) % side_num_;
        int p3 = (i + 2) % side_num_;
        v_orig_vertex_p_[p1] = Vector3d(object_msg.polygon.points[p1].x, object_msg.polygon.points[p1].y, 0);
        v_orig_psi_[p1](0) = atan2(object_msg.polygon.points[p3].y -object_msg.polygon.points[p2].y,
                                object_msg.polygon.points[p3].x -object_msg.polygon.points[p2].x)
          - atan2(object_msg.polygon.points[p2].y - object_msg.polygon.points[p1].y,
                  object_msg.polygon.points[p2].x - object_msg.polygon.points[p1].x);

        if(v_orig_psi_[p1](0) > M_PI ) v_orig_psi_[p1](0) -= (2 * M_PI);
        if(v_orig_psi_[p1](0) < -M_PI ) v_orig_psi_[p1](0) += (2 * M_PI);

        v_orig_psi_[p1](1) = abs_psi;
        if(v_orig_psi_[p1](1) > M_PI ) v_orig_psi_[p1](1) -= (2 * M_PI);
        if(v_orig_psi_[p1](1) < -M_PI ) v_orig_psi_[p1](1) += (2 * M_PI);

        abs_psi +=  v_orig_psi_[p1](0);

        ROS_INFO("objectCalc: original vertex %d: x: %f, y: %f, rel psi: %f, abs psi: %f", p1, v_orig_vertex_p_[p1](0), v_orig_vertex_p_[p1](1), v_orig_psi_[p1](0), v_orig_psi_[p1](1));
      }

    /* fill the v_psi_ and v_vertex_p_ based on the first side */
    convexPolygonalColumnVertexConvert(0);

    return true;
  }

  void Base::convexPolygonalColumnVertexConvert(int base_side)
  {
    float abs_psi = 0;
    /* the orientation of the original coordinate towards frame of base side */
    Quaternion<double> rotate(AngleAxisd(-v_orig_psi_[base_side](1), Vector3d::UnitZ()));
    cog_object_ = Vector3d(0,0,0);

    if(debug_)
      {
        ROS_INFO("objectVertexConvert: object vertex convert: base side: %d", base_side);
        std::cout << "  rotate : \n " << rotate.matrix() << std::endl;
      }

    for(int i = 0; i < side_num_; i++)
      {
        int p = (base_side + i) % side_num_;
        Vector3d v_vertex_p = v_orig_vertex_p_[p] - v_orig_vertex_p_[base_side];
        v_vertex_p_[i] = rotate * v_vertex_p;
        v_side_length_[i] = (v_orig_vertex_p_[(base_side + i + 1) % side_num_]  - v_orig_vertex_p_[p]).norm();
        v_psi_[i](0) = v_orig_psi_[p](0);
        v_psi_[i](1) = abs_psi;
        abs_psi +=  v_psi_[i](0);
        if(abs_psi > M_PI ) abs_psi -= (2 * M_PI);
        if(abs_psi < -M_PI ) abs_psi += (2 * M_PI);
        Vector3d cog_object_tmp = cog_object_;
        cog_object_ = cog_object_tmp + v_vertex_p_[i];

        ROS_INFO("objectVertexConvert: vertex %d: x: %f, y: %f, side length: %f, rel psi: %f, abs psi: %f", p, v_vertex_p_[i](0), v_vertex_p_[i](1), v_side_length_[i], v_psi_[i](0), v_psi_[i](1));
      }

    cog_object_ /= side_num_;
    ROS_INFO("objectVertexConvert: cog of object is (%f, %f)", cog_object_[0], cog_object_[1]);
    return;
  }

  bool Base::cylinderCalc()
  {
    /*
      note that: we roughly calculate the contact number with cylinder,
      in the condition that the each link is perpendicular with contact point normal line.
    */

    /* the angle between the lines of neighbour contact points to centers */
    float psi = atan2(link_length_ /2, link_radius_ + cylinder_radius_) * 2;

    /* check whether the link num is enough */
    if(psi * (link_num_ -1 ) <= M_PI) 
      {
        if(debug_)
          ROS_WARN("cylinderObjectCalc: the link_num of link_lenght is too short to envelope the whole cylinder, psi * (link_num_ -1 ): %f", psi * (link_num_ -1 ));
        return false;
      }

    contact_num_ = link_num_;

    if(debug_) ROS_INFO("cylinderObjectCalc: contact_num: %d, psi: %f", contact_num_, psi);

    /* check whether the link num exceed the enveloping condition */
    float exceed_angle = psi * link_num_ - 2 * M_PI;
    //ROS_INFO("exceed_angle:%f", exceed_angle);
    if(exceed_angle > 1e-6)
      {
        contact_num_ -= ((int)exceed_angle / (int)psi + 1);
        if(debug_)
          ROS_WARN("cylinderObjectCalc: the link_num exceeds the whole cylinder, contact_num: %d", contact_num_);
      }

    /* init about the config of vertex */
    // x(): the angle before two neighbour contact point lines
    // y(): the direction of contact point tangent line
    v_orig_psi_.resize(contact_num_);
    v_psi_.resize(contact_num_);
    cog_object_ = Vector3d(0, 0, 0);

    return true;
  }

  bool Base::linkStatics(std::vector<Vector3d> v_contact_p, std::vector<Vector3d> v_joint_p, VectorXd& tau, VectorXd& min_f_fc)
  {
    /* 1. calculate the force in each contact point*/
    /* this is based on the force closure and the minimize the internal forces */
    /* force closure: G_c\bm{f_{FC}} + \F_e = 0, \bm{f_{FC}} = [\bm{f_{ci}}, \cdots, \bm{f_{cin}]^{t}  */
    /* constraints: \bm{f_{ci}} \in FC,  FC = { [f_x, f_y, f_z]^T \mid  f^2_y + f^2_z \leq \mu^2 f^2_x, f_x \req 0} */
    /* linear constraints: FC = { [f_x, f_y, f_z]^T \mid f_x \req 0,  -\mu f_x \leq f_y \leq \mu f_x, 0 \leq f_z \leq \mu f_x} */

    /* force closure constraints */
    MatrixXd G_c = MatrixXd::Zero(6, 3 * contact_num_);
    for(int i = 0; i < contact_num_; i++)
      {
        /* G_c = [R_c [P_oc x]R_c] */
        Quaternion<double> R_ci(AngleAxisd(v_psi_[i][1] + M_PI / 2, Vector3d::UnitZ())); //absolute angle from first link

        MatrixXd G_ci(6, 3);
        G_ci.block(0, 0, 3, 3) = R_ci.matrix();
        G_ci.block(3, 0, 3, 3) = skew(v_contact_p[i] - cog_object_) * R_ci.matrix();

        //TODO remove!!!
        for(int j = 0; j < 6; j++)
          {
            for(int k = 0; k < 3; k++)
              {
                if(fabs(G_ci(j,k)) < 1e-6) G_ci(j,k) = 0;
              }
          }

        G_c.block(0, 3 * i, 6, 3) = G_ci;
      }
    A_.block(0,0, 6, 3 * contact_num_) = G_c;
    if(debug_) 
      {
        std::cout << "  G_c : \n " << G_c << std::endl;
        std::cout << "  A_ : \n " << A_  << std::endl;
      }
    MatrixXd At = A_.transpose(); //eigen is col-major order array in default

    /* approximated friction constraints */
    /* constant */

    /* calulate the minimum force-closure based QP solver */
    //double start_tm = ros::Time::now().toSec();
    n_wsr_ = 100; //why can not set from rosparam?
    int solver_result = 0;
#if 1
    if(!qp_init_flag_)
      { /* first time */
        qp_init_flag_ = true;

        solver_result = fc_solver_->init(H_.data(), g_.data(), At.data(),
                         lb_.data(), ub_.data(), lA_.data(), uA_.data(), n_wsr_);
        // std::cout << "  H_ : \n " << H_ << std::endl;
        // std::cout << "  g_ : \n " << g_ << std::endl;
        // std::cout << "  A_ : \n " << A_ << std::endl;
        // std::cout << "  lb_ : \n " << lb_ << std::endl;
        // std::cout << "  ub_ : \n " << ub_ << std::endl;
        // std::cout << "  lA_ : \n " << lA_ << std::endl;
        // std::cout << "  uA_ : \n " << uA_ << std::endl;
      }
    else
      solver_result = fc_solver_->hotstart(H_.data(), g_.data(), At.data(),
                           lb_.data(), ub_.data(), lA_.data(), uA_.data(), n_wsr_);

    fc_solver_->getPrimalSolution(min_f_fc.data());
    //fc_solver_->printProperties();
#else
    SQProblem fc_solver(contact_num_ * 3, contact_num_ * 3 + 6);
    fc_solver.setOptions(qp_options_);
    solver_result = fc_solver.init(H_.data(), g_.data(), At.data(),
                   lb_.data(), ub_.data(), lA_.data(), uA_.data(), n_wsr_);
    fc_solver.getPrimalSolution(min_f_fc.data());
#endif

    //ROS_INFO("the solver_result is %d", solver_result);
    if(solver_result != 0)
      {
        ROS_ERROR("the solver_result is %d", solver_result);
        return false;
      }

    if(debug_)
      std::cout << " min f fc: [" << min_f_fc.transpose() << "]"  << std::endl;

    //ROS_WARN("solution time is %f", ros::Time::now().toSec() - start_tm);

    //example.getDualSolution( yOpt );


    /* 2. calculate the torque of each joint
    /* the statics of link, based on the equation of 3.3.55 at page 119 of "robot motion" */
    /* eq: \bm{\tau} = - \sum{K}{i = 1}J_{i}^{T}(\bm{\theta})\bm{Q}_{i} */
    for(int i = 0; i < contact_num_; i++)
      { // i : the force point
        MatrixXd J_b_i = MatrixXd::Zero(6, contact_num_ - 1);
        for(int j = 0; j < contact_num_ - 1; j++)
          {// j : the joint
            VectorXd  J_b_ij = VectorXd::Zero(6);
            if(i - j > 0)
              {
                Vector3d e_z(0,0,1);
                J_b_ij.block(0, 0, 3, 1) = e_z.cross(v_contact_p[i] - v_joint_p[j]);
                J_b_ij.block(3, 0, 3, 1) = e_z;
                J_b_i.block(0, j, 6 , 1) = J_b_ij;
              }
          }
        if(debug_) std::cout << "  J_b_" << i << " : \n " << J_b_i << std::endl;
        //tau += J_b_i.transpose() * G_c.block(0, 3 * i, 6, 3) * min_f_fc.block(3 * i, 0, 3, 1);
        VectorXd tau_tmp = tau;
        tau = tau_tmp + J_b_i.transpose() * G_c.block(0, 3 * i, 6, 3) * min_f_fc.block(3 * i, 0, 3, 1);
      }
    if(debug_)
      std::cout << " tau_: [" << tau.transpose() << "]" << std::endl;
  }

  bool Base::convexPolygonBasedJointsCalc(double first_contact_d, double first_phy, std::vector<Vector2d>& v_theta, std::vector<float>& v_phy, std::vector<float>& v_contact_d, std::vector<Vector3d>& v_contact_p, std::vector<Vector3d>& v_joint_p)
  {
    float theta = 0; //for each link joint angle
    float abs_theta = first_phy; //for each link joint angle based on the first link
    float contact_d = 0; // for each contact point distance

    /* the first elements */
    v_phy[0] = first_phy;
    v_contact_d[0] = first_contact_d;
    v_contact_p[0](0) = v_contact_d[0];
    v_contact_p[0](1) = 0;
    v_joint_p[0] = v_contact_p[0] + Vector3d(cos(abs_theta) * link_length_ / 2,
                                          sin(abs_theta) * link_length_ / 2
                                          - link_radius_, 0);

    for(int i = 0; i < contact_num_ - 1; i ++)
      {
        /* check the joint is out of the Convex polygon */
        if(v_phy[i] > 0 &&
           -link_radius_ + link_length_ / 2 * sin(v_phy[i]) > 0 &&
           (link_radius_ / (v_side_length_[i] - v_contact_d[i] )) < tan(v_phy[i]))
          {
            if(debug_)
              ROS_WARN("joint calc: the joint is inside the convex polygon, phy %f", v_phy[i]);
            return false;
          }
        if(v_phy[i] < 0 &&
           -link_radius_ + link_length_ / 2 * sin(-v_phy[i]) > 0 &&
           (link_radius_ / v_contact_d[i]) < tan(-v_phy[i]))
          {
            if(debug_)
              ROS_WARN("joint calc: the joint is inside the convex polygon, phy %f", v_phy[i]);
            return false;
          }

        /* joint position based on current side */
        Vector3d joint_p = AngleAxisd(-v_psi_[i](1), Vector3d::UnitZ()) * (v_joint_p[i] - v_contact_p[i]);

#if 0 //when v_psi_[i](0) == M_PI/2, tangent will be infite, should avoid

        /* sin(alpha) = beta_1 * cos(alpha) + beta_2 */
        /* alpha = theta_i + phy_i */

        float beta_1 = tan(v_psi_[i](0));
        float beta_2 = (beta_1 * (-link_radius_ * sin(v_psi_[i](0)) + joint_p(0) - (v_side_length_[i] - v_contact_d[i])) - link_radius_ * cos(v_psi_[i](0)) - joint_p(1)) / (link_length_ / 2);

        /* solve theta_i */
        /* (beta_1^2 + 1) cos(alpha) + 2 * beta_1 * beta_2 + beta_2-2 -1 = 0 */
        float sqrt_part = 1 + beta_1 * beta_1 - beta_2 * beta_2;

        float cos_alpha1 = (- beta_1 * beta_2 + sqrt(sqrt_part)) / (beta_1 * beta_1 + 1);
        float cos_alpha2 = (- beta_1 * beta_2 - sqrt(sqrt_part)) / (beta_1 * beta_1 + 1);
        float sin_alpha1 = beta_1 * cos_alpha1 + beta_2;
        float sin_alpha2 = beta_1 * cos_alpha2 + beta_2;

        float theta1 = atan2(sin_alpha1, cos_alpha1) - v_phy[i];
        float theta2 = atan2(sin_alpha2, cos_alpha2) - v_phy[i];

        if(debug_)
          ROS_WARN("joint calc: beta_1: %f, beta_2: %f, sqrt_part: %f", beta_1, beta_2, sqrt_part);

        if(debug_)
          ROS_WARN("joint calc: theta1: %f, sin_alpha1: %f, cos_alpha1: %f, theta2: %f, sin_alpha2: %f, cos_alpha2: %f", theta1, sin_alpha1, cos_alpha1, theta2, sin_alpha2, cos_alpha2);


#else // gamma = cos_psi_i * beta_2
        float gamma = (sin(v_psi_[i](0)) * (-link_radius_ * sin(v_psi_[i](0)) + joint_p(0) - (v_side_length_[i] - v_contact_d[i])) - link_radius_ * cos(v_psi_[i](0)) * cos(v_psi_[i](0)) - cos(v_psi_[i](0)) * joint_p(1)) / (link_length_ / 2);
        float sqrt_part = cos(v_psi_[i](0)) * cos(v_psi_[i](0)) * (1 - gamma * gamma);

        float cos_alpha1 = -sin(v_psi_[i](0)) * gamma + sqrt(sqrt_part);
        float cos_alpha2 = -sin(v_psi_[i](0)) * gamma - sqrt(sqrt_part);

        float theta1 = acos(cos_alpha1) - v_phy[i];
        float theta2 = acos(cos_alpha2) - v_phy[i];

        if(debug_)
          ROS_WARN("joint calc: gamma: %f, sqrt_part: %f", gamma, sqrt_part);

        if(debug_)
          ROS_WARN("joint calc: theta1: %f, cos_alpha1: %f, theta2: %f, cos_alpha2: %f", theta1, cos_alpha1, theta2, cos_alpha2);

#endif

        if(sqrt_part < 0)
          {
            if(debug_)
              ROS_WARN("joint calc: sqrt is negative, no solution for one side one contact rule, maybe the side is too long");
            return false;
          }

        /* take the bigger one */
        theta = theta1;

        /* check the validation of joint angle */
        if(theta <= 0 || fabs(theta) >= joint_angle_limit_ || theta + v_phy[i] < 1.0e-6)
          {// case1: negative angle means no envelop, case2: angle limitation, case3: no contact with real side
            if(debug_)
              ROS_WARN("joint calc: the %d joint angle is not valid, angle is %f", i + 1, theta);
            return false;
          }

        /* result : */
        /* 1. v_theta: contact_num -1, joint states */
        /* 2. v_phy: contact_num */
        /* 3. v_contact_d: contact_num, first_link coord */
        /* 4. v_contact_p: contact_num, first_link coord */
        /* 5. v_joint_p: contact_num, first_link coord */
        v_theta[i](0) = theta;
        v_theta[i](1) = abs_theta;
        abs_theta += theta;
        if(abs_theta > M_PI ) abs_theta -= (2 * M_PI);
        if(abs_theta < -M_PI ) abs_theta += (2 * M_PI);

        v_phy[i + 1] = theta + v_phy[i] - v_psi_[i].x();
        //ROS_WARN("v_phy[i + 1] : %f", v_phy[i + 1]);
        if(i  < contact_num_ - 2)
          v_joint_p[i+1] = v_joint_p[i] + Vector3d(cos(abs_theta) * link_length_,
                                                 sin(abs_theta) * link_length_, 0);
        else
          v_theta[i+1](1) = abs_theta;

        v_contact_p[i+1] = v_joint_p[i] + Vector3d(cos(abs_theta) * link_length_ / 2,
                                                     sin(abs_theta) * link_length_ / 2, 0)
          + Vector3d(-sin(v_psi_[i+1](1)) * link_radius_,
                     cos(v_psi_[i+1](1)) * link_radius_, 0);
        
        v_contact_d[i+1] = (v_contact_p[i+1] - v_vertex_p_[i+1]).norm();

        /* check the validation of contact point */
        if(v_contact_d[i+1] >= v_side_length_[i+1])
          {
            if(debug_)
              {
                ROS_WARN("v_contact_d[%d]: %f, v_side_length_[i+1]: %f", i, v_contact_d[i+1], v_side_length_[i+1]) ;
                ROS_WARN("joint calc: the %d contact point is not valid, contact distance exceeds the side_length", i + 1);
              }
            return false;
          }
        
        /* check the y of the contact point whether is possitive */
        Vector3d contact_p = AngleAxisd(-v_psi_[i](1), Vector3d::UnitZ()) * (v_contact_p[i+1] - v_contact_p[i]);
        if(contact_p(1) < 1e-6)
          {
            if(debug_)
              ROS_WARN("joint calc: the %d contact point is not valid, contact point y below 0: %f", i + 1, contact_p(1));
            return false;
          }

        if(debug_)
          {
            ROS_INFO("joint calc: the iteration %d, theta: %f, phy: %f, contat dist: %f",
                     i + 1, v_theta[i].x(), v_phy[i + 1], v_contact_d[i+1]);
            std::cout << "v_joint_p[" << i << "]: " << v_joint_p[i].transpose() << std::endl;
            std::cout << "v_contact_p[" << i << "]: " << v_contact_p[i].transpose() << std::endl;
            std::cout << "v_vertex_p[" << i << "]: " << v_vertex_p_[i].transpose() << std::endl;
            if(i == contact_num_ - 2)
              {
                std::cout << "v_contact_p[" << i+1 << "]: " << v_contact_p[i+1].transpose() << std::endl;
                std::cout << "v_vertex_p[" << i+1 << "]: " << v_vertex_p_[i+1].transpose() << std::endl;
              }
          }
      }
    return true;
  }

  bool Base::circleBasedJointsCalc(double first_phy, std::vector<Vector2d>& v_theta, std::vector<float>& v_phy, std::vector<Vector3d>& v_contact_p, std::vector<Vector3d>& v_joint_p)
  {
    float theta = 0; //for each link joint angle
    float abs_theta = first_phy; //for each link joint angle based on the first link

    /* the first elements */
    v_theta[0](1) = first_phy ;
    //v_psi_[0](0) = 0;
    v_psi_[0](1) = 0;
    v_phy[0] = first_phy;
    v_contact_p[0](0) = 0;
    v_contact_p[0](1) = - cylinder_radius_;
    v_joint_p[0] = v_contact_p[0] + Vector3d(cos(abs_theta) * link_length_ / 2,
                                          sin(abs_theta) * link_length_ / 2
                                          - link_radius_, 0);
    /* distance from center to each joint point */
    float d_1 = sqrt(link_length_ * link_length_ / 4 + (link_radius_ + cylinder_radius_) * (link_radius_ + cylinder_radius_) - link_length_ * (link_radius_ + cylinder_radius_) * cos(M_PI / 2 + first_phy));
    float d_2 = sqrt(link_length_ * link_length_ / 4 + (link_radius_ + cylinder_radius_) * (link_radius_ + cylinder_radius_) - link_length_ * (link_radius_ + cylinder_radius_) * cos(M_PI / 2 - first_phy));
    float theta_1 = M_PI - 2 * asin((link_radius_ + cylinder_radius_) / d_1 * sin(M_PI / 2 + first_phy));
    float theta_2 = M_PI - 2 * asin((link_radius_ + cylinder_radius_) / d_2 * sin(M_PI / 2 - first_phy));
    float psi_1 = 2 * asin(link_length_ / 2 / d_1 * sin(M_PI / 2 + first_phy));
    float psi_2 = 2 * asin(link_length_ / 2 / d_2 * sin(M_PI / 2 - first_phy));
    float phy_1 = first_phy;
    float phy_2 = -first_phy;

    if(debug_)
      {
        ROS_INFO("d1: %f, d2: %f, tehta1: %f, theta2: %f, psi1: %f, psi2: %f, phy1: %f, phy2: %f", d_1, d_2, theta_1, theta_2, psi_1, psi_2, phy_1, phy_2);

        ROS_INFO("circle joint calc: contact point: %d, theta_y: %f, v_psi_y: %f",
                 0, v_theta[0].y(), v_psi_[0].y());
        std::cout << "  v_joint_p[0]: " << v_joint_p[0].transpose() << std::endl;
        std::cout << "  v_contact_p[0]: " << v_contact_p[0].transpose() << std::endl;
      }

    for(int i = 1; i < contact_num_; i ++)
      {
        /* result : */
        /* 1. v_theta: contact_num -1, joint states */
        /* 2. v_psi_: contact_num */
        /* 4. v_contact_p: contact_num, first_link coord */
        /* 5. v_joint_p: contact_num, first_link coord */

        /* i starts from 0 */
        float theta = (i % 2 == 0)? theta_1: theta_2;
        float psi = (i % 2 == 0)? psi_1: psi_2;
        float d = (i % 2 == 0)? d_1: d_2;
        float phy = (i % 2 == 0)? phy_1: phy_2;

        v_theta[i-1](0) = theta;
        abs_theta += theta;
        if(abs_theta > M_PI ) abs_theta -= (2 * M_PI);
        if(abs_theta < -M_PI ) abs_theta += (2 * M_PI);
        v_theta[i](1) = abs_theta;

        v_phy[i] = phy;
        v_psi_[i - 1](0) = psi;
        v_psi_[i](1) = v_psi_[i-1](1) + psi;
        if(v_psi_[i](1) > M_PI ) v_psi_[i](1) -= (2 * M_PI);
        if(v_psi_[i](1) < -M_PI ) v_psi_[i](1) += (2 * M_PI);

        if(i < contact_num_ -1)
          v_joint_p[i] = v_joint_p[i-1] + AngleAxisd(v_theta[i](1), Vector3d::UnitZ()) * Vector3d(link_length_, 0, 0);
        v_contact_p[i] = AngleAxisd(v_psi_[i](1) + M_PI/2, Vector3d::UnitZ()) * Vector3d(-cylinder_radius_, 0, 0);

        /* check the validation of joint angle */
        if(theta <= 0 || fabs(theta) >= joint_angle_limit_)
          {
            ROS_FATAL("circle joint calc: something wrong with this algorithm, theta:%f, joint_angle_limit: %f", theta, joint_angle_limit_);
            return false;
          }

        if(debug_)
          {
            ROS_INFO("circle joint calc: contact point: %d, theta_x: %f, theta_y:%f, v_psi_x: %f, v_psi_y: %f",
                     i, v_theta[i-1].x(), v_theta[i].y(), v_psi_[i-1].x(), v_psi_[i].y());
            if(i < contact_num_ -1)
              std::cout << "  v_joint_p[" << i << "]: " << v_joint_p[i].transpose() << std::endl;
            std::cout << "  v_contact_p[" << i << "]: " << v_contact_p[i].transpose() << std::endl;
          }
      }
    return true;
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
    /* 1.1 calculate o_b: the best approach position of center of base link in terms of  best_base_side_ frame */
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
            v_base_link = AngleAxisd(M_PI + v_best_theta_[approach_base_link_](1), Vector3d::UnitZ()) * Vector3d(link_length_ /2, 0, 0);
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
            v_base_link = AngleAxisd(v_best_theta_[approach_base_link_](1), Vector3d::UnitZ()) * Vector3d(link_length_ /2, 0, 0);
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

#if 0 //bad reuslt from the variant
        E_beta1 += beta(0);
        E_beta2 += beta(1);
        E_beta1_beta1 += (beta(0) * beta(0));
        E_beta2_beta2 += (beta(1) * beta(1));
        E_beta1_beta2 += (beta(0) * beta(1));
        E_beta1_beta1_beta1 += (beta(0) * beta(0) * beta(0));
        E_beta2_beta2_beta2 += (beta(1) * beta(1) * beta(1));
        E_beta1_beta2_beta2 += (beta(0) * beta(1) * beta(1));
        E_beta1_beta1_beta2 += (beta(0) * beta(0) * beta(1));
#else
        E_beta1 += (beta(0) * weight_rate);
        E_beta2 += (beta(1) * weight_rate);
#endif

      }

    grasping_cog /= contact_num_;

#if 0 //bad reuslt from the variant
    E_beta1 /= contact_num_;
    E_beta2 /= contact_num_;
    E_beta1_beta1 /= contact_num_;
    E_beta2_beta2 /= contact_num_;
    E_beta1_beta2 /= contact_num_;
    E_beta1_beta1_beta1 /= contact_num_;
    E_beta2_beta2_beta2 /= contact_num_;
    E_beta1_beta2_beta2 /= contact_num_;
    E_beta1_beta1_beta2 /= contact_num_;

    Matrix2d sigma;
    sigma << E_beta1_beta1 - E_beta1 * E_beta1, E_beta1_beta2 - E_beta1 * E_beta2
      , E_beta1_beta2 - E_beta1 * E_beta2, E_beta2_beta2 - E_beta2 * E_beta2;
    if(debug_) std::cout << "sigma: \n" << sigma << std::endl;

    Vector2d b;
    b << E_beta1 * (E_beta1_beta1 + E_beta2 * E_beta2) - E_beta1_beta1_beta1 - E_beta1_beta2_beta2
      , E_beta2 * (E_beta1_beta1 + E_beta2 * E_beta2) - E_beta2_beta2_beta2 - E_beta1_beta1_beta2;
    Vector2d O_b_best_base_side = sigma.inverse() / 2 * b;
#else
    Vector2d O_b_best_base_side;
    O_b_best_base_side << grasping_cog.x() - E_beta1, grasping_cog.y() - E_beta2;
    std::cout << "grasping_cog: " << grasping_cog.transpose() << std::endl;
#endif

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

  void Base::qpInit()
  {
    fc_solver_ = boost::shared_ptr<SQProblem>(new SQProblem(contact_num_ * 3, contact_num_ * 3 + 6));
    qp_init_flag_ = false;

    /* Hessian */
    H_ = MatrixXd::Zero(contact_num_ * 3, contact_num_ * 3);
    Matrix3d h;
    h << 1, 0, 0, 0, 1, 0, 0, 0, 0;
    for(int i = 0; i < contact_num_; i++)
      H_.block(3 * i, 3 * i, 3, 3) = h;

    /* linear constraints */
    A_ = MatrixXd::Zero(6 + contact_num_ * 3, contact_num_ * 3);
    MatrixXd G_fric = MatrixXd::Zero(3 * contact_num_, 3 * contact_num_);
    MatrixXd g_fric(3,3);
    g_fric << fric_x_mu_, -1, 0, fric_x_mu_, 1, 0, fric_z_mu_, 0, -1;
    for(int i = 0; i < contact_num_; i++)
      { /* G_{fric} = diag(G_{fric \ 1}, \cdots, G_{fric \ K}), G_{fric}f_{FC} \req 0 */
        G_fric.block(3 * i, 3 * i, 3, 3) = g_fric;
      }
    A_.block(6, 0, 3 * contact_num_, 3 * contact_num_) = G_fric;
    if(debug_)
      {
        std::cout << "  A_ : \n " << A_ << std::endl;
        std::cout << "  G_fric : \n " << G_fric << std::endl;
      }

    /* gradient */
    g_ = MatrixXd::Zero(1, contact_num_ * 3);

    /* lower & upper bound for linear constraints */
    lA_ = VectorXd::Zero(6 + contact_num_ * 3);
    uA_ = VectorXd::Constant(6 + contact_num_ * 3, INFTY);
    VectorXd Fe(6);
    Fe << 0, 0, -object_mass_ * 9.797, 0, 0, 0; //external force
    lA_.block(0, 0, 6, 1) = - Fe;
    uA_.block(0, 0, 6, 1) = - Fe;
    if(debug_)
      {
        std::cout << "  lA_ T: " << lA_.transpose() << std::endl;
        std::cout << "  uA_ T: " << uA_.transpose() << std::endl;
      }

    /* lower & upper bound for variables */
    lb_ = VectorXd::Zero(contact_num_ * 3);
    for(int i = 0; i < contact_num_; i++)
      lb_(i * 3 + 1) = - INFTY;
    ub_ = VectorXd::Constant(contact_num_ * 3, INFTY);
    if(debug_)
      {
        std::cout << "  lb_ T: " << lb_.transpose() << std::endl;
        std::cout << "  ub_ T: " << ub_.transpose() << std::endl;
      }

    qp_options_.enableEqualities = BT_TRUE;
    qp_options_.printLevel = PL_LOW;
    fc_solver_->setOptions(qp_options_);
  }

  void Base::getResultFromFile()
  {
    std::ifstream ifs(planning_file_name_.c_str());

    if(ifs.fail()) 
      {
        ROS_ERROR("File do not exist");
        return;
      }

    std::stringstream ss[11];
    std::string str;
    std::string header;

    std::getline(ifs, str);
    ss[0].str(str);
    ss[0] >> header >> object_type_;
    ROS_INFO("getResultFromFile: %s: %d", header.c_str(), object_type_);

    std::getline(ifs, str);
    ss[1].str(str);
    if(object_type_ == CONVEX_POLYGONAL_COLUMN)
      {
        ss[1] >> header >> side_num_;
        ROS_INFO("getResultFromFile: %s: %d", header.c_str(), side_num_);
      }
    else if(object_type_ == CYLINDER)
      {
        ss[1] >> header >> cylinder_radius_;
        ROS_INFO("getResultFromFile: %s: %f", header.c_str(), cylinder_radius_);
      }

    std::getline(ifs, str);
    ss[2].str(str);
    ss[2] >> header >> contact_num_;
    ROS_INFO("getResultFromFile: %s: %d", header.c_str(), contact_num_);

    /* init for all neccessary base variables */
    v_best_theta_.resize(contact_num_); //x: contact_num_-1, y: contact_num
    v_best_phy_.resize(contact_num_);
    v_best_contact_p_.resize(contact_num_);
    v_best_contact_d_.resize(contact_num_);
    v_best_tau_ = VectorXd::Constant(contact_num_ -1, 1e6);

    /* init for CONVEX POLYGON OBJECT */
    if(object_type_ == CONVEX_POLYGONAL_COLUMN)
      {
        v_psi_.resize(side_num_);
        v_vertex_p_.resize(side_num_);
        v_orig_psi_.resize(side_num_);
        v_orig_vertex_p_.resize(side_num_);
        v_side_length_.resize(side_num_);

        std::getline(ifs, str);
        ss[3].str(str);
        ss[3] >> header >> best_base_side_;
        ROS_INFO("getResultFromFile: %s: %d", header.c_str(), best_base_side_);

        std::getline(ifs, str);
        ss[4].str(str);
        ss[4] >> header;
        ROS_INFO("getResultFromFile: %s:", header.c_str());
        for(int i = 0; i < side_num_; i++)
          {
            std::stringstream ss_tmp;
            std::getline(ifs, str);
            ss_tmp.str(str);
            ss_tmp >> v_orig_psi_[i](0) >> v_orig_psi_[i](1);
            ROS_INFO("[%f, %f]", v_orig_psi_[i](0), v_orig_psi_[i](1));
          }

        std::getline(ifs, str);
        ss[5].str(str);
        ss[5] >> header;
        ROS_INFO("getResultFromFile: %s:", header.c_str());
        for(int i = 0; i < side_num_; i++)
          {
            std::stringstream ss_tmp;
            std::getline(ifs, str);
            ss_tmp.str(str);
            ss_tmp >> v_orig_vertex_p_[i](0) >> v_orig_vertex_p_[i](1);
            ROS_INFO("[%f, %f]", v_orig_vertex_p_[i](0), v_orig_vertex_p_[i](1));
          }

        /* to obtain v_orig_vertex_p__psi_ and v_vertex_p_ */
        convexPolygonalColumnVertexConvert(best_base_side_);
      }
    else if(object_type_ == CYLINDER)
      {
        v_psi_.resize(contact_num_);
        v_orig_psi_.resize(contact_num_);

        std::getline(ifs, str);
        ss[4].str(str);
        ss[4] >> header;
        ROS_INFO("getResultFromFile: %s:", header.c_str());
        for(int i = 0; i < contact_num_; i++)
          {
            std::stringstream ss_tmp;
            std::getline(ifs, str);
            ss_tmp.str(str);
            ss_tmp >> v_orig_psi_[i](0) >> v_orig_psi_[i](1);
            v_psi_[i] = v_orig_psi_[i];
            ROS_INFO("[%f, %f]", v_orig_psi_[i](0),v_orig_psi_[i](1));
          }
      }

    std::getline(ifs, str);
    ss[6].str(str);
    ss[6] >> header;
    ROS_INFO("getResultFromFile: %s:", header.c_str());
    for(int i = 0; i < contact_num_; i++)
      {
        std::stringstream ss_tmp;
        std::getline(ifs, str);
        ss_tmp.str(str);
        ss_tmp >> v_best_contact_p_[i](0) >> v_best_contact_p_[i](1);
        ROS_INFO("[%f, %f]", v_best_contact_p_[i](0), v_best_contact_p_[i](1));
      }

    if(object_type_ == CONVEX_POLYGONAL_COLUMN)
      v_best_contact_d_[0] = v_best_contact_p_[0](0);

    std::getline(ifs, str);
    ss[7].str(str);
    ss[7] >> header;
    ROS_INFO("getResultFromFile: %s:", header.c_str());
    for(int i = 0; i < contact_num_; i++)
      {
        std::stringstream ss_tmp;
        std::getline(ifs, str);
        ss_tmp.str(str);
        ss_tmp >> v_best_phy_[i];
        ROS_INFO("%f", v_best_phy_[i]);
      }

    std::getline(ifs, str);
    ss[8].str(str);
    ss[8] >> header;
    ROS_INFO("getResultFromFile: %s:", header.c_str());
    for(int i = 0; i < contact_num_; i++)
      {
        std::stringstream ss_tmp;
        std::getline(ifs, str);
        ss_tmp.str(str);
        ss_tmp >> v_best_theta_[i](0) >> v_best_theta_[i](1);
        ROS_INFO("[%f, %f]", v_best_theta_[i](0), v_best_theta_[i](1));
      }

    std::getline(ifs, str);
    ss[9].str(str);
    ss[9] >> header;
    ROS_INFO("getResultFromFile: %s:", header.c_str());
    for(int i = 0; i < contact_num_ - 1; i++)
      {
        std::stringstream ss_tmp;
        std::getline(ifs, str);
        ss_tmp.str(str);
        ss_tmp >> v_best_tau_(i);
        ROS_INFO("%f", v_best_tau_(i));
      }

    planning_flag_ = 1;
  }

  void Base::resultRecord2File()
  {
    std::ofstream ofs; /* for record to a file */
    ofs.open( "grasp_planning_result.txt" );
    ofs << "object_type: " << (int)object_type_  << std::endl;
    /* only necessary for convex polygon object */
    if(object_type_ == CONVEX_POLYGONAL_COLUMN)
      ofs << "side_num: " << side_num_  << std::endl;
    else if(object_type_ == CYLINDER)
      ofs << "object_radius: " << cylinder_radius_  << std::endl;
    ofs << "contact_num: " << contact_num_  << std::endl;

    if(object_type_ == CONVEX_POLYGONAL_COLUMN)
      {
        ofs << "best_base_side: " << best_base_side_ << std::endl;
        ofs << "v_orig_psi: " << std::endl;
        for(int i = 0; i < side_num_; i++)
          ofs << v_orig_psi_[i](0) << " " << v_orig_psi_[i](1) <<std::endl;
        ofs << "v_orig_vertex: " << std::endl;
        for(int i = 0; i < side_num_; i++)
          ofs << v_orig_vertex_p_[i](0) << " " << v_orig_vertex_p_[i](1)  << std::endl;
      }
    else if(object_type_ == CYLINDER)
      {
        /* init of cog */
        cog_object_ = Vector3d(0, 0, 0);
        ofs << "v_orig_psi: " << std::endl;
        for(int i = 0; i < contact_num_; i++)
          ofs << v_orig_psi_[i](0) << " " << v_orig_psi_[i](1) <<std::endl;
      }

    ofs << "v_best_contact_p: " << std::endl;
    for(int i = 0; i < contact_num_; i++)
      ofs << v_best_contact_p_[i](0) << " " << v_best_contact_p_[i](1)  << std::endl;
    ofs << "v_best_phy: " << std::endl;
    for(int i = 0; i < contact_num_; i++)
      ofs << v_best_phy_[i] << std::endl;
    ofs << "v_best_theta: " << std::endl;
    for(int i = 0; i < contact_num_; i++)
      ofs << v_best_theta_[i](0) << " " << v_best_theta_[i](1) << std::endl;
    ofs << "v_best_tau: " << std::endl;
    for(int i = 0; i < contact_num_ - 1; i++)
      ofs << v_best_tau_(i) << std::endl;
  }

  void Base::showResult()
  {
    ROS_INFO("full search:");
    std::cout << "best_base_side: " << best_base_side_ << std::endl;
    std::cout << "v_best_first_phy: " << v_best_phy_[0] << std::endl;
    std::cout << "v_best_contact_d: " << v_best_contact_d_[0] << std::endl;
    std::cout << "best_tau_norm: " << v_best_tau_.norm() << std::endl;

    std::cout << "v_best_theta: " ;
    for(int j = 0; j < contact_num_ ; j ++)
      std::cout << "[" << v_best_theta_[j](0) << ", " << v_best_theta_[j](1) << "] "  ;
    std::cout << std::endl;

    std::cout << "v_best_tau: [ ";
    for(int j = 0; j < contact_num_ - 1 ; j ++)
      std::cout << v_best_tau_[j] << " ";
    std::cout << "]" << std::endl;
    std::cout << "v_best_min_f_fc: [ ";
    for(int j = 0; j < contact_num_ * 3; j ++)
      std::cout << v_best_f_fc_[j] << " ";
    std::cout << "]" <<std::endl;
  }

};


