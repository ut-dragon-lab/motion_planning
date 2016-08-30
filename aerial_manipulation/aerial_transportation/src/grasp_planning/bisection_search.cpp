#include <aerial_transportation/grasp_planning/bisection_search.h>

namespace grasp_planning
{
  void BisectionSearch::initialize(ros::NodeHandle nh, ros::NodeHandle nhp)
  {
    /* super class init */
    baseInit(nh, nhp);

    /* ros param init */
    rosParamInit();
  }

  void BisectionSearch::rosParamInit()
  {
    nhp_.param("thre_d", thre_d_, 0.1); //[mm]
    nhp_.param("thre_phy", thre_phy_, 0.001); //[rad]
    nhp_.param("res_phy", res_phy_, 0.01); //[rad]
    nhp_.param("k", k_, 100); //[times]
  }

  bool BisectionSearch::graspPlanning()
  {
    /* init */
    std::vector<Vector2d> v_theta(contact_num_);
    std::vector<float> v_phy(contact_num_);
    std::vector<float> v_contact_d(contact_num_);
    std::vector<Vector3d> v_contact_p(contact_num_);
    std::vector<Vector3d> v_joint_p(contact_num_ - 1);
    VectorXd min_f_fc = VectorXd::Zero(contact_num_ * 3);
    VectorXd tau = VectorXd::Zero(contact_num_);

    std::vector<Vector2d> v_theta1(contact_num_);
    std::vector<float> v_phy1(contact_num_);
    std::vector<float> v_contact_d1(contact_num_);
    std::vector<Vector3d> v_contact_p1(contact_num_);
    std::vector<Vector3d> v_joint_p1(contact_num_ - 1);
    VectorXd min_f_fc1 = VectorXd::Zero(contact_num_ * 3);
    VectorXd tau1 = VectorXd::Zero(contact_num_);

    std::vector<Vector2d> v_theta2(contact_num_);
    std::vector<float> v_phy2(contact_num_);
    std::vector<float> v_contact_d2(contact_num_);
    std::vector<Vector3d> v_contact_p2(contact_num_);
    std::vector<Vector3d> v_joint_p2(contact_num_ - 1);
    VectorXd min_f_fc2 = VectorXd::Zero(contact_num_ * 3);
    VectorXd tau2 = VectorXd::Zero(contact_num_);

    /* bisection: */
    /* search d, then phy*/
    if(object_type_ == CONVEX_POLYGONAL_COLUMN)
      {
        /* full search for each side to serve as the first side*/
        for(int i = 0; i < side_num_; i++)
          {
            convexPolygonalColumnVertexConvert(i);

            /* init */
            float best_d = 0;
            float best_phy1 = 0; //best phy for d1
            float best_phy2 = 0; //best phy for d2
            /* d: [0, v_side_length_[1]] */
            /* phy: [- atan2(link_radius_, d), atan2(link_radius_, v_side_length_[1] -d )] */
            float ld = 0;
            float ud = v_side_length_[1];
            float md = (ld + ud) /2;
            float d1 = (md + ld) / 2;
            float d2 = (md + ud) / 2;

            /* 1. find better d */
            //while(1)
            for(int j = 0; j < k_; j++)
              {
                /* 2. find better phy for d1  */

                std::vector<Vector2d> v_theta_tmp1(contact_num_);
                std::vector<float> v_phy_tmp1(contact_num_);
                std::vector<float> v_contact_d_tmp1(contact_num_);
                std::vector<Vector3d> v_contact_p_tmp1(contact_num_);
                std::vector<Vector3d> v_joint_p_tmp1(contact_num_ - 1);
                VectorXd min_f_fc_tmp1 = VectorXd::Zero(contact_num_ * 3);
                VectorXd tau_tmp1 = VectorXd::Zero(contact_num_);

                std::vector<Vector2d> v_theta_tmp2(contact_num_);
                std::vector<float> v_phy_tmp2(contact_num_);
                std::vector<float> v_contact_d_tmp2(contact_num_);
                std::vector<Vector3d> v_contact_p_tmp2(contact_num_);
                std::vector<Vector3d> v_joint_p_tmp2(contact_num_ - 1);
                VectorXd min_f_fc_tmp2 = VectorXd::Zero(contact_num_ * 3);
                VectorXd tau_tmp2 = VectorXd::Zero(contact_num_);

                float l_phy = - atan2(link_radius_, d1);
                float u_phy = atan2(link_radius_, v_side_length_[1] -d1);
                float m_phy = (l_phy + u_phy) /2;
                float phy1 = (m_phy + l_phy) / 2;
                float phy2 = (m_phy + u_phy) / 2;

                //while(1)
                for(int l = 0; l < k_; l++)
                  {
                    /* 2.1 calculate under d1, phy1 */
                    while(1)
                      {
                        bool validity = true;

                        /* 1. calculate the joint angles */
                        if(!convexPolygonBasedJointsCalc(d1, phy1, v_theta_tmp1, v_phy_tmp1, v_contact_d_tmp1, v_contact_p_tmp1, v_joint_p_tmp1))
                          validity = false;
                        /* 2. calculate the joint angles */
                        if(!linkStatics(v_contact_p_tmp1, v_joint_p_tmp1, tau_tmp1, min_f_fc_tmp1))
                          validity = false;

                        /* if not valid, move close to the center point . assume thant center is better than the bound */
                        if(!validity) 
                          {
                            phy1 += res_phy_;
                            if(debug_) ROS_INFO("bisection: d1, phy1 is not valid, move close to cneter ");
                          }
                        else break;
                      }

                    /* 2.2 calculate uder d1, phy2 */
                    while(1)
                      {
                        bool validity = true;

                        /* 1. calculate the joint angles */
                        if(!convexPolygonBasedJointsCalc(d1, phy2, v_theta_tmp2, v_phy_tmp2, v_contact_d_tmp2, v_contact_p_tmp2, v_joint_p_tmp2))
                          validity = false;
                        /* 2. calculate the joint angles */
                        if(!linkStatics(v_contact_p_tmp2, v_joint_p_tmp2, tau_tmp2, min_f_fc_tmp2))
                          validity = false;

                        /* if not valid, move close to the center point . assume thant center is better than the bound */
                        if(!validity)
                          {
                            phy2 -= res_phy_;
                            if(debug_) ROS_INFO("bisection: d1, phy2 is not valid, move close to cneter ");
                          }
                        else break;
                      }

                    /* 2.3 compare and update */
                    if(tau_tmp1.norm() < tau_tmp2.norm()) /* use d1, phy1 */
                      u_phy = m_phy;
                    else
                      l_phy = m_phy;

                    m_phy = (l_phy + u_phy) /2;
                    phy1 = (m_phy + l_phy) / 2;
                    phy2 = (m_phy + u_phy) / 2;
                    best_phy1 = m_phy;
                  }

                /* 2.4 the best phy in d1 */
                if(!convexPolygonBasedJointsCalc(d1, best_phy1, v_theta1, v_phy1, v_contact_d1, v_contact_p1, v_joint_p1))
                  ROS_FATAL("bisection for convex: d1 and best_phy is not valid in terms of joint calc");
                if(!linkStatics(v_contact_p1, v_joint_p1, tau1, min_f_fc1))
                  ROS_FATAL("bisection for convex: d1 and best_phy is not valid in terms of statics");

                if(file_log_flag_)
                  log_ofs_ << d1  << " " << best_phy1 << " " << tau1.norm() << " " << min_f_fc1.transpose() * H_ * min_f_fc1  << std::endl;

                //while(1)
                for(int l = 0; l < k_; l++)
                  {
                    /* 3.1 calculate under d2, phy1 */
                    while(1)
                      {
                        bool validity = true;

                        /* 1. calculate the joint angles */
                        if(!convexPolygonBasedJointsCalc(d2, phy1, v_theta_tmp1, v_phy_tmp1, v_contact_d_tmp1, v_contact_p_tmp1, v_joint_p_tmp1))
                          validity = false;
                        /* 2. calculate the joint angles */
                        if(!linkStatics(v_contact_p_tmp1, v_joint_p_tmp1, tau_tmp1, min_f_fc_tmp1))
                          validity = false;

                        /* if not valid, move close to the center point . assume thant center is better than the bound */
                        if(!validity) 
                          {
                            phy1 += res_phy_;
                            if(debug_) ROS_INFO("bisection for convex: d2, phy1 is not valid, move close to cneter ");
                          }
                        else break;
                      }

                    /* 3.2 calculate uder d2, phy2 */
                    while(1)
                      {
                        bool validity = true;

                        /* 1. calculate the joint angles */
                        if(!convexPolygonBasedJointsCalc(d2, phy2, v_theta_tmp2, v_phy_tmp2, v_contact_d_tmp2, v_contact_p_tmp2, v_joint_p_tmp2))
                          validity = false;
                        /* 2. calculate the joint angles */
                        if(!linkStatics(v_contact_p_tmp2, v_joint_p_tmp2, tau_tmp2, min_f_fc_tmp2))
                          validity = false;

                        /* if not valid, move close to the center point . assume thant center is better than the bound */
                        if(!validity)
                          {
                            phy2 -= res_phy_;
                            if(debug_) ROS_INFO("bisection for convex: d2, phy2 is not valid, move close to cneter ");
                          }
                        else break;
                      }

                    /* 2.3 compare and update */
                    if(tau_tmp1.norm() < tau_tmp2.norm())/* use d2, phy1 */
                      u_phy = m_phy;
                    else
                      l_phy = m_phy;

                    m_phy = (l_phy + u_phy) /2;
                    phy1 = (m_phy + l_phy) / 2;
                    phy2 = (m_phy + u_phy) / 2;
                    best_phy2 = m_phy;
                  }

                /* 3.4 the best phy in d2 */
                if(!convexPolygonBasedJointsCalc(d2, best_phy2, v_theta2, v_phy2, v_contact_d2, v_contact_p2, v_joint_p2))
                  ROS_FATAL("bisection for convex: d2 and best_phy is not valid in terms of joint calc");
                if(!linkStatics(v_contact_p2, v_joint_p2, tau2, min_f_fc2))
                  ROS_FATAL("bisection for convex: d2 and best_phy is not valid in terms of statics");
                if(file_log_flag_)
                  log_ofs_ << d2  << " " << best_phy2 << " " << tau2.norm() << " " << min_f_fc2.transpose() * H_ * min_f_fc2  << std::endl;

                /* 4 compare d1 and d2, update */
                if(tau1.norm() < tau2.norm()) /* select d1 */
                  ud = md;
                else /* select d1 */
                  ld = md;

                md = (ld + ud) / 2;
                d1 = (md + ld) / 2;
                d2 = (md + ud) / 2;

                if(debug_) ROS_INFO("bisection for convex: d1: %f, best_phy1: %f, d2: %f, best_phy2: %f", d1, best_phy1, d2, best_phy2);
              }

            /* 5 the best phy d */
            if(!convexPolygonBasedJointsCalc(md, (best_phy1 + best_phy2) / 2 , v_theta, v_phy, v_contact_d, v_contact_p, v_joint_p))
              ROS_FATAL("bisection for convex: best d and best_phy is not valid in terms of joint calc");
            if(!linkStatics(v_contact_p, v_joint_p, tau, min_f_fc))
              ROS_FATAL("bisection for convex: best d and best_phy is not valid in terms of statics");
            if(file_log_flag_)
              log_ofs_ << md  << " " << (best_phy1 + best_phy2) / 2 << " " << tau.norm() << " " << min_f_fc.transpose() * H_ * min_f_fc  << std::endl;

            if(tau.norm() < v_best_tau_.norm())
              {
                /* find better grasp pose */
                v_best_theta_ = v_theta;
                v_best_phy_ = v_phy;
                v_best_contact_p_ = v_contact_p;
                v_best_joint_p_ = v_joint_p;
                v_best_tau_ = tau;
                v_best_f_fc_ = min_f_fc;
                best_base_side_ = i;
              }

            if(one_side_flag_) break;
          }
      }
    else if(object_type_ == CYLINDER)
      {
        float u_phy = M_PI / 2  - acos((link_length_ / 2) / (link_radius_ + cylinder_radius_));
        float l_phy = - u_phy;
        float m_phy = (l_phy + u_phy) /2;
        float phy1 = (m_phy + l_phy) / 2;
        float phy2 = (m_phy + u_phy) / 2;
        float best_phy = 0;

        for(int i = 0; i < k_; i++)
          {
            if(!circleBasedJointsCalc(phy1, v_theta1, v_phy1, v_contact_p1, v_joint_p1))
              ROS_FATAL("bisection for circle: no valid phy1 for joint calc");
            if(!linkStatics(v_contact_p1, v_joint_p1, tau1, min_f_fc1))
              ROS_FATAL("bisection for circle: no valid phy1 for statics calc");

            if(!circleBasedJointsCalc(phy2, v_theta2, v_phy1, v_contact_p2, v_joint_p2))
              ROS_FATAL("bisection for circle: no valid phy2 for joint calc");
            if(!linkStatics(v_contact_p2, v_joint_p2, tau2, min_f_fc2))
              ROS_FATAL("bisection for circle: no valid phy2 for statics calc");

            if(tau1.norm() < tau2.norm())
              {
                u_phy = m_phy;
                best_phy = phy1;
              }
            else
              {
                l_phy = m_phy;
                best_phy = phy2;
              }

            m_phy = (l_phy + u_phy) /2;
            phy1 = (m_phy + l_phy) / 2;
            phy2 = (m_phy + u_phy) / 2;

            if(file_log_flag_)
              {
                log_ofs_ << i << " " << phy1 << " " << tau1.norm() << " " << min_f_fc1.transpose() * H_ * min_f_fc1  << std::endl;
                log_ofs_ << i << " " << phy2 << " " << tau2.norm() << " " << min_f_fc2.transpose() * H_ * min_f_fc2  << std::endl;
              }
          }
        if(!circleBasedJointsCalc(best_phy, v_best_theta_, v_best_phy_, v_best_contact_p_, v_best_joint_p_))
          ROS_FATAL("bisection for circle:  best_phy is not valid in terms of joint calc");
        if(!linkStatics(v_best_contact_p_, v_best_joint_p_, v_best_tau_, v_best_f_fc_))
          ROS_FATAL("bisection for circle:  best_phy is not valid in terms of statics");
      }
    return true;
  }

};

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(grasp_planning::BisectionSearch, grasp_planning::Base);

