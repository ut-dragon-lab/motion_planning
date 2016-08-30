#include <aerial_transportation/grasp_planning/full_search.h>

namespace grasp_planning
{
  void FullSearch::initialize(ros::NodeHandle nh, ros::NodeHandle nhp)
  {
    /* super class init */
    baseInit(nh, nhp);

    /* ros param init */
    rosParamInit();
  }

  void FullSearch::rosParamInit()
  {
    nhp_.param("one_side_flag", one_side_flag_, true); 
    nhp_.param("res_d", res_d_, 0.001); //[m]
    nhp_.param("res_phy", res_phy_, 0.01); //[rad]
  }

  bool FullSearch::graspPlanning()
  {
    /* init */
    std::vector<Vector2d> v_theta(contact_num_);
    std::vector<float> v_phy(contact_num_);
    std::vector<float> v_contact_d(contact_num_);
    std::vector<Vector3d> v_contact_p(contact_num_);
    std::vector<Vector3d> v_joint_p(contact_num_ - 1);

    if(object_type_ == CONVEX_POLYGONAL_COLUMN)
      {
        /* full search for each side to serve as the first side*/
        for(int i = 0; i < side_num_; i++)
          {
            convexPolygonalColumnVertexConvert(i);

            for(float d = 0; d <= v_side_length_[0]; d += res_d_)
              {
                /* a not full range for phy, because of the difficulty in modelling the point near the vertex */
                float l_phy = - atan2(link_radius_, d);
                float u_phy = atan2(link_radius_, v_side_length_[0] -d );

                //l_phy = 0;
                // u_phy = 0;

                for(float phy = l_phy; phy <= u_phy; phy += res_phy_)
                  {
                    VectorXd min_f_fc = VectorXd::Zero(contact_num_ * 3);
                    VectorXd tau = VectorXd::Zero(contact_num_ -1 );

                    bool kinematics_validity = true;
                    bool statics_validity = true;
                    /* 1. calculate the joint angles */
                    if(!convexPolygonBasedJointsCalc(d, phy, v_theta, v_phy, v_contact_d, v_contact_p, v_joint_p))
                      kinematics_validity = false;
                    /* 2. calculate the joint angles */
                    if(kinematics_validity)
                      {
                      if(!linkStatics(v_contact_p, v_joint_p, tau, min_f_fc))
                        statics_validity = false;
                      }
                    else
                      statics_validity = false;

                    /* 2.5 show the result of the validity */
                    // if(!kinematics_validity)
                    //   ROS_WARN("d:%f, phy:%f, l_phy:%f, u_phy:%f, kinematics invalid", d, phy, l_phy, u_phy);
                    if(kinematics_validity && !statics_validity)
                      ROS_ERROR("d:%f, phy:%f, l_phy:%f, u_phy:%f, statics invalid", d, phy, l_phy, u_phy);
                    if(kinematics_validity && statics_validity)
                      {
                        ROS_INFO("d:%f, phy:%f, l_phy:%f, u_phy:%f, valid state", d, phy, l_phy, u_phy);
                        //ROS_WARN("test!, tau:%f, best_tau:%f", tau.norm(), v_best_tau_.norm());
                      }
                    /* 3. update */
                    if(statics_validity && tau.norm() < v_best_tau_.norm())
                      {
                        /* find better grasp pose */
                        v_best_theta_ = v_theta;
                        v_best_phy_ = v_phy;
                        v_best_contact_d_ = v_contact_d;
                        v_best_contact_p_ = v_contact_p;
                        v_best_joint_p_ = v_joint_p;

                        v_best_tau_ = tau;
                        v_best_f_fc_ = min_f_fc;

                        best_base_side_ = i;

                        //debug
                        // ROS_INFO("update!");
                        // showResult();
                      }

                    if(file_log_flag_)
                      {
                        if(statics_validity)
                          log_ofs_ << i + 1 << "\t" << d  << "\t" << phy << "\t" << tau.norm() << "\t" << min_f_fc.transpose() * H_ * min_f_fc  << std::endl;
                        else if(kinematics_validity)
                          log_ofs_ << i + 1 << "\t" << d  << "\t" << phy << "\t" << -1 << "\t" << 0  << std::endl;
                        else
                          log_ofs_ << i + 1 << "\t" << d  << "\t" << phy << "\t" << -1 << "\t" << -1  << std::endl;
                      }
                  }
              }
            if(one_side_flag_) break;
          }
      }
    else if(object_type_ == CYLINDER)
      {
        float u_phy = M_PI / 2  - acos((link_length_ / 2) / (link_radius_ + cylinder_radius_));
        float l_phy = - u_phy;

        for(float phy = l_phy; phy <= u_phy; phy += res_phy_)
          {
            VectorXd min_f_fc = VectorXd::Zero(contact_num_ * 3);
            VectorXd tau = VectorXd::Zero(contact_num_ -1 );

            bool kinematics_validity = true;
            bool statics_validity = true;

            /* 1. calculate the joint angles */
            if(!circleBasedJointsCalc(phy, v_theta, v_phy, v_contact_p, v_joint_p))
              kinematics_validity = false;
            /* 2. calculate the joint angles */
            if(kinematics_validity)
              {
                if(!linkStatics(v_contact_p, v_joint_p, tau, min_f_fc))
                  statics_validity = false;
              }
            else
              statics_validity = false;

            /* 2.5 show the result of the validity */
            if(!kinematics_validity)
              ROS_WARN("phy:%f, l_phy:%f, u_phy:%f, kinematics invalid", phy, l_phy, u_phy);
            if(kinematics_validity && !statics_validity)
              ROS_ERROR("phy:%f, l_phy:%f, u_phy:%f, statics invalid", phy, l_phy, u_phy);
            if(kinematics_validity && statics_validity)
              {
                ROS_INFO("phy:%f, l_phy:%f, u_phy:%f, valid state", phy, l_phy, u_phy);
                //ROS_WARN("test!, tau:%f, best_tau:%f", tau.norm(), v_best_tau_.norm());
              }

            /* 3. update */
            if(statics_validity && tau.norm() < v_best_tau_.norm())
              {
                /* find better grasp pose */
                v_best_theta_ = v_theta;
                v_best_phy_ = v_phy;
                v_best_contact_p_ = v_contact_p;
                v_best_joint_p_ = v_joint_p;
                v_best_tau_ = tau;
                v_best_f_fc_ = min_f_fc;
                v_orig_psi_ = v_psi_; // different from convex type, the orig cotains the best result
              }

            if(file_log_flag_)
              {
                if(statics_validity)
                  log_ofs_ << phy << " " << tau.norm() << " " << min_f_fc.transpose() * H_ * min_f_fc  << std::endl;
                else if(kinematics_validity)
                  log_ofs_ << phy << " " << -1 << " " << 0  << std::endl;
                else
                  log_ofs_ << phy << " " << -1 << " " << -1  << std::endl;
              }
          }
      }
    return true;
  }

};

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(grasp_planning::FullSearch, grasp_planning::Base);

