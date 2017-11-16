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

#include <aerial_transportation/grasp_form_searching/grasp_form_search.h>

using namespace Eigen;

namespace grasp_form_search
{
  GraspFormSearch::GraspFormSearch(ros::NodeHandle nh, ros::NodeHandle nhp): nh_(nh), nhp_(nhp)
  {
    reset();
    uav_kinematics_ = boost::shared_ptr<TransformController>(new TransformController(nh_, nhp_, false));

    /* initialize the multilink(hydrus) kinematics */
    kinematicsInit();
    visualizeInit();

    nhp_.param("verbose", verbose_, false);
    nhp_.param("test_grasp_flag", test_grasp_flag_, false);
    if(verbose_) std::cout << "[grasp form search] test_grasp_flag: " << test_grasp_flag_ << std::endl;
    nhp_.param("link_length", link_length_, 0.0);
    if(verbose_) std::cout << "[grasp form search] link_length: " << link_length_ << std::endl;
    nhp_.param("duct_radius", duct_radius_, 0.0);
    if(verbose_) std::cout << "[grasp form search] duct_radius: " << duct_radius_ << std::endl;

    std::string srv_name;
    nhp_.param("object_configuration_srv_name", srv_name, std::string("/target_object_configuration"));
    object_configuration_srv_ = nh_.advertiseService(srv_name, &GraspFormSearch::graspFormSeachingReqeust, this);

    try
      {
        search_algorithm_loader_ptr_ =  boost::shared_ptr< pluginlib::ClassLoader<grasp_form_search::SearchAlgorithmBase> >( new pluginlib::ClassLoader<grasp_form_search::SearchAlgorithmBase>("aerial_transportation", "grasp_form_search::SearchAlgorithmBase"));
        std::string search_algorithm_plugin_name;
        nhp_.param ("search_algorithm_plugin_name", search_algorithm_plugin_name, std::string("search_algorithm_plugin/full_search"));
        if(verbose_) std::cout << "[Grasp Form Search] search_algorithm_plugin_name: " << search_algorithm_plugin_name << std::endl;
        search_solver_ = search_algorithm_loader_ptr_->createInstance(search_algorithm_plugin_name);
        search_solver_->initialize(nh_, nhp_, this);
      }
    catch(pluginlib::PluginlibException& ex)
      {
        ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
      }
    double func_loop_rate;
    nhp_.param("func_loop_rate", func_loop_rate, 10.0);

    /* timer init */
    func_timer_ = nhp_.createTimer(ros::Duration(1.0 / func_loop_rate), &GraspFormSearch::mainFunc,this);

  }

  void GraspFormSearch::mainFunc(const ros::TimerEvent & e)
  {
    if(object_type_ == aerial_transportation::ObjectConfigure::Request::UNKNOWN) return;

    if(!search_flag_)
      {
        /* test the jointCalc and linkStatics method */
        if(test_grasp_flag_) graspKinematicsStaticsTest();
        /* search the grasp pose */
        else search_flag_ = search_solver_->getSolution();

        if(!search_flag_)
          {
            ROS_WARN("[grasp_form_search]: can not find solution");
            object_type_ = aerial_transportation::ObjectConfigure::Request::UNKNOWN;
            return;
          }

        resultRecord2File();
      }

    showGraspResult();
  }

  void GraspFormSearch::reset()
  {
    object_type_ = aerial_transportation::ObjectConfigure::Request::UNKNOWN;
    object_inertia_ = geometry_msgs::Inertia();

    best_start_side_ = 0;
    search_flag_ = false;
    object_radius_ = 0;
    contact_num_ = 0;
    object_.clear();

    /* result */
    v_best_min_f_fc_ = VectorXd::Constant(1, 0);
    v_best_tau_ = VectorXd::Constant(1, 0);
    v_best_hover_thrust_ = VectorXd::Constant(1, 0);

    v_best_theta_.clear();
    v_best_delta_.clear();
    v_best_contact_d_.clear();
    v_best_contact_p_.clear();
    v_best_contact_rot_.clear();
    v_best_joint_p_.clear();
    v_valid_lower_bound_theta_.clear();
    v_valid_lower_bound_theta_.clear();
    v_valid_lower_bound_contact_d_.clear();
    v_valid_upper_bound_theta_.clear();
    v_valid_upper_bound_delta_.clear();
    v_valid_upper_bound_contact_d_.clear();
    assert(object_.size() == 0); //check

  }

  bool GraspFormSearch::graspFormSeachingReqeust(aerial_transportation::ObjectConfigure::Request& req, aerial_transportation::ObjectConfigure::Response& res)
  {
    if(req.vertices.size() == 0)
      {
        res.status = false;
        res.message = std::string("no vertices information");
        return false;
      }
    std::vector<Vector3d> vertices(0);
    for(int i = 0; i < req.vertices.size(); i++)
      vertices.push_back(Vector3d(req.vertices[i].x, req.vertices[i].y, req.vertices[i].z));

    if(!objectConfiguration(req.object_type, req.inertia, vertices))
      {
        res.status = false;
        res.message = std::string("bad object configuration");
        return false;
      }

      res.status = true;
      res.message = std::string("succeed to set object configuration");

      return true;
  }

  bool GraspFormSearch::objectConfiguration(int8_t object_type, geometry_msgs::Inertia object_inertia, std::vector<Vector3d> vertices)
  {
    /* clear the last data */
    reset();

    if(uav_kinematics_->getRotorNum() == 0)
      {
        ROS_WARN("[objectConfiguration]: uav kinematics is not set, check whether robot_description is set correctly");
        return false;
      }

    if(object_type == aerial_transportation::ObjectConfigure::Request::UNKNOWN)
      {
        ROS_WARN("[objectConfiguration]: no object type is assigned");
        return false;
      }

    if(object_inertia.m <= 0 )
      {
        ROS_WARN("[objectConfiguration]: object mass is invalid, %f", object_inertia.m);
        return false;
      }

    Vector3d object_cog;
    tf::vectorMsgToEigen(object_inertia.com, object_cog);
    if(object_cog != VectorXd::Zero(3))
      ROS_WARN("[objectConfiguration]: object cog is not zero: [%f, %f, %f,]", object_cog(0), object_cog(1), object_cog(2));

    object_type_ = object_type;
    object_inertia_ = object_inertia;

    /* special configuration for different type of object */
    switch(object_type_)
      {
      case aerial_transportation::ObjectConfigure::Request::CONVEX_POLYGONAL_COLUMN:
        {
          float abs_psi = 0;
          Vector3d object_cog_tmp(0, 0, 0);
          /* fill the object information */

          if(vertices.size() < 3)
            {
              ROS_WARN("[objectConfiguration]: not enougn vertices for grasp, %d", (int)vertices.size());
              return false;
            }

          for(int i = 0; i < vertices.size(); i++)
            {
              int convex_prev = (i + vertices.size() -1) % vertices.size();
              int convex_current = i % vertices.size();
              int convex_next = (i + 1) % vertices.size();

              float psi = atan2(vertices[convex_next](1) -vertices[convex_current](1), vertices[convex_next](0) -vertices[convex_current](0))
                - atan2(vertices[convex_current](1) - vertices[convex_prev](1), vertices[convex_current](0) - vertices[convex_prev](0));

              if(psi < - M_PI ) psi += (2 * M_PI);
              assert(psi > 0 && psi < M_PI);

              if(i > 0) abs_psi +=  psi;

              object_.push_back(VertexHandlePtr(new VertexHandle(psi, Quaterniond(AngleAxisd(abs_psi, Vector3d::UnitZ())), vertices[i])));
              /* set the side length */
              object_[i]->len_ =  (vertices[(i + 1) % vertices.size()] - vertices[i]).norm();

              if(i < uav_kinematics_->getRotorNum()) contact_num_ ++;
              ROS_INFO("[objectConfiguration] vertex %d: [%f, %f, %f], rel psi: %f, abs psi: %f", i, vertices[i].x(), vertices[i].y(), vertices[i].z(), psi, abs_psi);

              object_cog_tmp += vertices[i];
            }
          if(object_cog == VectorXd::Zero(3)) tf::vectorEigenToMsg(object_cog_tmp / vertices.size(), object_inertia_.com);
          ROS_INFO("[objectConfiguration]:  contact num: %d, cog of object is [%f, %f, %f,]", contact_num_, object_inertia_.com.x, object_inertia_.com.y, object_inertia_.com.z);

          break;
        }
      case aerial_transportation::ObjectConfigure::Request::CYLINDER:
        {

          if(vertices.size() != 1)
            {
              ROS_WARN("[objectConfiguration] invalid radius for cylinder");
              return false;
            }

          object_radius_ = vertices[0](0) / 2;
          if(!cylinderInitEnvelopCheck(contact_num_)) return false;
          for(int i = 0; i < contact_num_; i++)
            object_.push_back(VertexHandlePtr(new VertexHandle()));

          break;
        }
      default:
        {
          object_type_ = aerial_transportation::ObjectConfigure::Request::UNKNOWN;
          ROS_WARN("invalid object type: %d", object_type);
          return false;
          break;
        }
      }


    /* init statics */
    staticsInit();
    v_best_min_f_fc_ = VectorXd::Constant(contact_num_ * 3, 1e6);
    v_best_tau_ = VectorXd::Constant(contact_num_ - 1, 1e6);

    return true;
  }

  bool GraspFormSearch::graspKinematicsStaticsTest()
  {
    float d = 0.01;
    float delta = 0;

    if(object_type_ == aerial_transportation::ObjectConfigure::Request::CONVEX_POLYGONAL_COLUMN)
      {
        //d = object_[1]->vertex_p_(0) / 2;
        ROS_INFO("test grasp planner: first contact distance: %f", d);
      }

    if(!envelopingCalc(d, delta, v_best_theta_, v_best_delta_, v_best_contact_d_, v_best_contact_p_, v_best_contact_rot_, v_best_joint_p_, 0))
      {
        ROS_WARN("[grasp kineatics statics test]: can not get valid enveloping result");
        return false;
      }


    if(!graspForceClosure(v_best_contact_p_, v_best_contact_rot_, v_best_joint_p_, v_best_tau_, v_best_min_f_fc_))
      {
        ROS_WARN("[grasp kineatics statics test]: can not get valid force closure");
        return  false;
      }

    tf::Transform tf_object_origin_to_uav_root;
    tf::Vector3 uav_root_origin;
    tf::vectorEigenToTF(v_best_joint_p_[0], uav_root_origin);
    tf_object_origin_to_uav_root.setOrigin(uav_root_origin);
    tf::Quaternion uav_root_q;
    tf::quaternionEigenToTF(object_[0]->contact_rot_ * AngleAxisd(delta, Vector3d::UnitZ()), uav_root_q);
    tf_object_origin_to_uav_root.setRotation(uav_root_q);
    if(!hoveringStatics(v_best_theta_, tf_object_origin_to_uav_root.inverse(), v_best_hover_thrust_))
      {
        ROS_WARN("[grasp kineatics statics test]: can not get valid hovering thrust");
        return false;
      }

    ROS_WARN("[grasp kineatics statics test]: thet result is: ");
    std::cout << "tau: [" << v_best_tau_.transpose() << "]" << std::endl;
    std::cout << " min f fc: [" << v_best_min_f_fc_.transpose() << "]"  << std::endl;

    v_best_hover_thrust_ = uav_kinematics_->getStableState();
    std::cout << " hovering thrust: " << v_best_hover_thrust_.transpose() << std::endl;
    std::cout << " min hovering thrust: " << v_best_hover_thrust_.minCoeff() << std::endl;
    std::cout << " max hovering thrust: " << v_best_hover_thrust_.maxCoeff() << std::endl;

    search_flag_ = true;
    return true;
  }

};


