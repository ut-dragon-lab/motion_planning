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

#ifndef GRASP_FORM_SEARCH_H
#define GRASP_FORM_SEARCH_H

/* ros */
#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <tf/LinearMath/Transform.h>
#include <aerial_transportation/ObjectConfigure.h>

/* hydrus aerial robot kinematics */
#include <hydrus/transform_control.h>

/* search algorithm */
#include <aerial_transportation/grasp_form_searching/search_algorithm/search_algorithm_base.h>

using namespace Eigen;

namespace grasp_form_search
{
  class SearchAlgorithmBase;

  class VertexHandle
  {
  public:
    VertexHandle(double psi = 0, Quaterniond contact_rot = Quaterniond(1, 0, 0, 0),
                 Vector3d vertex_p = Vector3d(0, 0, 0)):
      psi_(psi), vertex_p_(vertex_p), contact_rot_(contact_rot), len_(0)
    {
    }
    ~VertexHandle(){}
    double psi_;
    /* convex polygon: the angle of right (next) vertex of object   */
    /* cylinder: the angle between the lines of neighbour contact points to centers */

    Vector3d vertex_p_; /* convex polygon: the orientation of the side based on the first side */
    Quaterniond contact_rot_; /* the orientation of contact point coord based on the first side */
    double len_; /* convex polygon: side length; cylinder: radius */
  };
  typedef boost::shared_ptr<grasp_form_search::VertexHandle> VertexHandlePtr;

  class GraspFormSearch
  {
  public:
    //GraspFormSearch(){
    GraspFormSearch(ros::NodeHandle nh, ros::NodeHandle nhp);
    ~GraspFormSearch(){}

    /* search */
    bool graspFormSeachingReqeust(aerial_transportation::ObjectConfigure::Request& req, aerial_transportation::ObjectConfigure::Response& res);
    bool objectConfiguration(int8_t object_type, geometry_msgs::Inertia object_inertia, std::vector<Vector3d> vertices); /* to calculate the geometric config of cylinder object */
    void reset();
    bool graspKinematicsStaticsTest();
    void mainFunc(const ros::TimerEvent & e);

    /* kinematics */
    void kinematicsInit();
    bool cylinderInitEnvelopCheck(int& contact_num);
    bool convexPolygonNeighbourContacts(double curr_contact_d, double curr_delta, double curr_psi, double curr_side_len, double next_side_len,  double& theta /* joint_angle */, double& next_delta, Vector3d& joint_p, double& next_contact_d);
    bool envelopingCalc(double first_contact_d, double first_delta, std::vector<double>& v_theta, std::vector<double>& v_delta, std::vector<double>& v_contact_d, std::vector<Vector3d>& v_contact_p, std::vector<Quaterniond>& v_contact_rot, std::vector<Vector3d>& v_joint_p, int start_side /* convex */);

    /* statics */
    void staticsInit();
    bool graspForceClosure(std::vector<Vector3d> v_contact_p, std::vector<Quaterniond> v_contact_rot, std::vector<Vector3d> v_joint_p, VectorXd& tau, VectorXd& min_f_fc);
    bool hoveringStatics(std::vector<double>& v_theta, tf::Transform tf_uav_root_to_object_origin, VectorXd& hover_thrust);
    /* visualize */
    void visualizeInit();
    void getResultFromFile();
    void resultRecord2File();
    void showGraspResult();

    /* tools */
    inline boost::shared_ptr<TransformController> getUavKinematics() {return uav_kinematics_;}
    inline std::vector<VertexHandlePtr> getObjectInfo() {return object_;}
    inline int getObjectType() {return object_type_;}
    inline int getSideNum() {return object_.size();}
    inline int getContactNum() {return contact_num_;}
    inline int getBestStartSide() {return best_start_side_;}
    inline geometry_msgs::Inertia getObjectInertia() {return object_inertia_;}
    inline double getLinkLength() {return link_length_;}
    inline double getDuctRadius() {return duct_radius_;}
    inline double getObjectRadius() {return object_radius_;}

    inline void setBestStartSide(int best_start_side){best_start_side_ = best_start_side; }
    void setBestForceClosureResult(std::vector<double> v_best_theta, std::vector<double> v_valid_lower_bound_theta, std::vector<double> v_valid_upper_bound_theta, std::vector<double> v_best_delta, std::vector<double> v_valid_lower_bound_delta, std::vector<double> v_valid_upper_bound_delta, std::vector<Vector3d> v_best_contact_p, std::vector<Quaterniond> v_best_contact_rot, std::vector<Vector3d> v_best_joint_p, VectorXd v_best_min_f_fc, VectorXd v_best_tau, VectorXd v_best_hover_thrust)
    {
      v_best_theta_ = v_best_theta;
      v_valid_lower_bound_theta_ = v_valid_lower_bound_theta;
      v_valid_upper_bound_theta_ = v_valid_upper_bound_theta;
      v_best_delta_ = v_best_delta;
      v_valid_lower_bound_delta_ = v_valid_lower_bound_delta;
      v_valid_upper_bound_delta_ = v_valid_upper_bound_delta;
      v_best_contact_p_ = v_best_contact_p;
      v_best_contact_rot_ = v_best_contact_rot;
      v_best_joint_p_ = v_best_joint_p;
      v_best_min_f_fc_ = v_best_min_f_fc;
      v_best_tau_ = v_best_tau;
      v_best_hover_thrust_ = v_best_hover_thrust;
    }
    void setBestContactDVector(std::vector<double> v_best_contact_d, std::vector<double> v_valid_lower_bound_contact_d, std::vector<double> v_valid_upper_bound_contact_d)
    {
      v_best_contact_d_ = v_best_contact_d;
      v_valid_lower_bound_contact_d_ = v_valid_lower_bound_contact_d;
      v_valid_upper_bound_contact_d_ = v_valid_upper_bound_contact_d;
    }

  protected:
    /* ros node */
    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;

    /* visualize */
    ros::Publisher joint_states_pub_; /* CAUTION: can not write in the non-name space */
    tf::TransformBroadcaster br_; /* CAUTION: can not write in the non-name space */

    /* ros publisher & subscirber */
    ros::ServiceServer object_configuration_srv_; /* the geometric information of  target object */

    /* main routine */
    ros::Timer  func_timer_;
    boost::shared_ptr<TransformController> uav_kinematics_;
    double link_length_ = 0; // TODO: not good
    double duct_radius_ = 0; // TODO: not good

    /* rosparam based variables */
    bool verbose_;

    /* base variable */
    bool search_flag_; /* to realzie onetime receive of object_info_sub_ */
    bool test_grasp_flag_; /* to realzie onetime receive of object_info_sub_ */

    /* object */
    std::vector<VertexHandlePtr> object_;
    geometry_msgs::Inertia object_inertia_;
    int object_type_;

    /* the search general result */
    int contact_num_;
    VectorXd v_best_hover_thrust_;
    VectorXd v_best_tau_; /* final result of best tau */
    VectorXd v_best_min_f_fc_; /* final result of best contact force */
    std::vector<double> v_best_theta_; /* final result of best joint angle */
    std::vector<double> v_best_delta_; /* the vector of tilt angles of between each link and related side */
    std::vector<double> v_valid_lower_bound_theta_; /* final result of valid_lower_bound_ joint angle */
    std::vector<double> v_valid_lower_bound_delta_; /* the vector of tilt angles of between each link and related side */
    std::vector<double> v_valid_upper_bound_theta_; /* final result of valid_upper_bound_ joint angle */
    std::vector<double> v_valid_upper_bound_delta_; /* the vector of tilt angles of between each link and related side */

    std::vector<double> v_best_contact_d_; /* the vector of position of contact point at each side from the previous vertex */
    std::vector<double> v_valid_lower_bound_contact_d_; /* the vector of contact dist from vertex to next vertex */
    std::vector<double> v_valid_upper_bound_contact_d_; /* the vector of contact dist from vertex to next vertex */

    std::vector<Vector3d> v_best_contact_p_; /* the vector of position of contact point at each side */
    std::vector<Quaterniond> v_best_contact_rot_; /* the vector of rotation of contact point at each side */
    std::vector<Vector3d> v_best_joint_p_; /* the vector of position of joint point at each side */

    /* special for convex/ cylinder */
    int best_start_side_; /* the funal result of best based side for convex polygon object */
    double object_radius_;

    /* search algorithm */
    boost::shared_ptr<grasp_form_search::SearchAlgorithmBase> search_solver_;
    boost::shared_ptr< pluginlib::ClassLoader<grasp_form_search::SearchAlgorithmBase> > search_algorithm_loader_ptr_;
  };
};

#endif
