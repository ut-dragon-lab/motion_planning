#ifndef GRASP_PLANNING_H
#define GRASP_PLANNING_H

/* ros */
#include <ros/ros.h>
#include <geometry_msgs/PolygonStamped.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

#include <boost/shared_ptr.hpp>
#include <vector>

/* random */
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real_distribution.hpp>

/* for QP solution for force-closure */
#include <qpOASES.hpp>
USING_NAMESPACE_QPOASES

/* for eigen cumputation */
#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Dense>
#include <Eigen/Geometry> 
#include <Eigen/Eigenvalues>

// file
#include <iostream>
#include <sstream>
#include <fstream>


using namespace Eigen;
USING_NAMESPACE_QPOASES

namespace grasp_planning
{
  /* the kinematics and Dynamics of multi-link aerial robot */
  class Base
  {
  public:
    virtual void initialize(ros::NodeHandle nh, ros::NodeHandle nhp)  = 0;

    virtual ~Base() { }

    const static uint8_t CONVEX_POLYGONAL_COLUMN  = 0;
    const static uint8_t CYLINDER = 1;

    void getObjectApproachOffset(std::vector<float> v_theta, double& object_approach_offset_x, double& object_approach_offset_y, double& object_approach_offset_yaw);

    void getObjectGraspAngles(float tighten_delta_angle, float approach_delta_angle, int& contact_num, std::vector<float>& v_hold_angle, std::vector<float>& v_tighten_angle, std::vector<float>& v_approach_angle);

  protected:
    /* ros node */
    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;

    /* main thread */
    ros::Timer  func_timer_;

    /* ros publisher & subscirber */
    ros::Subscriber convex_polygonal_column_info_sub_; /* the geometric information of convex_polygon_object */
    ros::Subscriber cylinder_info_sub_; /* the geometric information of cylinder_object */
    ros::Publisher joint_states_pub_; /* final result */
    tf::TransformBroadcaster br_;

    /* rosparam based variables */
    bool debug_;
    bool test_grasp_flag_; /* active the graspPlanningTest() */
    bool file_log_flag_; /* record the search process */
    bool file_result_flag_; /* record the search process */
    bool play_file_flag_; /*  get result from result file */
    bool control_test_flag_; /*  check the control part */
    std::string convex_polygonal_column_info_sub_name_;
    std::string cylinder_info_sub_name_;
    std::string joint_states_pub_name_;
    std::string link1_frame_name_;
    std::string object_frame_name_;
    std::string planning_file_name_;
    double func_loop_rate_;

    double joint_angle_limit_; /* the limit of joint angle */
    double link_length_; /* the length of link */
    double link_radius_; /* the radius of propeller protector */
    int link_num_; /* the number of link */

    double object_mass_; /* the mass of object */
    double fric_z_mu_; /* the rate of vertical firction */
    double fric_x_mu_; /* the rate of horizontal firction */

    /* base variable */
    std::ofstream log_ofs_; /* for record to a file */
    int planning_flag_; /* to realzie onetime receive of object_info_sub_ */

    /* base config for convex polygon and cylinder: common */
    int object_type_;

    /* convex polygon: the original vector of angles of vertex of object, 0: the relative angle based on the previous side, 1: the absolute angle based on the first side */
    std::vector<Vector2d> v_orig_psi_; 
    /* convex polygon: the original vector of position of each vertex */
    /* convex polygon: the vector of angles of vertex of object, 0: the relative angle of the nextbased on the previous side, 1: the absolute til angle on the current side based on the first side */
    /* cylinder:  0: the angle between the lines of neighbour contact points to centers, 1: the absolute direction of norm of the contact point */
    std::vector<Vector3d> v_orig_vertex_p_; 
    std::vector<Vector2d> v_psi_; 
    std::vector<Vector3d> v_vertex_p_; /* convex polygon: the vector of position of each vertex, z: side length */
    std::vector<float> v_side_length_; /* convex polygon: the vector of side length */
    int side_num_; /* the number of side */
    float cylinder_radius_; /* cylinder: the radius of cylinder */

    /* the grasp config */
    int contact_num_; /* the number of polygon side */
    int best_base_side_; /* the funal result of best based side for convex polygon object */
    VectorXd v_best_tau_; /* final result of best tau */
    VectorXd v_best_f_fc_; /* final result of best contact force */
    std::vector<Vector2d> v_best_theta_; /* final result of best joint angles, 0: joint angle, 1: link direction in terms of base side */

    std::vector<float> v_best_phy_; /* the vector of tilt angles of between each link and related side */
    std::vector<float> v_best_contact_d_; /* the vector of position of contact point at each side from the previous vertex */
    std::vector<Vector3d> v_best_contact_p_; /* the vector of position of contact point at each side */
    std::vector<Vector3d> v_best_joint_p_; /* the vector of position of joint point at each side */

    Vector3d cog_object_;
    VectorXd tau_;
    VectorXd min_f_fc_;

    //MatrixXd min_f_fc_; /* the minimum force based on the qp problem of force-closure

    /* the QP solver variables */
    boost::shared_ptr<SQProblem> fc_solver_;
    bool qp_init_flag_;
    MatrixXd H_;
    MatrixXd A_;
    MatrixXd g_;
    VectorXd lb_;
    VectorXd ub_;
    VectorXd lA_;
    VectorXd uA_;
    int n_wsr_;
    Options qp_options_;

    /* the grasp approaching control variables */
    int approach_base_link_; // the link which has imu and mocap(position recognized link)
    double approach_pos_weight_rate_; //weight for cog calculation
    double approach_angle_weight_rate_; //weight for approach pose
    double object_approach_offset_x_, object_approach_offset_y_, object_approach_offset_yaw_;

    /* base function */
    void baseInit(ros::NodeHandle nh, ros::NodeHandle nhp);
    void qpInit();
    void baseRosParamInit();
    void mainFunc(const ros::TimerEvent & e);
    bool linkStatics(std::vector<Vector3d> v_contact_p, std::vector<Vector3d> v_joint_p, VectorXd& tau, VectorXd& min_f_fc);
    bool convexPolygonBasedJointsCalc(double first_contact_d, double first_phy, std::vector<Vector2d>& v_theta, std::vector<float>& v_phy, std::vector<float>& v_contact_d, std::vector<Vector3d>& v_contact_p, std::vector<Vector3d>& v_joint_p); /* to calculate the enveloping grasp joints angles */
    bool circleBasedJointsCalc(double first_phy, std::vector<Vector2d>& v_theta, std::vector<float>& v_phy, std::vector<Vector3d>& v_contact_p, std::vector<Vector3d>& v_joint_p);

    void getResultFromFile();
    void resultRecord2File();
    void showResult();

    /*************************************************
      note that, the origin of coordinate of
       1) the convex polygon is the first vertex, and the x axis direction is the first side.
       2) the clydiner is the center of the cylinder, and the y axis is the opposite of the direction from center to the first contact point.
    **************************************************/
    bool convexPolygonalColumnCalc(geometry_msgs::PolygonStamped); /* to calculate the geometric config of convex poylgon object */
    void convexPolygonalColumnVertexConvert(int base_side = 0); /* to convert the coordinate based on the No.i side */
    bool cylinderCalc(); /* to calculate the geometric config of cylinder object */

    virtual bool graspPlanning() = 0; /* the special planning algorithm */
    bool graspPlanningTest();
    virtual void rosParamInit() = 0;

    void convexPolygonalColumnInfoCallback(const geometry_msgs::PolygonStampedConstPtr & msg);
    void cylinderInfoCallback(const visualization_msgs::MarkerConstPtr & msg);

    /* tools */
    inline float getLinkLength() {return link_radius_;}
    inline float getLinkRadius() {return link_radius_;}
    inline int getLinkNum() {return link_radius_;}
    inline int getObjectType() {return object_type_;}
    inline int getSideNum() {return side_num_;}
    inline float getCylinderRadius() {return cylinder_radius_;}
    inline int getContactNum() {return contact_num_;}
    inline int getBestBaseSide() {return best_base_side_;}
    inline Vector3d getCogObject() {return cog_object_;}

    inline std::vector<Vector2d> getOrigPsi() {return v_orig_psi_;}
    inline std::vector<Vector3d> getOrigVertexP() {return v_orig_vertex_p_;}
    inline std::vector<Vector2d> getPsi() {return v_psi_;}
    inline std::vector<Vector3d> getVertexP() {return v_vertex_p_;}
    inline std::vector<float> getPhy() {return v_best_phy_;}
    inline std::vector<Vector2d> getTheta() {return v_best_theta_;}
    inline MatrixXd getTau() {return v_best_tau_;}
    inline std::vector<Vector3d> getContactP() {return v_best_contact_p_;}

    Matrix3d skew(Eigen::Vector3d vec)
    {
       Matrix3d skew;
       skew << 0.0, -vec(2), vec(1),
         vec(2), 0.0, -vec(0),
         -vec(1), vec(0), 0.0;
       return skew;
    }
  };
};

#endif  // GRASP_PLANNING_H
