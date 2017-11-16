#include <aerial_transportation/grasp_form_searching/grasp_form_search.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

/* file */
#include <iostream>
#include <sstream>
#include <fstream>

using namespace Eigen;

namespace grasp_form_search
{
  namespace
  {
    bool visualize_verbose_;
    bool file_result_flag_; /* record the search process */
    bool play_file_flag_; /*  get result from result file */

    std::string joint_states_pub_name_;
    std::string link1_frame_name_;
    std::string object_frame_name_;
    std::string searching_file_name_;
  }

  void GraspFormSearch::visualizeInit()
  {
    nhp_.param("visualize_verbose", visualize_verbose_, false);
    if(verbose_) std::cout << "[visualize] verbose: " << visualize_verbose_ << std::endl;
    nhp_.param("play_file_flag", play_file_flag_, false);
    if(verbose_) std::cout << "[visualize]play_file_flag: " << play_file_flag_ << std::endl;
    nhp_.param("file_result_flag", file_result_flag_, false);
    if(verbose_) std::cout << "[visualize]file_result_flag: " << file_result_flag_ << std::endl;

    nhp_.param("searching_file_name", searching_file_name_, std::string("searching_log.txt"));
    if(verbose_) std::cout << "[visualize]searching file name: " << searching_file_name_ << std::endl;

    nhp_.param("joint_states_pub_name", joint_states_pub_name_, std::string("/searching/joint_states"));
    /* tf */
    nhp_.param("link1_frame_name", link1_frame_name_, std::string("link1")); //should not be here!!
    nhp_.param("object_frame_name", object_frame_name_, std::string("object")); //should not be here!!

    if(play_file_flag_) getResultFromFile();
    /* pub & sub */
    joint_states_pub_ = nh_.advertise<sensor_msgs::JointState>(joint_states_pub_name_, 1);

  }

  void GraspFormSearch::resultRecord2File()
  {
    std::cout << "Searching Result: " << std::endl;

    std::cout << "v_best_delta: [ ";
    for(int j = 0; j < v_best_delta_.size() ; j ++) std::cout << v_best_delta_[j] << " ";
    std::cout << "]" << std::endl;
    std::cout << "v_valid_lower_bound_delta: [ ";
    for(int j = 0; j < v_valid_lower_bound_delta_.size() ; j ++) std::cout << v_valid_lower_bound_delta_[j] << " ";
    std::cout << "]" << std::endl;
    std::cout << "v_valid_upper_bound_delta: [ ";
    for(int j = 0; j < v_valid_upper_bound_delta_.size() ; j ++) std::cout << v_valid_upper_bound_delta_[j] << " ";
    std::cout << "]" << std::endl;

    if(object_type_ == aerial_transportation::ObjectConfigure::Request::CONVEX_POLYGONAL_COLUMN)
      {
        std::cout << "best start side:" << best_start_side_ << std::endl;
        std::cout << "v_best_contact_d: [ ";
        for(int j = 0; j < v_best_contact_d_.size() ; j ++) std::cout << v_best_contact_d_[j] << " ";
        std::cout << "]" << std::endl;
        std::cout << "v_valid_lower_bound_contact_d: [ ";
        for(int j = 0; j < v_valid_lower_bound_contact_d_.size() ; j ++) std::cout << v_valid_lower_bound_contact_d_[j] << " ";
        std::cout << "]" << std::endl;

        std::cout << "v_valid_upper_bound_contact_d: [ ";
        for(int j = 0; j < v_valid_upper_bound_contact_d_.size() ; j ++) std::cout << v_valid_upper_bound_contact_d_[j] << " ";
        std::cout << "]" << std::endl;
      }

    std::cout << "v_best_theta: [ ";
    for(int j = 0; j < v_best_theta_.size() ; j ++) std::cout << v_best_theta_[j] << " ";
    std::cout << "]" << std::endl;

    std::cout << "v_best_tau: [ ";
    for(int j = 0; j < v_best_tau_.size() ; j ++) std::cout << v_best_tau_[j] << " ";
    std::cout << "]" << std::endl;
    std::cout << "v_best_min_f_fc: [ ";
    for(int j = 0; j < v_best_min_f_fc_.size() ; j ++) std::cout << v_best_min_f_fc_[j] << " ";
    std::cout << "]" <<std::endl;

    std::cout << "v_best_hover_thrust: [ ";
    for(int j = 0; j < v_best_hover_thrust_.size() ; j ++) std::cout << v_best_hover_thrust_[j] << " ";
    std::cout << "]" <<std::endl;

    if(!file_result_flag_) return;

    std::ofstream ofs; /* for record to a file */
    ofs.open("searching_result.txt");
    ofs << "object_type: " << (int)object_type_  << std::endl;
    /* only necessary for convex polygon object */
    if(object_type_ == aerial_transportation::ObjectConfigure::Request::CONVEX_POLYGONAL_COLUMN)
      {
        ofs << "side_num: " << object_.size()  << std::endl;
        ofs << "best_start_side: " << best_start_side_ << std::endl;

        ofs << "v_vertex_info: " << std::endl;
        for(int i = 0; i < object_.size(); i++)
          ofs << object_[i]->vertex_p_(0) << " " << object_[i]->vertex_p_(1)  << " " << object_[i]->psi_ << " " <<  object_[i]->len_ << " " <<  object_[i]->contact_rot_.x() <<  " " <<  object_[i]->contact_rot_.y() << " " <<  object_[i]->contact_rot_.z() << " " <<  object_[i]->contact_rot_.w() << std::endl;
      }

    else if(object_type_ == aerial_transportation::ObjectConfigure::Request::CYLINDER)
      ofs << "object_radius: " << object_radius_  << std::endl;

    ofs << "contact_num: " << contact_num_  << std::endl;

    ofs << "v_best_contact_p: " << std::endl;
    for(int i = 0; i < contact_num_; i++)
      ofs << v_best_contact_p_[i](0) << " " << v_best_contact_p_[i](1) << std::endl;

    ofs << "v_best_delta: " << std::endl;
    for(int i = 0; i < contact_num_; i++) ofs << v_best_delta_[i] << std::endl;
    ofs << "v_valid_lower_bound_delta: " << std::endl;
    for(int i = 0; i < contact_num_; i++) ofs << v_valid_lower_bound_delta_[i] << std::endl;
    ofs << "v_valid_upper_bound_delta: " << std::endl;
    for(int i = 0; i < contact_num_; i++) ofs << v_valid_upper_bound_delta_[i] << std::endl;
    if(object_type_ == aerial_transportation::ObjectConfigure::Request::CONVEX_POLYGONAL_COLUMN)
      {
        ofs << "v_best_contact_d: " << std::endl;
        for(int i = 0; i < contact_num_; i++) ofs << v_best_contact_d_[i] << std::endl;
        ofs << "v_valid_lower_bound_contact_d: " << std::endl;
        for(int i = 0; i < contact_num_; i++) ofs << v_valid_lower_bound_contact_d_[i] << std::endl;
        ofs << "v_valid_upper_bound_contact_d: " << std::endl;
        for(int i = 0; i < contact_num_; i++) ofs << v_valid_upper_bound_contact_d_[i] << std::endl;
      }

    ofs << "v_best_theta: " << std::endl;
    for(int i = 0; i < contact_num_ - 1 ; i++) ofs << v_best_theta_[i] << " " << std::endl;
    ofs << "v_valid_lower_bound_theta: " << std::endl;
    for(int i = 0; i < contact_num_ - 1 ; i++) ofs << v_valid_lower_bound_theta_[i] << " " << std::endl;
    ofs << "v_valid_upper_bound_theta: " << std::endl;
    for(int i = 0; i < contact_num_ - 1 ; i++) ofs << v_valid_upper_bound_theta_[i] << " " << std::endl;

    ofs << "v_best_tau: " << std::endl;
    for(int i = 0; i < v_best_tau_.size(); i++) ofs << v_best_tau_(i) << std::endl;

    ofs << "v_best_hover_thrust: " << std::endl;
    for(int i = 0; i < v_best_hover_thrust_.size(); i++) ofs << v_best_hover_thrust_(i) << std::endl;

    ofs << "v_best_joint_p: " << std::endl;
    for(int i = 0; i < contact_num_; i++)
      ofs << v_best_joint_p_[i](0) << " " << v_best_joint_p_[i](1) << std::endl;
  }

  void GraspFormSearch::getResultFromFile()
  {
    std::ifstream ifs(searching_file_name_.c_str());

    if(ifs.fail())
      {
        ROS_ERROR("File do not exist");
        return;
      }

    reset();

    std::stringstream ss[18];
    std::string str;
    std::string header;

    std::getline(ifs, str);
    ss[0].str(str);
    ss[0] >> header >> object_type_;
    ROS_INFO("getResultFromFile: %s: %d", header.c_str(), object_type_);


    if(object_type_ == aerial_transportation::ObjectConfigure::Request::CONVEX_POLYGONAL_COLUMN)
      {
        int side_num;
        std::getline(ifs, str);
        ss[1].str(str);
        ss[1] >> header >> side_num;
        ROS_INFO("getResultFromFile: %s: %d", header.c_str(), side_num);

        std::getline(ifs, str);
        ss[2].str(str);
        ss[2] >> header >> best_start_side_;
        ROS_INFO("getResultFromFile: %s: %d", header.c_str(), best_start_side_);

        std::getline(ifs, str);
        ss[3].str(str);
        ss[3] >> header;
        ROS_INFO("getResultFromFile: %s:", header.c_str());
        for(int i = 0; i < side_num; i++)
          {
            Vector3d vertex_p(0, 0, 0);
            Quaterniond contact_rot;
            double psi, len;
            std::stringstream ss_tmp;
            std::getline(ifs, str);
            ss_tmp.str(str);
            ss_tmp >> vertex_p(0) >> vertex_p(1) >> psi >> len
                   >> contact_rot.x() >> contact_rot.y() >> contact_rot.z() >> contact_rot.w();

            ROS_INFO("vertex%d, p: [%f, %f], rot: [%f, %f, %f, %f], psi: %f, len: %f",
                     i + 1, vertex_p(0), vertex_p(1), contact_rot.x(), contact_rot.y(), contact_rot.z(), contact_rot.w(), psi, len);

            object_.push_back(VertexHandlePtr(new VertexHandle(psi, contact_rot, vertex_p)));
            object_[i]->len_ = len;
          }
      }
    else if(object_type_ == aerial_transportation::ObjectConfigure::Request::CYLINDER)
      {
        std::getline(ifs, str);
        ss[1].str(str);
        ss[1] >> header >> object_radius_;
        ROS_INFO("getResultFromFile: %s: %f", header.c_str(), object_radius_);
      }

    std::getline(ifs, str);
    ss[4].str(str);
    ss[4] >> header >> contact_num_;
    ROS_INFO("getResultFromFile: %s: %d", header.c_str(), contact_num_);

    /* init for all neccessary base variables */
    v_best_tau_ = VectorXd::Constant(contact_num_ -1, 1e6);
    v_best_min_f_fc_ = VectorXd::Constant(contact_num_, 1e6);
    v_best_hover_thrust_ = VectorXd::Constant(uav_kinematics_->getRotorNum(), 0);
    v_best_theta_.resize(contact_num_ - 1);
    v_best_delta_.resize(contact_num_);
    v_best_contact_d_.resize(contact_num_);
    v_best_contact_p_.resize(contact_num_, Vector3d(0, 0, 0));
    v_best_joint_p_.resize(contact_num_, Vector3d(0, 0, 0));
    v_best_contact_rot_.resize(contact_num_, Quaterniond(1, 0, 0, 0));
    v_valid_lower_bound_theta_.resize(contact_num_ -1 );
    v_valid_lower_bound_delta_.resize(contact_num_);
    v_valid_lower_bound_contact_d_.resize(contact_num_);
    v_valid_upper_bound_theta_.resize(contact_num_ -1 );
    v_valid_upper_bound_delta_.resize(contact_num_);
    v_valid_upper_bound_contact_d_.resize(contact_num_);

    /* special process for convex contact rot */
    if(object_type_ == aerial_transportation::ObjectConfigure::Request::CONVEX_POLYGONAL_COLUMN)
      v_best_contact_rot_[0] = object_[best_start_side_]->contact_rot_;

    std::getline(ifs, str);
    ss[5].str(str);
    ss[5] >> header;
    ROS_INFO("getResultFromFile: %s:", header.c_str());
    for(int i = 0; i < contact_num_; i++)
      {
        std::stringstream ss_tmp;
        std::getline(ifs, str);
        ss_tmp.str(str);
        ss_tmp >> v_best_contact_p_[i](0) >> v_best_contact_p_[i](1);
        ROS_INFO("[%f, %f]", v_best_contact_p_[i](0), v_best_contact_p_[i](1));
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
        ss_tmp >> v_best_delta_[i];
        ROS_INFO("%f", v_best_delta_[i]);
      }

    std::getline(ifs, str);
    ss[7].str(str);
    ss[7] >> header;
    ROS_INFO("getResultFromFile: %s:", header.c_str());
    for(int i = 0; i < contact_num_; i++)
      {
        std::stringstream ss_tmp;
        std::getline(ifs, str);
        ss_tmp.str(str);
        ss_tmp >> v_valid_lower_bound_delta_[i];
        ROS_INFO("%f", v_valid_lower_bound_delta_[i]);
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
        ss_tmp >> v_valid_upper_bound_delta_[i];
        ROS_INFO("%f", v_valid_upper_bound_delta_[i]);
      }

    if(object_type_ == aerial_transportation::ObjectConfigure::Request::CONVEX_POLYGONAL_COLUMN)
      {
        std::stringstream ss2[3];
        std::getline(ifs, str);
        ss2[0].str(str);
        ss2[0] >> header;
        ROS_INFO("getResultFromFile: %s:", header.c_str());
        for(int i = 0; i < contact_num_; i++)
          {
            std::stringstream ss_tmp;
            std::getline(ifs, str);
            ss_tmp.str(str);
            ss_tmp >> v_best_contact_d_[i];
            ROS_INFO("%f", v_best_contact_d_[i]);
          }

        std::getline(ifs, str);
        ss2[1].str(str);
        ss2[1] >> header;
        ROS_INFO("getResultFromFile: %s:", header.c_str());
        for(int i = 0; i < contact_num_; i++)
          {
            std::stringstream ss_tmp;
            std::getline(ifs, str);
            ss_tmp.str(str);
            ss_tmp >> v_valid_lower_bound_contact_d_[i];
            ROS_INFO("%f", v_valid_lower_bound_contact_d_[i]);
          }

        std::getline(ifs, str);
        ss2[2].str(str);
        ss2[2] >> header;
        ROS_INFO("getResultFromFile: %s:", header.c_str());
        for(int i = 0; i < contact_num_; i++)
          {
            std::stringstream ss_tmp;
            std::getline(ifs, str);
            ss_tmp.str(str);
            ss_tmp >> v_valid_upper_bound_contact_d_[i];
            ROS_INFO("%f", v_valid_upper_bound_contact_d_[i]);
          }

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
        ss_tmp >> v_best_theta_[i];
        ROS_INFO("%f", v_best_theta_[i]);
      }

    std::getline(ifs, str);
    ss[10].str(str);
    ss[10] >> header;
    ROS_INFO("getResultFromFile: %s:", header.c_str());
    for(int i = 0; i < contact_num_ - 1; i++)
      {
        std::stringstream ss_tmp;
        std::getline(ifs, str);
        ss_tmp.str(str);
        ss_tmp >> v_valid_lower_bound_theta_[i];
        ROS_INFO("%f", v_valid_lower_bound_theta_[i]);
      }

    std::getline(ifs, str);
    ss[11].str(str);
    ss[11] >> header;
    ROS_INFO("getResultFromFile: %s:", header.c_str());
    for(int i = 0; i < contact_num_ - 1; i++)
      {
        std::stringstream ss_tmp;
        std::getline(ifs, str);
        ss_tmp.str(str);
        ss_tmp >> v_valid_upper_bound_theta_[i];
        ROS_INFO("%f", v_valid_upper_bound_theta_[i]);
      }

    std::getline(ifs, str);
    ss[12].str(str);
    ss[12] >> header;
    ROS_INFO("getResultFromFile: %s:", header.c_str());
    for(int i = 0; i < contact_num_ - 1; i++)
      {
        std::stringstream ss_tmp;
        std::getline(ifs, str);
        ss_tmp.str(str);
        ss_tmp >> v_best_tau_(i);
        ROS_INFO("%f", v_best_tau_(i));
      }

    std::getline(ifs, str);
    ss[13].str(str);
    ss[13] >> header;
    ROS_INFO("getResultFromFile: %s:", header.c_str());
    for(int i = 0; i < uav_kinematics_->getRotorNum(); i++)
      {
        std::stringstream ss_tmp;
        std::getline(ifs, str);
        ss_tmp.str(str);
        ss_tmp >> v_best_hover_thrust_(i);
        ROS_INFO("%f", v_best_hover_thrust_(i));
      }

    std::getline(ifs, str);
    ss[14].str(str);
    ss[14] >> header;
    ROS_INFO("getResultFromFile: %s:", header.c_str());
    for(int i = 0; i < contact_num_; i++)
      {
        std::stringstream ss_tmp;
        std::getline(ifs, str);
        ss_tmp.str(str);
        ss_tmp >> v_best_joint_p_[i](0) >> v_best_joint_p_[i](1);
        ROS_INFO("[%f, %f]", v_best_joint_p_[i](0), v_best_joint_p_[i](1));
      }

    search_flag_ = true;
  }

  void GraspFormSearch::showGraspResult()
  {
    //if(play_file_flag_) return;

    /* publish the final result */
    sensor_msgs::JointState searching_joint_states;
    searching_joint_states.header.stamp = ros::Time::now();
    int joint_num = uav_kinematics_->getRotorNum() - 1 ;

    searching_joint_states.position.resize(joint_num, 0);
    searching_joint_states.name.resize(joint_num);
    for(int i = 0; i < joint_num; i++)
      {
        std::stringstream joint_no;
        joint_no << i + 1;
        if(i < contact_num_ - 1) searching_joint_states.position[i] = v_best_theta_[i];
        searching_joint_states.name[i] = std::string("joint") + joint_no.str();
      }

    joint_states_pub_.publish(searching_joint_states);

    /* broadcast tf from object coord to rkbkt */
    tf::Transform transform;

    tf::Transform tf_object_origin_to_uav_root;
    tf::Vector3 uav_root_origin;
    tf::vectorEigenToTF(v_best_joint_p_[0], uav_root_origin);
    tf_object_origin_to_uav_root.setOrigin(uav_root_origin);
    tf::Quaternion uav_root_q;

    tf::quaternionEigenToTF(v_best_contact_rot_[0] * AngleAxisd(v_best_delta_[0], Vector3d::UnitZ()), uav_root_q);
    tf_object_origin_to_uav_root.setRotation(uav_root_q);
    br_.sendTransform(tf::StampedTransform(tf_object_origin_to_uav_root.inverse(), ros::Time::now(), link1_frame_name_, object_frame_name_));
  }
};
