#include <aerial_recognition/object_detection/laser_detection.h>

double Object2dDetection::r_thre_ = 0;
std::vector<tf::Vector3> Object2dDetection::link_center_v_;
double Object2dDetection::link_length_;
double Object2dDetection::link_radius_;
double Object2dDetection::collision_margin_;
boost::mutex Object2dDetection::collision_mutex_;

Object2dDetection::Object2dDetection(ros::NodeHandle nh, ros::NodeHandle nhp):
  nh_(nh), nhp_(nhp)
{
  string visualization_marker_topic_name;
  nhp_.param("visualization_marker_topic_name", visualization_marker_topic_name, string("detected_object_marker"));
  string object_info_topic_name;
  nhp_.param("object_info_topic_name", object_info_topic_name, string("detected_object"));

  nhp_.param("base_link", base_link_, string("base_link"));
  nhp_.param("dist_thresh", dist_thresh_, 0.0);
  nhp_.param("verbose", verbose_, false);

  nhp_.param("r_thre", r_thre_, 0.18);

  /* temp */
  nhp_.param("link_length", link_length_, 0.0);
  nhp_.param("link_radius", link_radius_, 0.0);
  nhp_.param("collision_margin", collision_margin_, 0.08);

  visualization_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(visualization_marker_topic_name, 1);
  object_info_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>(object_info_topic_name, 1);
  std::string joint_state_sub_name;
  nhp_.param("joint_state_sub_name", joint_state_sub_name, std::string("joint_state"));
  joint_state_sub_ = nh_.subscribe(joint_state_sub_name, 1, &Object2dDetection::jointStatecallback, this);

  /* non-sync */
  nhp_.param("scan_num", scan_num_, 0);
  scan_subs_.resize(scan_num_);
  transforms_.resize(scan_num_);
  scan_data_.resize(scan_num_);

  for(int i = 0; i < scan_num_; i++)
    {
      transforms_[i].stamp_.fromSec(0);
      string laserscan_topic_name;
      std::stringstream scan_no;
      scan_no << i + 1;
      nhp_.param(string("scan") + scan_no.str() + string("_topic_name"), laserscan_topic_name, string("scan") + scan_no.str());

      scan_subs_[i] = nh_.subscribe<sensor_msgs::LaserScan>(laserscan_topic_name, 1, boost::bind(&Object2dDetection::laserScanCallback, this, _1, i));

      /* fill the mask for the scan */
      scan_mask_ += 1 << i;
    }
  double rate;
  nhp_.param("rate", rate, 2.0); //2hz
  main_timer_ = nhp_.createTimer(ros::Duration(1.0 / rate), &Object2dDetection::circleFitting, this);
}

void Object2dDetection::laserScanCallback(const sensor_msgs::LaserScanConstPtr& scan_msg, int scan_no)
{
  //ROS_WARN("scan no: %d, size: %d", scan_no+ 1, scan_msg->ranges.size());

  /* get tf */
  ros::Duration du (0.05);
  if (tf_.waitForTransform(base_link_, scan_msg->header.frame_id, scan_msg->header.stamp, du))
    {
      try
        {
          tf_.lookupTransform(base_link_, scan_msg->header.frame_id, scan_msg->header.stamp, transforms_[scan_no]);
        }
      catch (tf::TransformException ex)
        {
          ROS_ERROR("%s",ex.what());
          return;
        }
    }
  else
    ROS_WARN("can not get fresh tf in no.%d scan, %s -> %s", scan_no, base_link_.c_str(), scan_msg->header.frame_id.c_str());


  CMatrixDouble data(2,scan_msg->ranges.size());
  /* set laser scan to data */
  for (size_t i = 0; i < scan_msg->ranges.size(); i++)
    {
      tf::Matrix3x3 rotation;
      rotation.setRPY(0, 0, i * scan_msg->angle_increment + scan_msg->angle_min);
      tf::Vector3 point = transforms_[scan_no].getOrigin() + transforms_[scan_no].getBasis() * rotation * tf::Vector3(scan_msg->ranges[i], 0, 0);

      /* 2D */
      //const double x = transforms_[i] + scan_msg->ranges[i] * cos(i * scan_msg->angle_increment + scan_msg->angle_min);
      //const double y = scan_msg->ranges[i] * sin(i * scan_msg->angle_increment + scan_msg->angle_min);

      /* 2D */
      data(0,i) = point.x();
      data(1,i) = point.y();
    }

  {
    boost::lock_guard<boost::mutex> lock(scan_mutex_);
    scan_data_[scan_no] = data;
    scan_mask_ &= ~(1 << scan_no);
  }
}


void Object2dDetection::circleFitting(const ros::TimerEvent & e)
{
  CMatrixDouble all_data;

  /* start RANSAC */
  {
    boost::lock_guard<boost::mutex> lock(scan_mutex_);
    int all_data_size = 0;

    if(scan_mask_) return;

    for(auto itr = scan_data_.begin(); itr != scan_data_.end(); ++itr)
        all_data_size += size(*itr, 2);

    all_data = CMatrixDouble(2, all_data_size);

    int itr_data = 0;
    for(auto itr = scan_data_.begin(); itr != scan_data_.end(); ++itr)
      {
        all_data.block(0, itr_data, 2, size((*itr), 2)) = *itr;
        itr_data += size((*itr), 2);
      }


  }

  double x_c, y_c, r;
  boost::lock_guard<boost::mutex> lock(scan_mutex_);
  ransac(all_data, x_c, y_c, r);

  /* publish */
  geometry_msgs::Vector3Stamped object_info_msg;
  object_info_msg.header.stamp = ros::Time::now();
  object_info_msg.header.frame_id = base_link_;
  object_info_msg.vector.x = x_c;
  object_info_msg.vector.y = y_c;
  object_info_msg.vector.z = r;
  object_info_pub_.publish(object_info_msg);

  visualization_msgs::MarkerArray msg;
  visualization_msgs::Marker marker;
  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = base_link_;
  marker.ns = "object";
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = x_c;
  marker.pose.position.y = y_c;
  marker.pose.position.z = 0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 2 * r;
  marker.scale.y = 2 * r;
  marker.scale.z = 0.3;
  marker.color.g = 1.0;
  marker.color.a = 1.0;
  msg.markers.push_back(marker);
  visualization_marker_pub_.publish(msg);
}


void Object2dDetection::ransac(const CMatrixDouble& all_data, double& x_c, double& y_c, double& r)
{
  CMatrixDouble best_model;
  vector_size_t best_inliers;

  double start_t = ros::Time::now().toSec();

  ransac_.execute(all_data,
                  circleFit,
                  circleDistance,
                  circleDegenerate,
                  dist_thresh_,
                  3,  // Minimum set of points
                  best_inliers,
                  best_model,
                  verbose_
                  );

  ASSERT_(size(best_model,1)==1 && size(best_model,2)==3)

    if(verbose_)
      cout << "RANSAC finished in " << ros::Time::now().toSec() - start_t
           << "[sec]: Best model: " << best_model << "; inlier size: "<< best_inliers.size() << endl;

  x_c = best_model(0,0);
  y_c = best_model(0,1);
  r = best_model(0,2);
}


void Object2dDetection::jointStatecallback(const sensor_msgs::JointStateConstPtr& state)
{
  std::vector<tf::Vector3> link_center_v(state->position.size() + 1);
  tf::Vector3 half_link(link_length_ / 2, 0, 0);
  link_center_v[0] = half_link;

  double abs_theta = 0;
  for(int i = 0; i < state->position.size(); i++)
    {
      link_center_v[i+1] +=
        (tf::Matrix3x3(tf::createQuaternionFromYaw(abs_theta)) * half_link + link_center_v[i]);
      abs_theta += state->position[i];
      link_center_v[i+1] += (tf::Matrix3x3(tf::createQuaternionFromYaw(abs_theta)) * half_link);

      std::cout << "link" << i+2 << ": [" << link_center_v[i+1].x() << ", " << link_center_v[i+1].y() << "]" << std::endl;
    }

  {
    boost::lock_guard<boost::mutex> lock(collision_mutex_);
    link_center_v_ = link_center_v;
  }
}
