#include <aerial_recognition/object_detection/laser_detection.h>

Object2dDetection::Object2dDetection(ros::NodeHandle nh, ros::NodeHandle nhp):
  nh_(nh), nhp_(nhp)
{
  string laserscan_topic_name;
  nhp_.param("laserscan_topic_name", laserscan_topic_name, string("laser_scan"));
  string visualization_marker_topic_name;
  nhp_.param("visualization_marker_topic_name", visualization_marker_topic_name, string("laser_scan"));
  string object_info_topic_name;
  nhp_.param("object_info_topic_name", object_info_topic_name, string("laser_scan"));

  nhp_.param("dist_thresh", dist_thresh_, 0.0);
  nhp_.param("verbose", verbose_, false);

  laserscan_sub_ = nh_.subscribe(laserscan_topic_name, 1, &Object2dDetection::laserScanCallback, this);

  visualization_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(visualization_marker_topic_name, 1);
  object_info_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>(object_info_topic_name, 1);
}

void Object2dDetection::laserScanCallback(const sensor_msgs::LaserScanConstPtr& scan_msg)
{
  /* set laser scan to data */
  CMatrixDouble data(2,scan_msg->ranges.size());
  for (size_t i = 0; i < scan_msg->ranges.size(); i++)
    {
      /* calculate the point */
      const double x = scan_msg->ranges[i] * cos(i * scan_msg->angle_increment + scan_msg->angle_min);
      const double y = -scan_msg->ranges[i] * sin(i * scan_msg->angle_increment + scan_msg->angle_min);

      data(0,i) = x;
      data(1,i) = y;
    }

  /* start RANSAC */
  double x_c, y_c, r;
  circleFitting(data, x_c, y_c, r);

  /* publish */
  geometry_msgs::Vector3Stamped object_info_msg;
  object_info_msg.header.stamp = scan_msg->header.stamp;
  object_info_msg.vector.x = x_c;
  object_info_msg.vector.y = y_c;
  object_info_msg.vector.z = r;
  object_info_pub_.publish(object_info_msg);

  visualization_msgs::MarkerArray msg;
  visualization_msgs::Marker marker;
  marker.header = scan_msg->header;
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


void Object2dDetection::circleFitting(const CMatrixDouble& all_data, double& x_c, double& y_c, double& r)
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
                  best_model
                  );

  ASSERT_(size(best_model,1)==1 && size(best_model,2)==3)

    if(verbose_)
      cout << "RANSAC finished in" << ros::Time::now().toSec() - start_t
           << "[sec]: Best model: " << best_model << endl;
}

