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

/* ros */
#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/UInt8.h>

/* message filter */
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

/* ransac */
#include <mrpt/math/ransac.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/stock_objects.h>

/* mutex */
#include <boost/version.hpp>
#include <boost/thread/mutex.hpp>
#if BOOST_VERSION>105200
 #include <boost/thread/lock_guard.hpp>
#endif

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt::poses;
using namespace std;


class Object2dDetection
{
public:
  Object2dDetection(ros::NodeHandle nh, ros::NodeHandle nhp);
  ~Object2dDetection(){}

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::LaserScan> SyncPolicy;

  /* static ros param */
  static double r_max_, r_min_;
  static std::vector<tf::Vector3> link_center_v_;
  static double link_length_, link_radius_, collision_margin_;
  static boost::mutex collision_mutex_;

  static const uint8_t DIRECT_START_BIT = 7;

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;

  ros::Publisher object_info_pub_;
  ros::Publisher visualization_marker_pub_;
  ros::Subscriber start_detection_sub_;
  ros::Subscriber joint_state_sub_;

  /* laser */
  int scan_num_;
  /* non-sync mode */
  ros::Timer  main_timer_;
  vector<ros::Subscriber> scan_subs_;
  vector<tf::StampedTransform>  transforms_;
  vector<CMatrixDouble>  scan_data_;

  /* sync-mode */
  //boost::shared_ptr< message_filters::Synchronizer<SyncPolicy> > sync_;
  //vector< message_filters::Subscriber<sensor_msgs::LaserScan> > sub_image_;

  uint8_t scan_mask_;

  bool verbose_;
  double dist_thresh_;
  string base_link_;

  /* tf */
  tf::TransformListener tf_;

  /* ransac */
  math::RANSAC ransac_;

  /* mutex */
  boost::mutex scan_mutex_;

  /* common func */
  void circleFitting(const ros::TimerEvent & e);
  void laserScanCallback(const sensor_msgs::LaserScanConstPtr& scan_msg, int scan_no);
  void ransac(const CMatrixDouble& all_data, double& x_c, double& y_c, double& r);

  void jointStatecallback(const sensor_msgs::JointStateConstPtr& state);
  void startDetectionCallback(const std_msgs::UInt8ConstPtr& msg);
  void visualize(double x_c, double y_c, double r);
};

void  circleFit(const CMatrixDouble  &allData,
                const vector_size_t  &useIndices,
                vector< CMatrixDouble > &fitModels )
{
  ASSERT_(useIndices.size()==3);

  TPoint2D  p1(allData(0, useIndices[0]), allData(1, useIndices[0]));
  TPoint2D  p2(allData(0, useIndices[1]), allData(1, useIndices[1]));
  TPoint2D  p3(allData(0, useIndices[2]), allData(1, useIndices[2]));

  try
    {
      fitModels.resize(1);
      CMatrixDouble &M = fitModels[0];
      M.setSize(1,3);

      /*
        2.4 : How do I generate a circle through three points?
        Let the three given points be a, b, c. Use _0 and _1 to represent x and y coordinates. The coordinates of the center p=(p_0,p_1) of the circle determined by a, b, and c are:
        A = b_0 - a_0; B = b_1 - a_1; C = c_0 - a_0; D = c_1 - a_1;
        E = A*(a_0 + b_0) + B*(a_1 + b_1); F = C*(a_0 + c_0) + D*(a_1 + c_1);
        G = 2.0*(A*(c_1 - b_1)-B*(c_0 - b_0));
        p_0 = (D*E - B*F) / G; p_1 = (A*F - C*E) / G;
        If G is zero then the three points are collinear and no finite-radius circle through them exists. Otherwise, the radius of the circle is:
        r^2 = (a_0 - p_0)^2 + (a_1 - p_1)^2
        [O' Rourke (C)] p. 201. Simplified by Jim Ward.
      */

      double A = p2.x - p1.x,
        B = p2.y - p1.y,
        C = p3.x - p1.x,
        D = p3.y - p1.y;

      double E = A*(p1.x + p2.x) + B*(p1.y + p2.y),
        F = C*(p1.x + p3.x) + D*(p1.y + p3.y),
        G = 2.0*(A*(p3.y - p2.y)-B*(p3.x - p2.x));

      if (G == 0) throw exception();

      M(0, 0) = (D*E - B*F) / G; //c_x
      M(0, 1) = (A*F - C*E) / G; //c_y
      M(0, 2) = sqrt((p1.x - M(0, 0)) * (p1.x - M(0, 0))
                     + (p1.y - M(0, 1)) * (p1.y - M(0, 1))); // r

      /* radius check */
      if(M(0, 2) > Object2dDetection::r_max_ || M(0, 2) < Object2dDetection::r_min_)
        throw exception();

      /* collision check */
      {
        boost::lock_guard<boost::mutex> lock(Object2dDetection::collision_mutex_);

        tf::Vector3 obj_c(M(0, 0), M(0, 1), 0);

        for(auto it = Object2dDetection::link_center_v_.begin(); it != Object2dDetection::link_center_v_.end(); ++it)
          {
            if((*it - obj_c).length() < Object2dDetection::link_radius_ + M(0,2) - Object2dDetection::collision_margin_)
              throw exception();
          }
      }
    }
  catch(exception &)
    {
      fitModels.clear();
      return;
    }
}

void circleDistance(const CMatrixDouble &allData,
                    const vector< CMatrixDouble > & testModels,
                    const double distanceThreshold,
                    unsigned int & out_bestModelIndex,
                    vector_size_t & out_inlierIndices )
{
  ASSERT_(testModels.size() == 1)
    out_bestModelIndex = 0;
  const CMatrixDouble &M = testModels[0];

  ASSERT_( size(M,1)==1 && size(M,2)==3 )

    const size_t N = size(allData,2);
  out_inlierIndices.clear();
  out_inlierIndices.reserve(100);
  for (size_t i=0;i<N;i++)
    {
      const double d =
        fabs(sqrt((allData.get_unsafe(0,i) - M(0, 0)) * (allData.get_unsafe(0,i) - M(0, 0))
                  + (allData.get_unsafe(1,i) - M(0, 1)) * (allData.get_unsafe(1,i) - M(0, 1))) - M(0, 2));

      if (d < distanceThreshold)
        out_inlierIndices.push_back(i);
    }
}

/** Return "true" if the selected points are a degenerate (invalid) case.
 */
bool circleDegenerate(const CMatrixDouble &allData,
                      const mrpt::vector_size_t &useIndices )
{
  return false;
}

