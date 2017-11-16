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
#include <geometry_msgs/PolygonStamped.h>

/*************************************************
      note that, the origin of coordinate of
       1) the convex polygon is the first vertex, and the x axis direction is the first side.
       2) the clydiner is the center of the cylinder, and the y axis is the opposite of the direction from center to the first contact point.
**************************************************/

namespace grasp_form_search
{
  namespace
  {
    /* ros publisher & subscirber */
    ros::Subscriber convex_polygonal_column_info_sub_; /* the geometric information of convex_polygon_object */
    ros::Subscriber cylinder_info_sub_; /* the geometric information of cylinder_object */

    std::string convex_polygonal_column_info_sub_name_;
    std::string cylinder_info_sub_name_;

    double joint_angle_limit_; /* the limit of joint angle */
  }


  void GraspFormSearch::kinematicsInit()
  {
    
    convex_polygonal_column_info_sub_ = nh_.subscribe<geometry_msgs::PolygonStamped>(convex_polygonal_column_info_sub_name_, 1, &GraspFormSearch::convexPolygonalColumnInfoCallback, this);
    cylinder_info_sub_ = nh_.subscribe<visualization_msgs::Marker>(cylinder_info_sub_name_, 1, &GraspFormSearch::cylinderInfoCallback, this);
  }
};
