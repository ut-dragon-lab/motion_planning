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

#ifndef BSPLINE_GENERATOR_H_
#define BSPLINE_GENERATOR_H_

/* ros */
#include <ros/ros.h>
#include <bspline_generator/TinysplineInterface.h>
#include <std_msgs/MultiArrayDimension.h>
#include <gap_passing/Keyposes.h>

/* utils */
#include <iostream>
#include <vector>

using namespace tinyspline_interface;

namespace bspline_generator{
  #define PI 3.1415926
  class BsplineGenerator{
  public:
    BsplineGenerator(ros::NodeHandle nh, ros::NodeHandle nhp, double period, int bspline_degree);
    ~BsplineGenerator();
    void manuallySetControlPts();
    void displayBspline();
    std::vector<double> getPosition(double time);
    std::vector<double> getVelocity(double time);
    std::vector<double> getKeypose(int id);

  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;
    double period_;
    int deg_;
    boost::shared_ptr<TinysplineInterface> bspline_ptr_;
    ControlPoints* control_pts_ptr_;
    ros::ServiceClient sampling_plannar_client_;
    void waitForKeyposes();
    double getContinousYaw(double yaw, int id);
  };
}
#endif
