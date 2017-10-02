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
#include <hydrus/transform_control.h>

/* utils */
#include <iostream>
#include <valarray>
#include <limits>
#include <vector>
#include <sstream>
#include <fstream>

class StateValidation
{
  public:

  StateValidation(ros::NodeHandle nh, ros::NodeHandle nhp, boost::shared_ptr<TransformController>  transform_controller): nh_(nh), nhp_(nhp)
  {
    nhp_.param("joint_min", joint_min_, -90.0);
    nhp_.param("joint_max", joint_max_, 90.0);

    transform_controller_ = transform_controller;
    link_num_ = transform_controller_->getRotorNum();

    stateValiationFunc();
  }

  ~StateValidation(){}

  private:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;

  boost::shared_ptr<TransformController> transform_controller_;

  double joint_min_, joint_max_;
  int link_num_;


  void stateValiationFunc()
  {//only for four links, every 1 deg

    std::ofstream ofs;
    ofs.open( "state_validation_log.txt" );

    int cnt1 = 0, cnt2 = 0;

    for(int i = joint_min_; i <= joint_max_; i++)
      {
        for(int j = joint_min_; j <= joint_max_; j++)
          {
            for(int k = joint_min_; k <= joint_max_; k++)
              {
                sensor_msgs::JointState joint_state;
                joint_state.name.push_back(std::string("joint1"));
                joint_state.position.push_back(i / 180.0 * M_PI);
                joint_state.name.push_back(std::string("joint2"));
                joint_state.position.push_back(j / 180.0 * M_PI);
                joint_state.name.push_back(std::string("joint3"));
                joint_state.position.push_back(k / 180.0 * M_PI);

                transform_controller_->kinematics(joint_state);
                bool dist_check = transform_controller_->distThreCheck();
                bool range_check = transform_controller_->modelling();

                bool ctrl_check = true;
                if(fabs(transform_controller_->getP().determinant()) < 0.0000001)
                  ctrl_check = false;

                if(!ctrl_check || !dist_check || !range_check)
                  {
                    for(int l = 0; l < link_num_ -1;  l++)
                      ofs << joint_state.position[l] << "\t";

                    int type = 0;

                    if(!ctrl_check)
                      {
                        type = 1;
                        ofs << type << std::endl;
                        continue;
                      }

                    if(!range_check) type = 2;
                    if(!dist_check) type += 4;

                    ofs << type << std::endl;

                    cnt1++;
                  }
                cnt2 ++;
                if(cnt2 % 100000 == 0) ROS_INFO("step : %d", cnt2);

              }
          }
      }
    ROS_WARN("the invalid state is %d", cnt1);
    ofs.close();
  }



};



int main(int argc, char **argv)
{
  ros::init (argc, argv, "state_validation");

  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  
  boost::shared_ptr<TransformController>  transform_controller = boost::shared_ptr<TransformController>(new TransformController(nh, nhp, false));
  StateValidation *state_validation = new StateValidation(nh,nhp, transform_controller);
  ros::spin();

  ros::shutdown(); 
  delete state_validation;


 return 0;
}
