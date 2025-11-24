// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, JSK Lab
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

#include <aerial_motion_planning_msgs/continuous_path_generator.h>

ContinuousPathGenerator:: ContinuousPathGenerator(ros::NodeHandle nh, ros::NodeHandle nhp, boost::shared_ptr<aerial_robot_model::transformable::RobotModel> robot_model_ptr): nh_(nh), nhp_(nhp), robot_model_ptr_(robot_model_ptr)
{
  nhp_.param("debug_verbose", debug_verbose_, false);

  /* continous path generator */
  bspline_ = boost::make_shared<BsplineRos>(nh_, nhp_);
}


const std::vector<double> ContinuousPathGenerator::getPositionVector(double t)
{
  return bspline_->evaluate(t);
}

const std::vector<double> ContinuousPathGenerator::getVelocityVector(double t)
{
  return bspline_->evaluate(t, 1);
}

const std::vector<MultilinkState> ContinuousPathGenerator::discretePathSmoothing(const std::vector<MultilinkState>& raw_path) const
{
  /* low path filter */
  double filter_rate;
  nhp_.param("filter_rate", filter_rate, 0.1);

  int joint_num = robot_model_ptr_->getLinkJointIndices().size();
  std::vector<MultilinkState> filtered_path(0);
  FirFilter states_lpf1 = FirFilter(filter_rate, 3 + joint_num); //root position + joint_num
  FirFilterQuaternion states_lpf2 = FirFilterQuaternion(filter_rate); //root orientation

  /* init state */
  auto init_state = raw_path.at(0);
  Eigen::VectorXd init_state_vec = Eigen::VectorXd::Zero(3 + joint_num);
  init_state_vec.head(3) = Eigen::Vector3d(init_state.getRootPoseConst().position.x,
                                           init_state.getRootPoseConst().position.y,
                                           init_state.getRootPoseConst().position.z);
  for(int j = 0; j < joint_num; j++)
    init_state_vec(3 + j) = init_state.getJointStateConst()(robot_model_ptr_->getLinkJointIndices().at(j));
  states_lpf1.setInitValues(init_state_vec); //init filter with the first value

  tf::Quaternion init_q;
  tf::quaternionMsgToTF(init_state.getRootPoseConst().orientation, init_q);
  states_lpf2.setInitValues(init_q); //init filter with the first value

  /* do filtering */
  for(int index = 0; index < raw_path.size() + 1 / states_lpf1.getFilterFactor(); index++)
    {
      auto state_itr = raw_path.back();
      if(index < raw_path.size()) state_itr = raw_path.at(index);

      Eigen::VectorXd state_vec = Eigen::VectorXd::Zero(3 + joint_num);
      state_vec.head(3) = Eigen::Vector3d(state_itr.getRootPoseConst().position.x,
                                          state_itr.getRootPoseConst().position.y,
                                          state_itr.getRootPoseConst().position.z);

      for(int j = 0; j < joint_num; j++)
        state_vec(3 + j) = state_itr.getJointStateConst()(robot_model_ptr_->getLinkJointIndices().at(j));
      Eigen::VectorXd filtered_state =  states_lpf1.filterFunction(state_vec);

      /* joint */
      auto filtered_joint_vector = state_itr.getJointStateConst();
      for(int j = 0; j < joint_num; j++)
        filtered_joint_vector(robot_model_ptr_->getLinkJointIndices().at(j)) = filtered_state(3 + j);

      /* root position */
      geometry_msgs::Pose filtered_root_pose;
      filtered_root_pose.position.x = filtered_state(0);
      filtered_root_pose.position.y = filtered_state(1);
      filtered_root_pose.position.z = filtered_state(2);

      /* root orientation */
      tf::Quaternion raw_q;
      tf::quaternionMsgToTF(state_itr.getRootPoseConst().orientation, raw_q);
      tf::quaternionTFToMsg(states_lpf2.filterFunction(raw_q), filtered_root_pose.orientation);

      filtered_path.push_back(MultilinkState(robot_model_ptr_, filtered_root_pose, filtered_joint_vector));
    }

  return filtered_path;
}

const std::vector<MultilinkState> ContinuousPathGenerator::discretePathResampling(const std::vector<MultilinkState>& raw_path) const
{
  /* resampling */
  double resampling_angular_rate;
  double resampling_seg_diff_thresh;
  nhp_.param("resampling_angular_rate", resampling_angular_rate, 1.0);
  nhp_.param("resampling_seg_diff_thresh", resampling_seg_diff_thresh, 0.2);
  double path_length = 0;

  for(int i = 0; i < raw_path.size() - 1; i++)
    {
      /* translation motion */
      tf::Vector3 p1, p2;
      tf::pointMsgToTF(raw_path.at(i).getCogPoseConst().position, p1);
      tf::pointMsgToTF(raw_path.at(i + 1).getCogPoseConst().position, p2);
      path_length += (p1-p2).length();

      /* rotational motion */
      double angle_diff = fabs(raw_path.at(i).getBaselinkDesiredAttConst().angleShortestPath(raw_path.at(i+1).getBaselinkDesiredAttConst()));
      path_length += resampling_angular_rate * angle_diff;
      //ROS_WARN("%d: angle_diff: %f", i, angle_diff);
    }

  double ave_delta_trans = path_length / raw_path.size();
  ROS_WARN("path length is %f, average delta trans is %f", path_length, ave_delta_trans);

  /* do resampling */
  std::vector<MultilinkState> resampling_path;
  double delta_trans = 0;
  bool not_enough = false;
  resampling_path.push_back(raw_path.front());

  for(int i = 1; i < raw_path.size();)
    {
      tf::Vector3 prev_p;
      tf::pointMsgToTF(raw_path.at(i - 1).getCogPoseConst().position, prev_p);
      tf::Quaternion prev_q = raw_path.at(i - 1).getBaselinkDesiredAttConst();
      tf::Vector3 new_p;
      tf::pointMsgToTF(raw_path.at(i).getCogPoseConst().position, new_p);
      tf::Quaternion new_q = raw_path.at(i).getBaselinkDesiredAttConst();
      tf::Vector3 ref_p;
      tf::pointMsgToTF(resampling_path.back().getCogPoseConst().position, ref_p);
      tf::Quaternion ref_q = resampling_path.back().getBaselinkDesiredAttConst();

      if(delta_trans == 0) delta_trans = (ref_p - new_p).length() + resampling_angular_rate * fabs(new_q.angleShortestPath(ref_q));
      if(not_enough)
        {
          delta_trans += ((prev_p - new_p).length() + resampling_angular_rate * fabs(new_q.angleShortestPath(prev_q)));
          not_enough = false;
        }

      if(debug_verbose_)
        {
          double r,p,y;
          tf::Matrix3x3(resampling_path.back().getBaselinkDesiredAttConst()).getRPY(r,p,y);
          ROS_INFO("ref pose [%f, %f, %f] [%f, %f, %f]", ref_p.x(), ref_p.y() ,ref_p.z(), r, p, y);
          std::cout << "new point vs resample ref point: [" << i << ", " << resampling_path.size() << "], inrcement vs delta_trans vs ave: [" << (ref_p - new_p).length() + resampling_angular_rate * fabs(new_q.angleShortestPath(ref_q)) << ", " << delta_trans << ", " << ave_delta_trans << "]. ";
        }

      if(fabs(delta_trans - ave_delta_trans) < resampling_seg_diff_thresh * ave_delta_trans)
        {
          if(debug_verbose_)
            {
              std::cout << "add raw state directly since convergence. rate: " << fabs(delta_trans - ave_delta_trans) / ave_delta_trans << ". " << std::endl;
              ROS_WARN("delta_trans: %f", delta_trans);
            }
          resampling_path.push_back(raw_path.at(i));
          delta_trans = 0;
          i++;
        }
      else if (delta_trans >  ave_delta_trans)
        {
          // interpolation
          double interpolate_rate = 1 - (delta_trans - ave_delta_trans) / ((prev_p - new_p).length() + resampling_angular_rate * fabs(prev_q.angleShortestPath(new_q)));
          if(debug_verbose_)
            std::cout << " interpolation since too big delta trans. interpolate rate: " << interpolate_rate << ". "  << std::endl;


          /* joint */
          auto joint_vector = raw_path.at(i - 1).getJointStateConst();
          for(auto index: robot_model_ptr_->getLinkJointIndices())
            joint_vector(index) = raw_path.at(i - 1).getJointStateConst()(index) * (1 - interpolate_rate) + raw_path.at(i).getJointStateConst()(index) * interpolate_rate;


          /* cog pose */
          geometry_msgs::Pose cog_pose;
          tf::pointTFToMsg(prev_p * (1 - interpolate_rate) + new_p * interpolate_rate, cog_pose.position);
          cog_pose.orientation.w = 1;

          /* baselink attitude and  root pose */
          tf::Quaternion desired_baselink_q = raw_path.at(i - 1).getBaselinkDesiredAttConst().slerp(raw_path.at(i).getBaselinkDesiredAttConst(), interpolate_rate);

          resampling_path.push_back(MultilinkState(robot_model_ptr_,
                                                   desired_baselink_q, cog_pose,
                                                   joint_vector));
          tf::Vector3 new_ref_p;
          tf::pointMsgToTF(resampling_path.back().getCogPoseConst().position, new_ref_p);
          tf::Quaternion new_ref_q = resampling_path.back().getBaselinkDesiredAttConst();
          delta_trans = (new_ref_p - new_p).length() + resampling_angular_rate * fabs(new_q.angleShortestPath(new_ref_q));

          if(debug_verbose_) ROS_WARN("delta_trans: %f", delta_trans);
        }
      else
        {
          not_enough = true;
          i++;
          if(debug_verbose_) std::cout << "  not enough delta trans. "  << std::endl;
        }
    }

  return resampling_path;
}

void ContinuousPathGenerator::calcContinuousPath(const std::vector<MultilinkState>& discrete_path, double trajectory_period)
{
  int joint_num = robot_model_ptr_->getLinkJointIndices().size();

  /* insert data */
  std::vector<std::vector<double> > control_point_list;

  double prev_yaw = 0;
  for (int i = -1; i < (int)discrete_path.size() + 1; i++)
    {
      /* add one more start & end keypose to guarantee speed 0 */
      int id = i;
      if (id < 0) id = 0;
      if (id >= discrete_path.size()) id = discrete_path.size() - 1;

      std::vector<double> control_point;

      // cog position
      const auto cog_pos = discrete_path.at(id).getCogPoseConst().position;
      control_point.push_back(cog_pos.x);
      control_point.push_back(cog_pos.y);
      control_point.push_back(cog_pos.z);

      // euler enagles
      tf::Matrix3x3 att(discrete_path.at(id).getBaselinkDesiredAttConst());
      double r, p, y; att.getRPY(r, p, y);
      control_point.push_back(r);
      control_point.push_back(p);
      // keep yaw euler angle continous
      if (id == 0) prev_yaw = y;
      y = generateContinousEulerAngle(y, prev_yaw);
      control_point.push_back(y);
      prev_yaw = y;

      /* set joint state */
      for(auto itr : robot_model_ptr_->getLinkJointIndices())
        control_point.push_back(discrete_path.at(id).getJointStateConst()(itr));

      control_point_list.push_back(control_point);
    }

  // continuous path
  int bspline_degree;
  nhp_.param("bspline_degree", bspline_degree, 5);

  bspline_->initialize(true, 0, trajectory_period, bspline_degree, control_point_list);
  std::vector<int> pos_indicies{0,1,2};
  bspline_->display3dPath(pos_indicies);

  ROS_INFO("Generated continuous based on B-Spline .");
}

