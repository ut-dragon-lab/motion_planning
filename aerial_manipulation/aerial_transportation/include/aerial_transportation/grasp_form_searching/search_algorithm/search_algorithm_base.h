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

#ifndef SEARCH_ALGORITHM_BASE_PLUGIN_H
#define SEARCH_ALGORITHM_BASE_PLUGIN_H

/* ros */
#include <ros/ros.h>
#include <aerial_transportation/grasp_form_searching/grasp_form_search.h>
#include <tf_conversions/tf_eigen.h>

using namespace Eigen;

namespace grasp_form_search
{
  class GraspFormSearch;

  class SearchAlgorithmBase
  {
  public:
    virtual void initialize(ros::NodeHandle nh, ros::NodeHandle nhp, GraspFormSearch* form_searcher)
    {
      nh_ = ros::NodeHandle(nh);
      nhp_ = ros::NodeHandle(nhp);

      form_searcher_ = form_searcher;
      best_start_side_ = 0;

      nhp_.param("search_verbose", search_verbose_, false);
      std::cout << "[search algorithm] verbose: " << search_verbose_ << std::endl;

      nhp_.param("search_file_log_flag", search_file_log_flag_, false);
      std::cout << "[search algorithm] search_file_log_flag: " << search_file_log_flag_ << std::endl;
    }

    virtual ~SearchAlgorithmBase() { }
    virtual bool getSolution() {return true;}
    virtual void reset()
    {
      best_start_side_ = 0;
    }
  protected:
    /* ros node */
    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;

    bool search_verbose_;
    bool search_file_log_flag_;

    GraspFormSearch* form_searcher_;

    int best_start_side_;
  };
};

#endif  // SEARCH_ALGORITHM_BASE_PLUGIN_H
