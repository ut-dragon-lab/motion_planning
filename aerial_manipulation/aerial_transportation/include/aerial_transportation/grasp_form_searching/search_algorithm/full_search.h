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
#ifndef FULL_SEARCH_H
#define FULL_SEARCH_H

#include <aerial_transportation/grasp_form_searching/grasp_form_search.h>

/* file */
#include <iostream>
#include <sstream>
#include <fstream>

namespace grasp_form_search
{
  class FullSearch :public grasp_form_search::SearchAlgorithmBase
  {
  public:
    FullSearch(): search_map_(0) {}
    ~FullSearch() {}

    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp, GraspFormSearch* form_searcher) override;

    bool getSolution() override;

    void reset() override;
  private:

    /* rosparam based variables */
    bool one_side_flag_; /* only search the first side */
    double res_delta_; /* [rad] resolution of first contact angle */
    double res_d_; /* [m] resolution of first contact distance */

    double delta_valid_range_; /* the valid range of delta to search */
    double d_valid_range_; /* the valid range of d to search */

    /* cost function: F = w_1 * max_val(F) + w_2 * max_val(tau) */
    double thrust_weight_; /* the weight of thrust in the cost function */
    double torque_weight_; /* the weight of torque in the cost function */

    std::vector<MatrixXd> search_map_;
  };
};

#endif
