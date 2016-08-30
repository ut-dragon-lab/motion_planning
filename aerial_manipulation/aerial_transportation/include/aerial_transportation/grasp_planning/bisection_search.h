#ifndef BISECTION_SEARCH_H
#define BISECTION_SEARCH_H

#include <aerial_transportation/grasp_planning/grasp_planner.h> // plugin base class

namespace grasp_planning
{
  class BisectionSearch :public grasp_planning::Base
  {
  public:
    BisectionSearch() {}
    ~BisectionSearch() {}

    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp);
  protected:

    void rosParamInit();
    bool graspPlanning();

  private:
    /* rosparam based variables */
    bool one_side_flag_; /* only search the first side */
    double thre_d_; /* [rad] threshold of bisection in terms of d */
    double thre_phy_; /* [mm] threshold of bisection in terms of phy */
    double res_phy_; /* [mm] resolution of phy to escape the invalid point */
    int k_; /* max itreration */
  };
};

#endif
