#ifndef FULL_SEARCH_H
#define FULL_SEARCH_H

#include <aerial_transportation/grasp_planning/grasp_planner.h> // plugin base class

namespace grasp_planning
{
  class FullSearch :public grasp_planning::Base
  {
  public:
    FullSearch() {}
    ~FullSearch() {}

    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp);
  protected:

    void rosParamInit();
    bool graspPlanning();

  private:
    /* rosparam based variables */
    bool one_side_flag_; /* only search the first side */
    double res_d_; /* [rad] resolution of first contact distance */
    double res_phy_; /* [mm] resolution of first contact distance */
  };
};

#endif
