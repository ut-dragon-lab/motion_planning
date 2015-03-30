#include <wave_propagation/wave_propagation_ros.h>
#include <wave_propagation/SetCostmap.h>
#include <wave_propagation/MakeNavPlan.h>
#include <boost/shared_ptr.hpp>

namespace cm=costmap_2d;
namespace rm=geometry_msgs;

using std::vector;
using rm::PoseStamped;
using std::string;
using cm::Costmap2D;
using cm::Costmap2DROS;

typedef boost::shared_ptr<Costmap2D> CmapPtr;

namespace wave_propagation {

class WavePropagationWithLocalCostmap : public WavePropagationROS
{
public:
  WavePropagationWithLocalCostmap(string name, Costmap2DROS* cmap);
  bool setCostmap(SetCostmap::Request& req, SetCostmap::Response& resp);
  bool makePlanService(MakeNavPlan::Request& req, MakeNavPlan::Response& resp);

protected:
  virtual void getCostmap(Costmap2D& cmap);

private:
  CmapPtr cmap_;
  ros::ServiceServer set_costmap_service_;
  ros::ServiceServer make_plan_service_;
};


bool WavePropagationWithLocalCostmap::makePlanService(MakeNavPlan::Request& req, MakeNavPlan::Response& resp)
{
  vector<PoseStamped> path;

  req.start.header.frame_id = "/map";
  req.goal.header.frame_id = "/map";

  bool success = makePlan(req.start, req.goal, path);

  resp.plan_found = success;
  if (success) {
    resp.path = path;
  }

  return true;
}


WavePropagationWithLocalCostmap::WavePropagationWithLocalCostmap(string name, Costmap2DROS* cmap) : 
  WavePropagationROS(name, cmap)
{
  inscribed_radius_ = 0.0;
  circumscribed_radius_ = 0.0;
  inflation_radius_ = 0.0;

  ros::NodeHandle private_nh("~");
  set_costmap_service_ = private_nh.advertiseService("set_costmap", &WavePropagationWithLocalCostmap::setCostmap, this);
  make_plan_service_ = private_nh.advertiseService("make_plan", &WavePropagationWithLocalCostmap::makePlanService, this);
}


bool WavePropagationWithLocalCostmap::setCostmap(SetCostmap::Request& req, SetCostmap::Response& resp)
{
  cmap_.reset(new Costmap2D(req.width, req.height, 1.0, 0.0, 0.0));
  unsigned ind=0;
  for (unsigned y=0; y<req.height; y++) 
    for (unsigned x=0; x<req.width; x++) 
      cmap_->setCost(x, y, req.costs[ind++]);


  for (unsigned y=0; y<req.height; y++) 
    for (unsigned x=0; x<req.width; x++) 
      ROS_DEBUG_NAMED ("node", "Cost of %u, %u is %u", x, y, cmap_->getCost(x,y));

  planner_.reset(new WavePropagation(cmap_->getSizeInCellsX(), cmap_->getSizeInCellsY()));
  ROS_DEBUG_STREAM_NAMED ("node", "Resetting planner object to have size " << cmap_->getSizeInCellsX() << ", " << cmap_->getSizeInCellsY());

  return true;
}

void WavePropagationWithLocalCostmap::getCostmap(Costmap2D& cmap)
{
  cmap = *cmap_;
}


} // namespace

int main (int argc, char** argv)
{
  ros::init(argc, argv, "WavePropagation_node");
  

  ros::NodeHandle n("~");
  tf::TransformListener tf;

  // Set params
  n.setParam("dummy_costmap/global_frame", "/map");
  n.setParam("dummy_costmap/robot_base_frame", "/map"); // Do this so it doesn't complain about unavailable transform 
  n.setParam("dummy_costmap/publish_frequency", 0.0);
  n.setParam("dummy_costmap/observation_sources", string(""));
  n.setParam("dummy_costmap/static_map", false);

  
  Costmap2DROS dummy_costmap("dummy_costmap", tf);
  wave_propagation::WavePropagationWithLocalCostmap WavePropagation("WavePropagation_planner", &dummy_costmap);

  ros::spin();
  return 0;
}
















