#include <boost/shared_ptr.hpp>
#include <pluginlib/class_loader.h>
#include <aerial_transportation/grasp_planning/grasp_planner.h>

int main (int argc, char **argv)
{
  ros::init (argc, argv, "grasp_planning");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  std::string grasp_planning_plugin_name_;
  nhp.param("grasp_planning_plugin_name", grasp_planning_plugin_name_, std::string("grasp_planning/full_search"));

  pluginlib::ClassLoader<grasp_planning::Base> grasp_planning_loader("aerial_transportation", "grasp_planning::Base");
  boost::shared_ptr<grasp_planning::Base> grasp_planning_method;

  try
    {
      grasp_planning_method = grasp_planning_loader.createInstance(grasp_planning_plugin_name_);
      grasp_planning_method->initialize(nh, nhp);

      ROS_INFO("Load plugin");
    }
  catch(pluginlib::PluginlibException& ex)
    {
      ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
    }
  ros::spin();

  return 0;
}

