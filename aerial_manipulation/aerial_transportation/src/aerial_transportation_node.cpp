#include <boost/shared_ptr.hpp>
#include <pluginlib/class_loader.h>
#include <aerial_transportation/grasp_control/base.h>

int main (int argc, char **argv)
{
  ros::init (argc, argv, "aerial_transportation");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  std::string aerial_transportation_plugin_name_;
  nhp.param("aerial_transportation_plugin_name", aerial_transportation_plugin_name_, std::string("aerial_transportation/eletromagnet"));

  pluginlib::ClassLoader<aerial_transportation::Base> aerial_transportation_loader("aerial_transportation", "aerial_transportation::Base");
  boost::shared_ptr<aerial_transportation::Base> aerial_transportation_method;

  try
    {
      aerial_transportation_method = aerial_transportation_loader.createInstance(aerial_transportation_plugin_name_);
      aerial_transportation_method->initialize(nh, nhp);

      ROS_INFO("Load plugin");
    }
  catch(pluginlib::PluginlibException& ex)
    {
      ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
    }

  ros::spin();

  return 0;
}

