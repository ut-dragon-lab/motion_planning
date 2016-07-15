#include <boost/shared_ptr.hpp>
#include <pluginlib/class_loader.h>
#include <object_transportation/grasp/grasp_base_plugin.h>

int main(int argc, char** argv)
{
  ros::init (argc, argv, "grasp_plugin_test");

  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  pluginlib::ClassLoader<grasp_base_plugin::GraspBase> grasp_loader("object_transportation", "grasp_base_plugin::GraspBase");
  boost::shared_ptr<grasp_base_plugin::GraspBase> eletro_magnet;

  try
     {
       eletro_magnet  = grasp_loader.createInstance("grasp_plugin/eletromagnet");
       eletro_magnet->initialize(nh, nhp);

       ROS_INFO("Result OK");
     }
   catch(pluginlib::PluginlibException& ex)
     {
       ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
     }

  ros::spin();

   return 0;
}
