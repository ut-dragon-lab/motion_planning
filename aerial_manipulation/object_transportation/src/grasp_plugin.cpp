#include <pluginlib/class_list_macros.h>
#include <object_transportation/grasp/grasp_base_plugin.h>
#include <object_transportation/grasp/eletromagnet.h>
#include <object_transportation/grasp/hydrus.h>

PLUGINLIB_EXPORT_CLASS(grasp_plugin::Eletromagnet, grasp_base_plugin::GraspBase);
PLUGINLIB_EXPORT_CLASS(grasp_plugin::Hydrus, grasp_base_plugin::GraspBase);

