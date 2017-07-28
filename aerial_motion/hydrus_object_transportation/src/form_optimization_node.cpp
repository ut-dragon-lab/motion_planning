#include <hydrus_object_transportation/form_optimization.h>

int main(int argc, char **argv)
{
  ros::init (argc, argv, "hydrus_form_optimization");

  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  
  FormOptimization *form_optimization = new FormOptimization(nh, nhp);
  form_optimization->process();

  ros::spin();
  ros::shutdown();
  delete form_optimization;
  
  return 0;
}
