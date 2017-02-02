#include <hydrus_object_transportation/hydrus_object_transportation.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hydrus_object_transportation");

  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  
  HydrusObjectTransportation *hydrus_object_transportation = new HydrusObjectTransportation(nh,nhp);
  ros::spin();
  ros::shutdown();
  delete hydrus_object_transportation;
  return 0;
}
