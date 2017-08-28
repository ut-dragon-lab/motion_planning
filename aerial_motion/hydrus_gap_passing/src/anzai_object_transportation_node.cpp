#include <hydrus_gap_passing/anzai_object_transportation.h>

int main(int argc, char **argv)
{
  ros::init (argc, argv, "anzai_object_transportation");

  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  
  AnzaiObjectTransportation *anzai_object_transportation = new AnzaiObjectTransportation(nh,nhp);
  ros::spin();
  ros::shutdown();
  delete anzai_object_transportation;
  return 0;
}
