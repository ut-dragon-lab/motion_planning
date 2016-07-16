#include <object_transportation/object_transportation.h>

int main (int argc, char **argv)
{
  ros::init (argc, argv, "object_transpotation");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  ObjectTransportation*  objectTransportationNode = new ObjectTransportation(nh, nh_private);
  ros::spin ();
  delete objectTransportationNode;
  return 0;
}

