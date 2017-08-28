#include <hydrus_object_transportation/object_detection.h>

int main(int argc, char **argv)
{
  ros::init (argc, argv, "object_detection");

  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  
  ObjectDetection *object_detection = new ObjectDetection(nh, nhp);

  ros::spin();
  ros::shutdown();
  delete object_detection;
  
  return 0;
}
