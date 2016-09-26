#include <hydrus_gap_passing/motion_control.h>

int main(int argc, char **argv)
{
  ros::init (argc, argv, "hydrus_motion_control");

  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  
  boost::shared_ptr<TransformController> transform_controller;
  MotionControl *motion_control = new MotionControl(nh,nhp, transform_controller);
  ros::spin();

  ros::shutdown();
  delete motion_control;


 return 0;
}
