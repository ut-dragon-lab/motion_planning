#include <vectoring_thrust_grasp/grasping_motion.h>

int main (int argc, char **argv)
{
  ros::init (argc, argv, "grasping_motion");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  GraspingMotion*  grasping_motion_node = new GraspingMotion(nh, nh_private);
  ros::spin ();
  delete grasping_motion_node;
  return 0;
}




