#include <hydra_gap_passing/motion_planning.h>

int main(int argc, char **argv)
{
  ros::init (argc, argv, "hydra_motion_planning");

  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  
  MotionPlanning *motion_planning = new MotionPlanning(nh,nhp);
  ros::spin();

  ros::shutdown(); 
  delete motion_planning;


 return 0;
}
