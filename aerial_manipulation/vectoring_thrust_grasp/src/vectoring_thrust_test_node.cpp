#include <vectoring_thrust_grasp/vectoring_thrust_test.h>

int main (int argc, char **argv)
{
  ros::init (argc, argv, "grasp_vectoring_thrust");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  GraspVectoringThrust*  grasp_vectoring_thrust_node = new GraspVectoringThrust(nh, nh_private, boost::shared_ptr<DragonRobotModel>(new DragonRobotModel(true)));
  ros::spin ();
  delete grasp_vectoring_thrust_node;
  return 0;
}




