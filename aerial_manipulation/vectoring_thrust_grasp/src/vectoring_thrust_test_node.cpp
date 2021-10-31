#include <vectoring_thrust_grasp/vectoring_thrust_planner.h>

int main (int argc, char **argv)
{
  ros::init (argc, argv, "grasp_vectoring_thrust");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  bool from_real_joint_angle;
  nh_private.param("from_real_joint_angle", from_real_joint_angle, false);
  GraspVectoringThrust*  grasp_vectoring_thrust_node = new GraspVectoringThrust(nh, nh_private, boost::make_shared<Dragon::HydrusLikeRobotModel>(true), from_real_joint_angle);
  ros::spin ();
  delete grasp_vectoring_thrust_node;
  return 0;
}




