#include <path2d/2d_path.h>



int main(int argc, char** argv){
  ros::init(argc, argv, "move_base_node");

  ros::NodeHandle node_handle;
  ros::NodeHandle node_handle_private("~");
  tf::TransformListener tf(ros::Duration(10));
  Path2D* path_2d = new Path2D(node_handle, node_handle_private, tf);

  ros::spin();
  delete path_2d;

  return(0);

}

