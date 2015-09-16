#include <ros/ros.h>
#include <hydra_transform_control/transform_control.h>

#include <iostream>
#include <valarray>
#include <limits>
#include <vector>
#include <sstream>

// file
#include <fstream>

class StateValidity
{
public:
  StateValidity(ros::NodeHandle nh, ros::NodeHandle nhp, boost::shared_ptr<TransformController>  transform_controller);
  ~StateValidity();


private:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  boost::shared_ptr<TransformController> transform_controller_;

  double interval_;

};


int main(int argc, char **argv)
{
  ros::init (argc, argv, "state_validity_check");

  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  
  boost::shared_ptr<TransformController>  transform_controller = boost::shared_ptr<TransformController>(new TransformController(nh, nhp, false));
  StateValidity *state_validity = new StateValidity(nh,nhp, transform_controller);
  ros::spin();

  ros::shutdown(); 
  delete state_validity;


 return 0;
}
