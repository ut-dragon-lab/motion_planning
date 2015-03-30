#include <collision_check/correlation_coefficient_check.h>


int main(int argc, char **argv)
{
  ros::init (argc, argv, "correlation_coefficient");

  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  
  CorrelationCoefficient *correlation_coefficient = new CorrelationCoefficient(nh,nhp);
  ros::spin();


  ros::shutdown(); 
  delete correlation_coefficient;


 return 0;
}


