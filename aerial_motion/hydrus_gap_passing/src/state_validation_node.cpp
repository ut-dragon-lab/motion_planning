#include <ros/ros.h>
#include <hydrus_transform_control/transform_control.h>


#include <iostream>
#include <valarray>
#include <limits>
#include <vector>
#include <sstream>

// file
#include <fstream>


class StateValidation
{
  public:

  StateValidation(ros::NodeHandle nh, ros::NodeHandle nhp, boost::shared_ptr<TransformController>  transform_controller): nh_(nh), nhp_(nhp)
  {

    nhp_.param("joint_min", joint_min_, -90.0);
    nhp_.param("joint_max", joint_max_, 90.0);
    
    transform_controller_ = transform_controller;
    link_num_ = transform_controller_->getLinkNum();

    stateValiationFunc();
  }

  ~StateValidation(){}

  private:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;

  boost::shared_ptr<TransformController> transform_controller_;

  double joint_min_, joint_max_;
  int link_num_;


  void stateValiationFunc()
  {//only for four links, every 1 deg

    std::ofstream ofs;
    ofs.open( "state_validation_log.txt" );

    int cnt1 = 0, cnt2 = 0;

    for(int i = joint_min_; i <= joint_max_; i++)
      {
        for(int j = joint_min_; j <= joint_max_; j++)
          {
            for(int k = joint_min_; k <= joint_max_; k++)
              {
                std::vector<double> joints_angle;
                joints_angle.push_back(i / 180.0 * M_PI);
                joints_angle.push_back(j / 180.0 * M_PI);
                joints_angle.push_back(k / 180.0 * M_PI);
                bool dist_check = transform_controller_->distThreCheckFromJointValues(joints_angle, 0);
                bool range_check = transform_controller_->stabilityCheck();

                bool ctrl_check = true;
                if(fabs(transform_controller_->getUDeterminant()) < 0.0000001)
                  ctrl_check = false;

                if(!ctrl_check || !dist_check || !range_check)
                  {
                    for(int l = 0; l < link_num_ -1;  l++)
                      ofs << joints_angle[l] << "\t";

                    int type = 0;

                    if(!ctrl_check)
                      {
                        type = 1;
                        ofs << type << std::endl;
                        continue;
                      }

                    if(!range_check) type = 2;
                    if(!dist_check) type += 4;

                    ofs << type << std::endl;

                    cnt1++;
                  }
                cnt2 ++;
                if(cnt2 % 100000 == 0) ROS_INFO("step : %d", cnt2);

              }
          }
      }
    ROS_WARN("the invalid state is %d", cnt1);
    ofs.close();
  }



};



int main(int argc, char **argv)
{
  ros::init (argc, argv, "state_validation");

  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  
  boost::shared_ptr<TransformController>  transform_controller = boost::shared_ptr<TransformController>(new TransformController(nh, nhp, false));
  StateValidation *state_validation = new StateValidation(nh,nhp, transform_controller);
  ros::spin();

  ros::shutdown(); 
  delete state_validation;


 return 0;
}
