#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <vector>

  float correlationCoefficientCheck(std::vector<tf::Vector3> origins)  
  {

        float x_sum = 0, y_sum = 0, z_sum = 0;
        int extend_point_num = 10;
        for(int i = 0; i < (int)origins.size(); i++)
          {
            //ROS_INFO("x: %f, y: %f", origins[i].x(), origins[i].y());
            for(int j = 0; j < extend_point_num; j++)
              {
                
                x_sum += origins[i].x() + 0.1 * cos(j * 0.785);
                y_sum += origins[i].y() + 0.1 * sin(j * 0.785);
                z_sum += origins[i].z() ;
              }
          }
        std::vector<float> cog(3,0.0);
        cog[0] = x_sum / ((int)origins.size() * extend_point_num); 
        cog[1] = y_sum / ((int)origins.size() * extend_point_num); 
        cog[2] = z_sum / ((int)origins.size() * extend_point_num); 
        float cov_x = 0, cov_y = 0, cov_xy = 0;
        for(int j = 0; j < (int)origins.size(); j ++)
          {
            cov_x = cov_x + ((origins[j].x() - cog[0]) * (origins[j].x() - cog[0]));
            cov_y = cov_y + ((origins[j].y() - cog[1]) * (origins[j].y() - cog[1]));
            cov_xy = cov_xy + ((origins[j].x() - cog[0]) * (origins[j].y() - cog[1]));
          }
        float correlation;
        correlation = cov_xy / (sqrt(cov_x) * sqrt(cov_y));
        ROS_INFO("correlation coefficient is %f, %f,%f,%f", correlation, cov_xy, cov_x, cov_y);
        return correlation;
  }



class CorrelationCoefficient
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  ros::Subscriber joints_sub_;
  double link_length_;
  int link_num_;
  std::string root_link_name_;
  std::vector<std::string> links_name_;
  tf::TransformListener tf_;
  double tf_rate_;
  ros::Timer  tf_timer_;
  

  void tfFunc(const ros::TimerEvent & e)
  {
    //get transform;
    std::vector<tf::StampedTransform>  transforms;
    transforms.resize(link_num_);

    ros::Duration dur (0.02);

    if (tf_.waitForTransform(root_link_name_, links_name_[link_num_ - 1], ros::Time(0),dur))
      {
        std::vector<tf::Vector3> links_origin;
        for(int i = 0; i < link_num_; i++)
          {
            tf_.lookupTransform(root_link_name_, links_name_[i], ros::Time(0), transforms[i]);
            links_origin.push_back(transforms[i].getOrigin());
          }


        correlationCoefficientCheck(links_origin);
      }
  }

  
public:
  
 CorrelationCoefficient(ros::NodeHandle nh, ros::NodeHandle nhp)
    :nh_(nh), nhp_(nhp)
  {
    nhp_.param("tf_rate", tf_rate_, 60.0);
    nhp_.param("link_num", link_num_, 4);
    nhp_.param("link_length", link_length_, 0.5); //m, hydra prototye!
    nhp_.param("root_link_name", root_link_name_, std::string("/link3_abdomen")); 
    links_name_.resize(link_num_);
    for(int i = 0; i < link_num_; i++)
      {
        std::stringstream ss;
        ss << i + 1;
        links_name_[i] = std::string("/link") + ss.str()  + std::string("_abdomen");
      }

    tf_timer_ = nhp_.createTimer(ros::Duration(1.0 / tf_rate_),
                                           &CorrelationCoefficient::tfFunc, this);
  }


  ~CorrelationCoefficient()
  {
    
  }

};



