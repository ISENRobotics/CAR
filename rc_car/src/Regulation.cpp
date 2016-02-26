#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "rc_car/defines.h"
#include "rc_car/Command.h"
#include "rc_car/RSRMsg.h"
#include "rc_car/distance.h"

using namespace std;




void gps(const sensor_msgs::NavSatFixConstPtr& FIX)
{
  ROS_INFO("gps lat : %f    long : %f", FIX->latitude,FIX->longitude);

  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "iRegulation");

  ros::NodeHandle n;

  ros::Subscriber tfix = n.subscribe("fix", 1000, gps);
  

  ros::spin();

  return 0;
}
