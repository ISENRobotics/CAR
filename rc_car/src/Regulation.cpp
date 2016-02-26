#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "rc_car/defines.h"
#include "rc_car/Command.h"
#include "rc_car/RSRMsg.h"
#include "rc_car/distance.h"

using namespace std;

double lat=0;
double lon=0;
DISTANCE dis;

void gps(const sensor_msgs::NavSatFixConstPtr& FIX)
{
  double d = dis.calculeDistance(FIX->latitude,FIX->longitude,lat,lon);
  ROS_INFO("gps lat : %f    long : %f   distance : %f\n", FIX->latitude,FIX->longitude,d);

  lat = FIX->latitude;
  lon = FIX->longitude;
  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "iRegulation");

  ros::NodeHandle n;

  ros::Subscriber tfix = n.subscribe("fix", 1000, gps);
  

  ros::spin();

  return 0;
}
