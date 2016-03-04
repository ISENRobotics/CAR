#include "ros/ros.h"
#include "std_msgs/String.h"
#include "rc_car/defines.h"
#include "rc_car/Command.h"
#include "rc_car/RSRMsg.h"
#include "rc_car/pwm.h"
#include <iostream>
#include <string>
#include <fstream>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

ros::Timer gps_timer;

void gps_timer_callback(const ros::TimerEvent&)
{
  ROS_INFO("gps_timer_callback triggered");
}

void tTest_sub_callback(const std_msgs::String::ConstPtr& msg, ros::NodeHandle * n)
{
  ROS_INFO("Test received from tTest_sub: %s", msg->data.c_str());

  // A chaque nouvelles coordonnées GPS reçues, démarrer un timer
  // si le timer dépasse une valeure limite, erreur et la voiture s'arrête
  // on peut prendre en compte le nombre de satellite dans l'erreur

  gps_timer = n->createTimer(ros::Duration(2), gps_timer_callback);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Test");

  ros::NodeHandle n;

  ros::Subscriber tTest_sub = n.subscribe<std_msgs::String>("tTest_sub", 1000, boost::bind(&tTest_sub_callback, _1, &n));
  //ros::Publisher tTest_pub = n.advertise<std_msgs::String>("tTest_pub", 1000);

  ros::spin();

  return 0;
}
