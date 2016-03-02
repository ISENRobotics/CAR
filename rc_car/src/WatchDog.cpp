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

ros::Timer gps_timer; // Préférer un ros::WallTimer ?
double gps_timer_duration = 2;

void gps_timer_callback(const ros::TimerEvent& a)
{
  ROS_INFO("gps_timer_callback triggered");
}

void tTest_callback(const std_msgs::String::ConstPtr& msg, ros::NodeHandle * n)
{
  ROS_INFO("iWatchDog received from tTest: %s", msg->data.c_str());

  // A chaque nouvelles coordonnées GPS reçues, démarrer un timer
  // si le timer dépasse une valeure limite, erreur et la voiture s'arrête
  // on peut prendre en compte le nombre de satellite dans l'erreur

  gps_timer.setPeriod(ros::Duration(gps_timer_duration), true); // oneshot timer
  gps_timer.start();
}

void RSR_process(const rc_car::RSRMsg::ConstPtr& RSR)
{
  ROS_INFO("iWatchDog received from tRSR: %s %s", (RSR->run)?"run":"stop", (RSR->reset)?"reset":"no_reset");

  if(!(RSR->run))
  {

  }
  else
  {

  }

  if(RSR->reset)
  {

  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "iWatchDog");

  ros::NodeHandle n;

  ros::Subscriber tRSR_sub = n.subscribe("tRSR", 1000, RSR_process);
  ros::Subscriber tCommand_sub = n.subscribe<std_msgs::String>("tTest", 1000, boost::bind(&tTest_callback, _1, &n));
  ros::Publisher tRSR_pub = n.advertise<rc_car::RSRMsg>("tRSR", 1000);
  //ros::Publisher tError_pub = n.advertise<Error>("tError", 1000);

  if (n.getParam("iWatchDog/watchdog_gps_timer", gps_timer_duration))
  {
    ROS_INFO("iWatchDog got param /watchdog_gps_timer: %f\n", gps_timer_duration);
  }
  else
  {
    ROS_ERROR("Failed to get param /watchdog_gps_timer\n");
  }

  gps_timer = n.createTimer(ros::Duration(gps_timer_duration), gps_timer_callback, false);

  ros::spin();

  return 0;
}
