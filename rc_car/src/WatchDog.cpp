#include "ros/ros.h"
#include "rc_car/GPSFix.h"
#include "rc_car/RSRMsg.h"
#include "rc_car/SwitchMsg.h"

ros::WallTimer gps_timer;
double gps_timer_duration = 3;
bool modeAuto = true;

void gps_timer_callback(const ros::WallTimerEvent& a, ros::Publisher * pub)
{
  rc_car::RSRMsg msg;

  msg.run = 0;
  msg.reset = 0;
  pub->publish(msg);

  ROS_ERROR("No data received from GPS since at least %f seconds. Car stoped.", gps_timer_duration);
}

void fix_callback(const rc_car::GPSFix::ConstPtr& msg, ros::NodeHandle * n, ros::Publisher * pub)
{
  rc_car::RSRMsg RSR_msg;
  rc_car::GPSStatus status = msg->status;

  if (status.status!=-1)
  {
    gps_timer.setPeriod(ros::WallDuration(gps_timer_duration), true);
    gps_timer.start();

    if(modeAuto)
    {
      RSR_msg.run = 1;
      RSR_msg.reset = 0;
      pub->publish(RSR_msg);
    }
  }

  ROS_INFO("WatchDog received a message from extended_fix");
}

void RSR_process(const rc_car::RSRMsg::ConstPtr& RSR)
{
  //ROS_INFO("WatchDog received from tRSR: %s %s", (RSR->run)?"run":"stop", (RSR->reset)?"reset":"no_reset");

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

void tSwitchMode_callback(const rc_car::SwitchMsg::ConstPtr& msg)
{
  modeAuto = msg->modeAuto;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "WatchDog");

  ros::NodeHandle n;

  ros::Subscriber tRSR_sub = n.subscribe("tRSR", 1000, RSR_process);
  ros::Subscriber tSwitchMode_sub = n.subscribe("tSwitchMode", 1000, tSwitchMode_callback);
  ros::Publisher tRSR_pub = n.advertise<rc_car::RSRMsg>("tRSR", 1000);
  ros::Subscriber tFix = n.subscribe<rc_car::GPSFix>("fix", 1000, boost::bind(&fix_callback, _1, &n, &tRSR_pub));
  
  //ros::Publisher tError_pub = n.advertise<Error>("tError", 1000);

  if (n.getParam("WatchDog/watchdog_gps_timer", gps_timer_duration))
  {
    ROS_INFO("WatchDog got param /watchdog_gps_timer: %f\n", gps_timer_duration);
  }
  else
  {
    ROS_ERROR("WatchDog failed to get param /watchdog_gps_timer\n");
  }

  gps_timer = n.createWallTimer(ros::WallDuration(gps_timer_duration), boost::bind(&gps_timer_callback, _1, &tRSR_pub), false);

  ros::spin();

  return 0;
}
