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

void refreshPWM_DCmotor(const rc_car::CommandConstPtr& cmd)
{
  float dutyPeriod = 0.0;

  ROS_INFO("iDCmotor received from tCommand: %f", cmd->speed);

  // A chaque nouvelles coordonnées GPS reçues, démarrer un timer
  // si le timer dépasse une valeure limite, erreur et la voiture s'arrête
  // on peut prendre en compte le nombre de satellite dans l'erreur

}

void RSR_process(const rc_car::RSRMsgConstPtr& RSR)
{
  ROS_INFO("iDCmotor received from tRSR: %s %s", (RSR->run)?"run":"stop", (RSR->reset)?"reset":"no_reset");

  if(!(RSR->run))
  {
    if(!(pwmDCmotor.setRunningState(false)))
    {
      ROS_ERROR("iDCmotor unable to disable the DCmotor");
    }
  }
  else
  {
    if(!(pwmDCmotor.setRunningState(true)))
    {
      ROS_ERROR("iDCmotor unable to enable the DCmotor");
    }
  }

  if(RSR->reset)
  {
    if(!(pwmDCmotor.setDuty(0)))
    {
      ROS_ERROR("iDCmotor unable to reset the duty period");
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "iWatchDog");

  ros::NodeHandle n;

  ros::Subscriber tRSR_sub = n.subscribe("tRSR", 1000, RSR_process);
  ros::Subscriber tCommand_sub = n.subscribe("extended_fix", 1000, refreshPWM_DCmotor);
  ros::Publisher tRSR_pub = n.advertise<RSRMsg>("tRSR", 1000);
  //ros::Publisher tError_pub = n.advertise<Error>("tError", 1000);



  ros::spin();

  return 0;
}
