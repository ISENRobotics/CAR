#include "ros/ros.h"
#include "std_msgs/String.h"
#include "rc_car/defines.h"
#include "rc_car/Command.h"
#include "rc_car/RSRMsg.h"
#include "rc_car/SwitchMsg.h"
#include "rc_car/pwm.h"
#include <iostream>
#include <string>
#include <fstream>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

using namespace std;

string lecture_fichier()
{
      struct passwd *pw = getpwuid(getuid());

      std::string homedir = pw->pw_dir;
      std::string path = homedir + "/catkin_ws/src/script/pwmswitch.txt";
      
      ifstream fichier(path.c_str(), ios::in);  // on ouvre en lecture
        string contenu="";
        if(fichier)  // si l'ouverture a fonctionné
        {
                  // déclaration d'une chaîne qui contiendra la ligne lue
                getline(fichier, contenu);  // on met dans "contenu" la ligne
                
                fichier.close();
        }
        else
                cerr << "Impossible d'ouvrir le fichier !" << endl;
 
        return contenu;
}


PWM pwmSwitch(lecture_fichier() + "/pwm0");

void switch_PWM_source(const rc_car::SwitchMsgConstPtr& switchMsg, ros::Publisher * tRSR_pub)
{
  rc_car::RSRMsg msg;

  ROS_INFO("iSwitchMode received from tSwitchMode: %s", (switchMsg->modeAuto)?"auto":"manual");

  if(switchMsg->modeAuto)
  {
    if(!(pwmSwitch.setDuty(PWM_SWITCH_PERIOD_AUTO)))
    {
      ROS_ERROR("iSwitchMode unable to set the duty period");
    }
    msg.run = true;
    msg.reset = false;
    tRSR_pub->publish(msg);
  }
  else
  {
    if(!(pwmSwitch.setDuty(PWM_SWITCH_PERIOD_MAN)))
    {
      ROS_ERROR("iSwitchMode unable to set the duty period");
    }
    msg.run = false;
    msg.reset = false;
    tRSR_pub->publish(msg);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "iSwitchMode");

  ros::NodeHandle n;

  ros::Publisher tRSR_pub = n.advertise<rc_car::RSRMsg>("tRSR", 1);
  ros::Subscriber tSwitch_sub = n.subscribe<rc_car::SwitchMsg>("tSwitchMode", 1, boost::bind(&switch_PWM_source, _1, &tRSR_pub));
  //ros::Publisher tError_pub = n.advertise<Error>("tError", 1000);

  ros::spin();

  return 0;
}
