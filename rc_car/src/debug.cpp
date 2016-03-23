#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include "rc_car/debugmsg.h"
#include <string>
#include <fstream>
#include <iostream>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

using namespace std;


void debug(const rc_car::debugmsg::ConstPtr& debmsg)
{
	ROS_INFO("%ld,%10.10f,%10.10f,%10.10f,%10.10f,%10.10f,%10.10f,%10.10f,%10.10f,%10.10f,%10.10f,%10.10f\n", debmsg->lat,debmsg->lon, debmsg->index,debmsg->OM1,debmsg->OM2,debmsg->OA1,debmsg->OA2,debmsg->OB1,debmsg->OB2,debmsg->theta,debmsg->thetades,debmsg->delta);
 
 struct passwd *pw = getpwuid(getuid());
  std::string homedir = pw->pw_dir;
  std::string path = homedir + "/catkin_ws/src/script/debug.txt";
  
/* ofstream fichier;

 fichier.open(path.c_str(), ofstream::out | std::ofstream::app); 

        if(fichier)
        {
               
                fichier << debmsg->index << "," << debmsg->OM1 << "/" << debmsg->OM2 << "," << debmsg->OA1 << "/" << debmsg->OA2 << "," << debmsg->OB1 << "/" <<debmsg->OB2 << "," << debmsg->theta << "," << debmsg->thetades<< "," << debmsg->a << "," << debmsg->delta << endl;
                
                fichier.close();
        }
        else
                cerr << "Impossible d'ouvrir le fichier !" << endl;
 */
       FILE * fp;

   fp = fopen (path.c_str(), "a");
   fprintf(fp, "%ld,%10.10f,%10.10f,%10.10f,%10.10f,%10.10f,%10.10f,%10.10f,%10.10f,%10.10f,%10.10f,%10.10f\n",debmsg->index,debmsg->lat,debmsg->lon,debmsg->OM1,debmsg->OM2,debmsg->OA1,debmsg->OA2,debmsg->OB1,debmsg->OB2,debmsg->theta,debmsg->thetades,debmsg->delta);
 
   
   fclose(fp);

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "iDebug");

  ros::NodeHandle n;

  

ros::Subscriber debugmsg_pub = n.subscribe("tDebug", 100, debug);
   


ros::spin();


  return 0;
}
