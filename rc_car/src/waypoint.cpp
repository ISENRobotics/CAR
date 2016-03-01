#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "rc_car/defines.h"
#include "rc_car/waypoint.h"
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <string>
#include <fstream>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
#include <vector>
#include <rc_car/conversions.h>




using namespace gps_common;
using namespace std;

vector<double> x,y;

vector<string> lecture_waypoint()
{
      struct passwd *pw = getpwuid(getuid());
      vector<string> position;
      std::string homedir = pw->pw_dir;
      std::string path = homedir + "/catkin_ws/src/script/waypoint.txt";
  
      ifstream fichier(path.c_str(), ios::in);  // on ouvre en lecture
        
        string contenu="";
        if(fichier)  // si l'ouverture a fonctionné
        {
                  // déclaration d'une chaîne qui contiendra la ligne lue
                  while(getline(fichier, contenu)) { // on met dans "contenu" la ligne
                
                position.push_back(contenu);
                
                //cerr << "lat : " << contenu << "  lon : " << endl;
              }
              fichier.close();

        }
        else
                cerr << "Impossible d'ouvrir le fichier !" << endl;
 
        return position;
}

bool waypoint(rc_car::waypoint::Request  &req,
          rc_car::waypoint::Response &res)
 {
   res.x = x[req.nb];
   res.y = y[req.nb];
   ROS_INFO("request: nb=%d", (int)req.nb);
   ROS_INFO("sending back response: x[%f]  y[%f]", res.x,res.y);
   return true;
}

//void gps(const sensor_msgs::NavSatFixConstPtr& FIX)
//{
 
//}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "iWayPoint");

  ros::NodeHandle n;
  ros::ServiceServer service = n.advertiseService("waypoint", waypoint);
 // ros::Subscriber tfix = n.subscribe("fix", 1, gps);

  //ros::Subscriber todom = n.subscribe("odom", 1, odome);
  


  double northing, easting;
  std::string zone;

  
vector<string> pos;


pos = lecture_waypoint();
string lat,lon;
int i;

for ( i = 0;i< pos.size();i++){
  lat = pos[i].substr(0,pos[i].find('/'));
  lon = pos[i].substr(pos[i].find('/')+1);
  LLtoUTM(atof(lat.c_str()), atof(lon.c_str()), northing, easting, zone);
  x.push_back(northing);
  y.push_back(-easting);
  //cerr << "lat : " << northing << "lon : " << easting << endl;
}

    

  	ros::spin();

	

  return 0;
}
