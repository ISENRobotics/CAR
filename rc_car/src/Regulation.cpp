#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "rc_car/defines.h"
#include "rc_car/Command.h"
#include "rc_car/RSRMsg.h"
#include <nav_msgs/Odometry.h>
#include "rc_car/distance.h"
#include <vector>
#include <cmath>


using namespace std;

vector<double> OM(2,0);
vector<double> OA(2,0);
vector<double> OB(2,0);
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
void odome(const nav_msgs::OdometryConstPtr& ODOM)
{
	ROS_INFO("x : %f    y : %f   \n", ODOM->pose.pose.position.x,ODOM->pose.pose.position.y);

OM[0]=ODOM->pose.pose.position.x;
OM[1]=ODOM->pose.pose.position.y;
}

double orientationSouhaitee(vector<double> OM, vector<double> OA, vector<double> OB, double R_MAX){
    // Calcul du vecteur directeur de la droite (AB) passant par les waypoints A et B
    
    double AB;
    vector<double> theta(2,0);
    vector<double> u(2,0);
    vector<double> AM(2,0);
    double e;
    double a;

    AB = sqrt(pow(OB[0] - OA[0],2) + pow(OB[1] - OA[1],2)); // Norme de AB
    
    u[0] =(OB[0] - OA[0])/AB;
    u[1]= (OB[1] - OA[1])/AB; // Vecteur directeur unitaire de (AB)
    
    AM[0] = OM[0] - OA[0]; // Vecteur position du mobile par rapport au point A
    AM[1] = OM[1] - OA[1];

    //e = det([AM, u]); // Distance algébrique entre le point M et la droite (AB)
    e = (AM[0] * u[1]) - (AM[1] *u[0]);
    a = atan2(e,R_MAX) + atan2(u[1],u[0]); // Angle de l'orientation souhaitée
                                          // Somme de l'angle souhaitée par rapport à la droite
                                          // et de l'angle de la droite par rapport à l'angle zéro
    theta[0]=cos(a);
    theta[1]=sin(a);

    double theta_des = atan2(theta[1], theta[0]);
    return theta_des;

 
}

// Fonction de calcul de la commande de l'angle des roues avant
double angleRoues(double theta, double theta_des, double angle_braq_max){
    // Angle désiré des roues avant entre -pi et pi
    double delta_des = fmod(theta_des - theta + M_PI, 2*M_PI) - M_PI;
    double delta;
    // Seuillage de l'angle des roues avant pour limiter à l'angle de braquage maximum
    if (delta_des<(-angle_braq_max)) 
        delta=-angle_braq_max;
    else if (delta_des>(angle_braq_max))
        delta=angle_braq_max;
    else
        delta=delta_des;

    return delta;
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "iRegulation");

  ros::NodeHandle n;

  ros::Subscriber tfix = n.subscribe("fix", 1, gps);

  ros::Subscriber todom = n.subscribe("odom", 1, odome);
  
  double R_max=2.5;
  double theta_des;
  double delta;

  

  OM[0]=2.1100306116261573;
  OM[1]=1.212674851791298;
  OA[0]=1;
  OA[1]=1;
  OB[0]=-2;
  OB[1]=2;
  theta_des=orientationSouhaitee(OM,OA,OB,R_max);

  double angle_braq_max=M_PI/4;
  delta=angleRoues(1.2020815280171306, theta_des, angle_braq_max);


  ROS_INFO("delta: %f    \n", delta);


  	ros::spin();

	

  return 0;
}
