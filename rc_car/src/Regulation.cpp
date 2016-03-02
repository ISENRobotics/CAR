#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "rc_car/defines.h"
#include "rc_car/Command.h"
#include "rc_car/RSRMsg.h"
#include "rc_car/SwitchMsg.h"
#include "rc_car/waypoint.h"
#include <nav_msgs/Odometry.h>
#include <rc_car/YPR.h>
#include <vector>
#include <cmath>


using namespace std;

vector<double> OM(2,0);
vector<double> OA(2,0);
vector<double> OB(2,0);
double lat=0;
double lon=0;
double theta=0;
bool mode=false;

void odome(const nav_msgs::OdometryConstPtr& ODOM)
{
	ROS_INFO("x : %f    y : %f   \n", ODOM->pose.pose.position.x,ODOM->pose.pose.position.y);

OM[0]=ODOM->pose.pose.position.x;
OM[1]=ODOM->pose.pose.position.y;
}

void imu(const rc_car::YPRConstPtr& YPR)
{
	ROS_INFO("Y : %f    P : %f  R: %f \n", YPR->Y,YPR->P,YPR->R);
 	theta=YPR->Y;

}

void Switch(const rc_car::SwitchMsgConstPtr& switchmsg)
{
	ROS_INFO("auto : %d    \n", switchmsg->modeAuto);
 	mode =  switchmsg->modeAuto;

}

// Fonction de calcul du critère de perpendicularité, produit scalaire de AB avec BM
double criterePerp(vector<double> OM, vector<double> OA, vector<double> OB){
    vector<double> AB(2) ;  // Vecteur AB
    vector<double> BM(2) ; // Vecteur BM
    double res;
    AB[0]=OB[0]-OA[0];
    BM[0]=OM[0]-OB[0];
    AB[1]=OB[1]-OA[1];
    BM[1]=OM[1]-OB[1];
    res = AB[0]*BM[0]+AB[1]*BM[1]; // Produit scalaire de AB avec BM
    return res;
}

// Fonction de calcul du critère de distance
double critereDist(vector<double> OM, vector<double> OB, double R_MAX){
    double res = pow((OM[0] - OB[0]),2) + pow((OM[1] - OB[1]),2) - pow(R_MAX,2);
    return res;
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
    double delta_des = fmod(fmod(theta_des - theta + M_PI, 2*M_PI)+2*M_PI,2*M_PI) - M_PI;
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

  
 	ros::ServiceClient client = n.serviceClient<rc_car::waypoint>("waypoint");
 	 ros::Subscriber todom = n.subscribe("odom", 1, odome);
 	 ros::Subscriber timu = n.subscribe("imu", 1, imu);
 	 ros::Subscriber tswitch = n.subscribe("tSwitchMode", 1, Switch);
   ros::Publisher command_pub = n.advertise<rc_car::Command>("tCommand", 10);

   ros::Rate loop_rate(10);
   rc_car::Command cmd;
  double R_max=0.5;
  double theta_des;
  double delta;

  

rc_car::waypoint srv;


int count = 0;
    while (ros::ok())
    {

if (mode){

	
   srv.request.nb = count;
   
   if (client.call(srv))
   {
        ROS_INFO("x: %f   y: %f", srv.response.x,srv.response.y);
        
   }
   else
   {
    ROS_ERROR("Failed to call service waypoint");
    return 1;
   }

 if(srv.request.nb == 0){
  OA[0]=OM[0];
  OA[1]=OM[1];
}else{
	OA[0]=OB[0];
  OA[1]=OB[1];
}
  OB[0]=srv.response.x;
  OB[1]=srv.response.y;



  theta_des=orientationSouhaitee(OM,OA,OB,R_max);

  double angle_braq_max=M_PI/4;
  delta=angleRoues(theta, theta_des, angle_braq_max);
  cmd.dir=delta;
  cmd.speed=1;
  command_pub.publish(cmd);

  if (critereDist(OM, OB, R_max)<=0 ){
        count++;
      }
    else if (criterePerp(OM, OA, OB)>=0){
        OA[0]=OM[0];
        OA[1]=OM[1];
    }


 ROS_INFO("OM1: %f  OM2: %f OA1: %f OA1: %f OB1: %f OB2: %f  \n", OM[0],OM[1],OA[0],OA[1],OB[0],OB[1]);
  ROS_INFO("theta : %f    delta: %f    \n", theta,delta);


  	ros::spinOnce();
  	
}
loop_rate.sleep();
	}

  return 0;
}
