#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "rc_car/defines.h"
#include "rc_car/Command.h"
#include "rc_car/RSRMsg.h"
#include "rc_car/debugmsg.h"
#include "rc_car/SwitchMsg.h"
#include "rc_car/waypoint.h"
#include <nav_msgs/Odometry.h>
#include <rc_car/YPR.h>
#include <vector>
#include <cmath>


using namespace std;
const double RADIANS_PER_DEGREE = M_PI/180.0;
const double DEGREES_PER_RADIAN = 180.0/M_PI;
vector<double> OM_GLOB(2,0);
double latglob,longlob;
ros::Publisher debugmsg_pub;

double thetaglobal=0;
int indexglobal=1;
bool mode=false;

void fixll(const sensor_msgs::NavSatFixConstPtr& fix) {
 
 ROS_INFO("lat : %f    longi : %f   \n", fix->latitude , fix->longitude);

  latglob = fix->latitude;
  longlob = fix->longitude;
}

void odome(const nav_msgs::OdometryConstPtr& ODOM)
{
	ROS_INFO("x : %f    y : %f   \n", ODOM->pose.pose.position.x,ODOM->pose.pose.position.y);

OM_GLOB[0]=ODOM->pose.pose.position.x;
OM_GLOB[1]=-ODOM->pose.pose.position.y;//- car esating 
}

void imu(const rc_car::YPRConstPtr& YPR)
{
	ROS_INFO("Y : %f    P : %f  R: %f \n", YPR->Y,YPR->P,YPR->R);
 	thetaglobal=YPR->Y*RADIANS_PER_DEGREE;
  ROS_INFO("Y en rad: %f   \n", thetaglobal);
  
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

double orientationSouhaitee(vector<double> OM, vector<double> OA, vector<double> OB, double R_MAX,double theta, double angle_braq_max, double lat, double lon){
    // Calcul du vecteur directeur de la droite (AB) passant par les waypoints A et B
    
    double AB;
    vector<double> u(2,0);
    vector<double> AM(2,0);
    double e;
    

    AB = sqrt(pow(OB[0] - OA[0],2) + pow(OB[1] - OA[1],2)); // Norme de AB
    
    u[0] =(OB[0] - OA[0])/AB;
    u[1]= (OB[1] - OA[1])/AB; // Vecteur directeur unitaire de (AB)
    
    AM[0] = OM[0] - OA[0]; // Vecteur position du mobile par rapport au point A
    AM[1] = OM[1] - OA[1];

    //e = det([AM, u]); // Distance algébrique entre le point M et la droite (AB)
    e = (AM[0] * u[1]) - (AM[1] *u[0]);
    double theta_des = atan(e/R_MAX) + atan2(u[1],u[0]);
    //a = atan(e/R_MAX) + atan(u[1]/u[0]); // Angle de l'orientation souhaitée
                                              // Somme de l'angle souhaitée par rapport à la droite
                                              // et de l'angle de la droite par rapport à l'angle zéro
    

    double delta_des = fmod(fmod(theta_des - theta + M_PI, 2*M_PI)+2*M_PI,2*M_PI) - M_PI;
    double delta;
    // Seuillage de l'angle des roues avant pour limiter à l'angle de braquage maximum
    if (delta_des<(-angle_braq_max)) 
        delta=-angle_braq_max;
    else if (delta_des>(angle_braq_max))
        delta=angle_braq_max;
    else
        delta=delta_des;

//publish debug msg
      rc_car::debugmsg debmsg;
      
      debmsg.index = indexglobal;
      debmsg.lat = lat;
      debmsg.lon = lon;
      debmsg.OM1 = OM[0];
      debmsg.OM2 = OM[1];
      debmsg.OA1 = OA[0];
      debmsg.OA2 = OA[1];
      debmsg.OB1 = OB[0];
      debmsg.OB2 = OB[1];
      debmsg.theta=theta*DEGREES_PER_RADIAN;
      debmsg.thetades=theta_des*DEGREES_PER_RADIAN;
      debmsg.delta=delta*DEGREES_PER_RADIAN;
 
      ROS_INFO("pub DEBUGGGGGGG");
       
     debugmsg_pub.publish(debmsg);

      indexglobal=indexglobal+1;


    return delta;
}




int main(int argc, char **argv)
{
  ros::init(argc, argv, "iRegulation");

  ros::NodeHandle n;

  
 	ros::ServiceClient client = n.serviceClient<rc_car::waypoint>("waypoint");
  ros::Subscriber fix_sub = n.subscribe("fix", 1, fixll);
 	 ros::Subscriber todom = n.subscribe("odom", 1, odome);
 	 ros::Subscriber timu = n.subscribe("imu", 1, imu);
 	 ros::Subscriber tswitch = n.subscribe("tSwitchMode", 1, Switch);
   ros::Publisher command_pub = n.advertise<rc_car::Command>("tCommand", 1);
   debugmsg_pub = n.advertise<rc_car::debugmsg>("tDebug", 100);


   ros::Rate loop_rate(8);
   rc_car::Command cmd;
  double Rayon_max=1;
  double Couloir_max=0.5;
  double theta_des;
  double delta;
vector<double> OA(2,0);
vector<double> OB(2,0);
  int debut = 1;

rc_car::waypoint srv;

ROS_INFO("deb reg");
int count = 0;
    while (ros::ok())
    {
ROS_INFO("boucle %d",mode);
if (mode){
  ROS_INFO("modeAuto count %d",count);

	
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

 if(debut){
  OA[0]=OM_GLOB[0];
  OA[1]=OM_GLOB[1];
  debut = 0;
}


  OB[0]=srv.response.x;
  OB[1]=srv.response.y;

  double angle_braq_max=M_PI/4;


  delta=orientationSouhaitee(OM_GLOB,OA,OB,Couloir_max,thetaglobal,angle_braq_max,latglob,longlob)*DEGREES_PER_RADIAN;

  cmd.dir=-delta;
  cmd.speed=1;
  command_pub.publish(cmd);

  if (critereDist(OM_GLOB, OB, Rayon_max)<=0 ){
        count++;
        //mode =0;
        cmd.dir=0;
        cmd.speed=0;
        command_pub.publish(cmd);
          OA[0]=OB[0];
          OA[1]=OB[1];
        ROS_INFO("count++");
      }
    else if (criterePerp(OM_GLOB, OA, OB)>=0){
      //  OA[0]=OM_GLOB[0];
      //  OA[1]=OM_GLOB[1];
        ROS_INFO("criterePerp");
    }


 ROS_INFO("OM1: %f  OM2: %f OA1: %f OA1: %f OB1: %f OB2: %f  \n", OM_GLOB[0],OM_GLOB[1],OA[0],OA[1],OB[0],OB[1]);

ROS_INFO("    delta en deg: %f    \n",delta); 


  	
  	
}
ros::spinOnce();
loop_rate.sleep();
	}

  return 0;
}
