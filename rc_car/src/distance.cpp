#include "rc_car/distance.h"
#include <math.h>

#define pi 3.14159265358979323846

DISTANCE::DISTANCE()
{
}

double DISTANCE::deg2rad(double deg) {
  return (deg * pi / 180);
}

double DISTANCE::rad2deg(double rad) {
  return (rad * 180 / pi);
}

double DISTANCE::calculeDistance(double lat1, double lon1, double lat2, double lon2) {
  double theta, dist;
  theta = lon1 - lon2;
  dist = sin(deg2rad(lat1)) * sin(deg2rad(lat2)) + cos(deg2rad(lat1)) * cos(deg2rad(lat2)) * cos(deg2rad(theta));
  dist = acos(dist);
  dist = rad2deg(dist);
  dist = dist * 60 * 1.1515;

  //distance en metre
  dist = dist * 1609.344;

  
  return (dist);
}




