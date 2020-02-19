/*
  UTM Geodesic converter
  Created by Lazaro Moratelli Jr, May 2019.

  latitude and longitude in radians
  x, y in meters
*/

#ifndef UTMCONVERTER_H
#define UTMCONVERTER_H

#include <math.h>

#define sa 6378137.000000
#define sb 6356752.314245
#define k0 0.9996
#define pi 3.141592653589793

class UTM {
  private:
    double e2  = pow( pow(sa,2) - pow(sb,2) , 0.5 ) / sb;
    double e2c = pow(e2,2);
    double c   = pow(sa,2) / sb;

  public:
    double Lat, Lon;
    double x, y;
    int    zone_n;
    char   zone_l = 'E';

//-----------------------------------------------------------------------
// Constructor
UTM(){
}
//-----------------------------------------------------------------------
// Convert lat and lon to UTM
// Input angles
void GeoToUTM(double lat_, double lon_){

  int S;
  double deltaS, lat_2, a, epsilon, nu;
  double v, ta, a1, a2, j2, j4, j6, alfa, beta, gama, Bm;

  zone_n = (int) (lon_ * 180/pi / 6 + 31);
  S = zone_n * 6 -183;
  deltaS = lon_ - S * pi/180.0;

  lat_2 = lat_ * 180/pi;

  if (lat_2<-72.0)      {zone_l='C';}
  else if (lat_2<-64.0) {zone_l='D';}
  else if (lat_2<-56.0) {zone_l='E';}
  else if (lat_2<-48.0) {zone_l='F';}
  else if (lat_2<-40.0) {zone_l='G';}
  else if (lat_2<-32.0) {zone_l='H';}
  else if (lat_2<-24.0) {zone_l='J';}
  else if (lat_2<-16.0) {zone_l='K';}
  else if (lat_2<-8.0)  {zone_l='L';}
  else if (lat_2<-0.0)  {zone_l='M';}
  else if (lat_2< 8.0)  {zone_l='N';}
  else if (lat_2< 16.0) {zone_l='P';}
  else if (lat_2< 24.0) {zone_l='Q';}
  else if (lat_2< 32.0) {zone_l='R';}
  else if (lat_2< 40.0) {zone_l='S';}
  else if (lat_2< 48.0) {zone_l='T';}
  else if (lat_2< 56.0) {zone_l='U';}
  else if (lat_2< 64.0) {zone_l='V';}
  else if (lat_2< 72.0) {zone_l='W';}
  else                  {zone_l='x';}

  a = cos(lat_) * sin(deltaS);
  epsilon = 0.5 * log( ( 1.0 +  a) / ( 1.0 - a ) );
  nu = atan( tan(lat_) / cos(deltaS) ) - lat_;

  v  =  c * k0 /  pow( 1.0 + e2c * pow(cos(lat_), 2),0.5);
  ta = ( e2c / 2 ) * pow(epsilon,2) * pow(cos(lat_),2);
  a1 = sin( 2.0 * lat_ );

  a2 = a1 * pow(cos(lat_),2);
  j2 = lat_ + a1 / 2.0 ;
  j4 = ( 3.0 * j2 + a2 ) / 4.0;

  j6 = ( 5.0 * j4  +  a2 * pow(cos(lat_),2) ) / 3.0;
  alfa = 3.0/4.0 * e2c;
  beta = 5.0/3.0 * pow(alfa,2);

  gama = 35.0 / 27.0  * pow(alfa,3);
  Bm = k0 * c * ( lat_ - alfa * j2 + beta * j4 - gama * j6 );

  x = epsilon * v * ( 1.0 + ( ta / 3.0 ) ) + 500000.0;
  y = nu * v * ( 1.0 + ta ) + Bm;

  if (y<0.0){y=9999999.0+y;}
}
//-----------------------------------------------------------------------
// Convert UTM to latitude and longitude
void UTM2Geo(double x_, double y_, int zone_n_,char zone_l_){
  
  zone_n = zone_n_;
  zone_l = zone_l_;
  
  int S;
  double lat_, v, a, a1, a2, j2, j4, j6, alfa, beta, gama, Bm;
  double b, Epsi, Eps, nab, senoheps, Delt, TaO;

  if (zone_l_ <= 'M'){ y_ = y_ - 10000000;}
  x_ = x_ - 500000.0;

  S = ( zone_n * 6 - 183 );
  lat_ =  y_ / ( 6366197.724 * k0 );

  v  =  c * k0 /  pow( 1.0 + e2c * pow(cos(lat_), 2),0.5);

  a = x_ / v;
  a1 = sin( 2.0 * lat_ );
  a2 = a1 * pow(cos(lat_),2);

  j2 = lat_ +  a1 / 2.0;
  j4 = ( 3.0 * j2  + a2 ) / 4.0;
  j6 = ( 5.0 * j4  +  a2 * pow(cos(lat_),2) ) / 3.0;

  alfa = 3.0/4.0 * e2c;
  beta = 5.0/3.0 * pow(alfa,2);
  gama = 35.0 / 27.0  * pow(alfa,3);
  Bm = k0 * c * ( lat_ - alfa * j2 + beta * j4 - gama * j6 );

  b = ( y_ - Bm ) / v;
  Epsi = ( e2c * pow(a,2) / 2.0 ) * pow(cos(lat_),2 );
  Eps = a * ( 1.0 - ( Epsi / 3.0 ) );

  nab = ( b * ( 1.0 - Epsi ) ) + lat_;
  senoheps = ( exp(Eps) - exp(-Eps) ) / 2.0;
  Delt = atan(senoheps / (cos(nab) ) );

  TaO = atan(cos(Delt) * tan(nab));

  Lon = Delt  + S * pi / 180.0;
  Lat =  lat_ + ( 1.0 + e2c* pow(cos(lat_),2) - ( 3.0 / 2.0 ) * e2c * sin(lat_) * cos(lat_) * ( TaO - lat_ ) ) * ( TaO - lat_ );
  
}
};

#endif //UTMCONVERTER_H