#include "ros/ros.h"
#include <stdio.h>
#include <sstream>
#include <boost/chrono.hpp>
#include <boost/thread/thread.hpp>
#include "TcpInterface.h"
#include <stdint.h>

#include "std_msgs/String.h"
#include "sensor_msgs/FluidPressure.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "smarc_msgs/DVL.h"
#include "smarc_msgs/AltitudeStamped.h"
#include "smarc_msgs/CaptainStatus.h"
#include "smarc_msgs/RudderAngle.h"
#include "smarc_msgs/ThrusterRPM.h"
#include "smarc_msgs/ThrusterRPMStatus.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"

#include <tf2/LinearMath/Quaternion.h>

#define IPADDRESS "192.168.1.177" // IP address of LOLO
#define PORT 8888


TcpInterFace captain;

struct ROSinterface {

  //================== ROS Nodehandle ====================//
  ros::NodeHandle* n;

  //======================================================//
  //================== ROS subscribers ===================//
  //======================================================//

  //TODO
  // heartbeat
  // USBL/RF/etc
  // done
  // abort mission
  // course
  // pitch
  // depth
  // altitude
  // waypoint

  //thrusters
  ros::Subscriber thrusterPort_sub;
  ros::Subscriber thrusterStrb_sub;

  //control surfaces
  ros::Subscriber rudderPort_sub;
  ros::Subscriber rudderStrb_sub;
  ros::Subscriber elevator_sub;

  //======================================================//
  //=================== ROS pubishers ====================//
  //======================================================//
  //thrusters
  ros::Publisher thrusterPort_pub;
  ros::Publisher thrusterStrb_pub;

  //constrol surfaces
  ros::Publisher rudderPort_pub;
  ros::Publisher rudderStrb_pub;
  ros::Publisher elevator_pub;

  //sensors
  ros::Publisher imu_pub;
  ros::Publisher magnetometer_pub;
  ros::Publisher pressure_pub;
  ros::Publisher dvl_pub;
  ros::Publisher gps_pub;

  //control / status
  ros::Publisher status_altitude_pub;
  ros::Publisher status_position_pub;
  ros::Publisher status_twist_pub;

  ros::Publisher control_status_pub;

  //======================================================//
  //=================== ROS callbacks ====================//
  //======================================================//

  void callback_rudderPort(const smarc_msgs::RudderAngle::ConstPtr &_msg) {
    //printf("Port rudder: %f\n", _msg->angle);
    captain.new_package(160); // Control message
    captain.add_float(_msg->angle);
    captain.send_package();
  };

  void callback_rudderStrb(const smarc_msgs::RudderAngle::ConstPtr &_msg) {
    //printf("Strb rudder: %f\n", _msg->angle);
    captain.new_package(161);
    captain.add_float(_msg->angle);
    captain.send_package();
  };

  void callback_elevator(const smarc_msgs::RudderAngle::ConstPtr &_msg) {
    //printf("elevator: %f\n", _msg->angle);
    captain.new_package(162);
    captain.add_float(_msg->angle);
    captain.send_package();
  };

  void callback_thrusterPort(const smarc_msgs::ThrusterRPM::ConstPtr &_msg) {
    //printf("Port Thruster angle received %f\n", _msg->angle);
    captain.new_package(163);
    captain.add_float(_msg->RPM);
    captain.send_package();
  };

  void callback_thrusterStrb(const smarc_msgs::ThrusterRPM::ConstPtr &_msg) {
    //printf("Strb Thruster: %f\n", _msg->angle);
    captain.new_package(164);
    captain.add_float(_msg->RPM);
    captain.send_package();
  };


  void init(ros::NodeHandle* nh) {
    n = nh;
    //==================================//
    //=========== Subscribers ==========//
    //==================================//
    //Thruster
    thrusterPort_sub = n->subscribe<smarc_msgs::ThrusterRPM>("/lolo_auv_1/thrusters/0/input", 1, &ROSinterface::callback_thrusterPort, this);
    thrusterStrb_sub = n->subscribe<smarc_msgs::ThrusterRPM>("/lolo_auv_1/thrusters/1/input", 1, &ROSinterface::callback_thrusterStrb, this);

    //control surfaces
    rudderPort_sub  = n->subscribe<smarc_msgs::RudderAngle>("/lolo_auv_1/fins/0/input", 1, &ROSinterface::callback_rudderPort, this);
    rudderStrb_sub  = n->subscribe<smarc_msgs::RudderAngle>("/lolo_auv_1/fins/1/input", 1, &ROSinterface::callback_rudderStrb, this);
    elevator_sub    = n->subscribe<smarc_msgs::RudderAngle>("/lolo_auv_1/back_fins/0/input", 1, &ROSinterface::callback_elevator, this);

    //==================================//
    //=========== Publishers ===========//
    //==================================//
    //thrusters
    thrusterPort_pub     = n->advertise<smarc_msgs::ThrusterRPMStatus>("/lolo_auv_1/thrusters/0/output", 10);
    thrusterStrb_pub     = n->advertise<smarc_msgs::ThrusterRPMStatus>("/lolo_auv_1/thrusters/1/output", 10);

    //constrol surfaces
    rudderPort_pub    = n->advertise<smarc_msgs::RudderAngle>("/lolo_auv_1/fins/0/output", 10);
    rudderStrb_pub    = n->advertise<smarc_msgs::RudderAngle>("/lolo_auv_1/fins/1/output", 10);
    elevator_pub      = n->advertise<smarc_msgs::RudderAngle>("/lolo_auv_1/back_fins/0/output", 10);

    //sensors
    imu_pub           = n->advertise<sensor_msgs::Imu>("/test/imu", 10);
    magnetometer_pub  = n->advertise<sensor_msgs::MagneticField>("/test/mag", 10);
    dvl_pub           = n->advertise<smarc_msgs::DVL>("/test/dvl", 10);
    gps_pub           = n->advertise<sensor_msgs::NavSatFix>("/test/gps", 10);
    pressure_pub      = n->advertise<sensor_msgs::FluidPressure>("/test/pressure", 10);

    //control / status
    status_altitude_pub = n->advertise<smarc_msgs::AltitudeStamped>("/test/status/altitude", 10);
    status_position_pub = n->advertise<geometry_msgs::PoseWithCovarianceStamped>("/test/status/position",10);
    status_twist_pub    = n->advertise<geometry_msgs::TwistWithCovarianceStamped>("test/status/twist",10);
    control_status_pub  = n->advertise<smarc_msgs::CaptainStatus>("/test/control_status", 10);

  }
};

ROSinterface rosInterface;

//======================================================//
//========= Captain interface callback =================//
//======================================================//
int captain_msgs = 0;
void callback_captain() {
  int msgID = captain.messageID();
  //printf("Received message from captain %d\n", msgID);
  switch (msgID) {
    case 8: { //status
      uint64_t timestamp    = captain.parse_llong();
      uint64_t sec = timestamp / 1000000;
      uint64_t usec = timestamp % 1000000;
      uint32_t sequence     = captain.parse_long();

      float lat         = captain.parse_float();
      float lon         = captain.parse_float();
      float depth       = captain.parse_float();
      float altitude    = captain.parse_float();
      float sogX        = captain.parse_float();
      float sogY        = captain.parse_float();
      float sogZ        = captain.parse_float();
      float rotX        = captain.parse_float();
      float rotY        = captain.parse_float();
      float rotZ        = captain.parse_float();
      float pitch       = captain.parse_float();
      float roll        = captain.parse_float();
      float yaw         = captain.parse_float();

      //publish position
      geometry_msgs::PoseWithCovarianceStamped pos_msg;
      pos_msg.header.stamp = ros::Time(sec,usec*1000);
      pos_msg.header.seq = sequence;
      pos_msg.pose.pose.position.x = lat;
      pos_msg.pose.pose.position.y = lon;
      pos_msg.pose.pose.position.z = depth;

      tf2::Quaternion q; q.setRPY(roll, pitch, yaw); q.normalize();
      pos_msg.pose.pose.orientation.x = q[0];
      pos_msg.pose.pose.orientation.y = q[1];
      pos_msg.pose.pose.orientation.z = q[2];
      pos_msg.pose.pose.orientation.w = q[3];

      rosInterface.status_position_pub.publish(pos_msg);

      geometry_msgs::TwistWithCovarianceStamped twist_msg;
      twist_msg.header.stamp = ros::Time(sec,usec*1000);
      twist_msg.header.seq = sequence;
      twist_msg.twist.twist.linear.x = sogX;
      twist_msg.twist.twist.linear.y = sogY;
      twist_msg.twist.twist.linear.z = sogZ;
      twist_msg.twist.twist.angular.x = rotX;
      twist_msg.twist.twist.angular.y = rotY;
      twist_msg.twist.twist.angular.z = rotZ;
      rosInterface.status_twist_pub.publish(twist_msg);

      //publish altitude
      smarc_msgs::AltitudeStamped alt_msg;
      alt_msg.header.stamp = ros::Time(sec,usec*1000);
      alt_msg.header.seq = sequence;
      alt_msg.altitude = altitude;
      rosInterface.status_altitude_pub.publish(alt_msg);
    }
    break;
    case 9: { //control
      uint64_t timestamp    = captain.parse_llong();
      uint64_t sec = timestamp / 1000000;
      uint64_t usec = timestamp % 1000000;
      uint32_t sequence     = captain.parse_long();

      float source  = captain.parse_byte();
      float targetWaypoint_lat = captain.parse_float();
      float targetWaypoint_lon =  captain.parse_float();
      float targetYaw      = captain.parse_float();
      float targetPitch    = captain.parse_float();
      float targetSpeed    = captain.parse_float();
      float targetRPM      = captain.parse_float();
      float targetDepth    = captain.parse_float();
      float targetAltitude =  captain.parse_float();

      smarc_msgs::CaptainStatus msg;
      msg.header.stamp = ros::Time(sec,usec*1000);
      msg.header.seq = sequence;
      rosInterface.control_status_pub.publish(msg);
    }
    break;
    case 10: { //port rudder
      uint64_t timestamp    = captain.parse_llong();
      uint32_t sequence     = captain.parse_long();
      float target_angle    = captain.parse_float();
      float current_angle   = captain.parse_float();

      smarc_msgs::RudderAngle msg;
      uint64_t sec = timestamp / 1000000;
      uint64_t usec = timestamp % 1000000;
      msg.header.stamp = ros::Time(sec,usec*1000);
      msg.header.seq = sequence;
      msg.angle = current_angle;
      rosInterface.rudderPort_pub.publish(msg);
    }
    break;
    case 11: { //strb rudder
      uint64_t timestamp    = captain.parse_llong();
      uint32_t sequence     = captain.parse_long();
      float target_angle    = captain.parse_float();
      float current_angle   = captain.parse_float();

      smarc_msgs::RudderAngle msg;
      uint64_t sec = timestamp / 1000000;
      uint64_t usec = timestamp % 1000000;
      msg.header.stamp = ros::Time(sec,usec*1000);
      msg.header.seq = sequence;
      msg.angle = current_angle;
      rosInterface.rudderStrb_pub.publish(msg);
    }
    break;
    case 12: { //elevator
      uint64_t timestamp    = captain.parse_llong();
      uint32_t sequence     = captain.parse_long();
      float target_angle    = captain.parse_float();
      float current_angle   = captain.parse_float();

      smarc_msgs::RudderAngle msg;
      uint64_t sec = timestamp / 1000000;
      uint64_t usec = timestamp % 1000000;
      msg.header.stamp = ros::Time(sec,usec*1000);
      msg.header.seq = sequence;
      msg.angle = current_angle;
      rosInterface.elevator_pub.publish(msg);
    }
    break;
    case 13: { //port thruster
      uint64_t timestamp    = captain.parse_llong();
      uint32_t sequence     = captain.parse_long();
      uint64_t sec = timestamp / 1000000;
      uint64_t usec = timestamp % 1000000;

      float rpm             = captain.parse_float();
      float current         = captain.parse_float();
      float torque          = captain.parse_float();
      float energy          = captain.parse_float();
      float voltage         = captain.parse_float();

      smarc_msgs::ThrusterRPMStatus msg;
      msg.thrusterRPM.header.stamp = ros::Time(sec,usec*1000);
      msg.thrusterRPM.header.seq = sequence;
      msg.thrusterRPM.RPM = rpm;
      msg.current = current;
      msg.torque = torque;
      msg.temperature = 0;
      rosInterface.thrusterPort_pub.publish(msg);
    }
    break;
    case 14: { //strb thruster
      uint64_t timestamp    = captain.parse_llong();
      uint32_t sequence     = captain.parse_long();
      uint64_t sec = timestamp / 1000000;
      uint64_t usec = timestamp % 1000000;

      float rpm             = captain.parse_float();
      float current         = captain.parse_float();
      float torque          = captain.parse_float();
      float energy          = captain.parse_float();
      float voltage         = captain.parse_float();

      smarc_msgs::ThrusterRPMStatus msg;
      msg.thrusterRPM.header.stamp = ros::Time(sec,usec*1000);
      msg.thrusterRPM.header.seq = sequence;
      msg.thrusterRPM.RPM = rpm;
      msg.current = current;
      msg.torque = torque;
      msg.temperature = 0;
      rosInterface.thrusterStrb_pub.publish(msg);
    }
    break;
    case 15: { //battery
    }
    break;
    case 16: { //DVL
      uint64_t timestamp    = captain.parse_llong();
      uint64_t sec = timestamp / 1000000;
      uint64_t usec = timestamp % 1000000;
      uint32_t sequence     = captain.parse_long();

      float sogX           = captain.parse_float();
      float sogY           = captain.parse_float();
      float sogZ           = captain.parse_float();
      float stwX           = captain.parse_float();
      float stwY           = captain.parse_float();
      float stwZ           = captain.parse_float();
      float range1         = captain.parse_float();
      float range2         = captain.parse_float();
      float range3         = captain.parse_float();
      float range4         = captain.parse_float();
      float range = 0.25*(range1+range2+range3+range4);

      smarc_msgs::DVL msg;
      msg.header.stamp = ros::Time(sec,usec*1000);
      msg.header.seq = sequence;
      msg.velocity.x = sogX;
      msg.velocity.y = sogY;
      msg.velocity.z = sogZ;
      msg.velocity_reference = smarc_msgs::DVL::VELOCITY_REFERENCE_BOTTOM;
      msg.range = range;
      rosInterface.dvl_pub.publish(msg);
    }
    break;
    case 17: { //GPS
      uint64_t timestamp    = captain.parse_llong();
      uint64_t sec = timestamp / 1000000;
      uint64_t usec = timestamp % 1000000;
      uint32_t sequence     = captain.parse_long();

      float lat           = captain.parse_float();
      float lon           = captain.parse_float();
      float cog           = captain.parse_float();
      float sog           = captain.parse_float();

      sensor_msgs::NavSatFix msg;
      msg.header.stamp = ros::Time(sec,usec*1000);
      msg.header.seq = sequence;
      msg.latitude = lat;
      msg.longitude = lon;
      rosInterface.gps_pub.publish(msg);
    }
    break;
    case 18: { //IMU
      uint64_t timestamp    = captain.parse_llong();
      uint64_t sec = timestamp / 1000000;
      uint64_t usec = timestamp % 1000000;
      uint32_t sequence     = captain.parse_long();

      float pitch           = captain.parse_float();
      float roll            = captain.parse_float();
      float yaw             = captain.parse_float();

      float accX            = captain.parse_float();
      float accY            = captain.parse_float();
      float accZ            = captain.parse_float();

      float rotX            = captain.parse_float();
      float rotY            = captain.parse_float();
      float rotZ            = captain.parse_float();

      sensor_msgs::Imu msg;
      msg.header.stamp = ros::Time(sec,usec*1000);
      msg.header.seq = sequence;

      tf2::Quaternion q; q.setRPY(roll, pitch, yaw); q.normalize();
      msg.orientation.x = q[0];
      msg.orientation.y = q[1];
      msg.orientation.z = q[2];
      msg.orientation.w = q[3];

      msg.angular_velocity.x = rotX;
      msg.angular_velocity.y = rotY;
      msg.angular_velocity.z = rotZ;

      msg.linear_acceleration.x = accX;
      msg.linear_acceleration.y = accY;
      msg.linear_acceleration.z = accZ;

      rosInterface.imu_pub.publish(msg);
    }
    break;
    case 19: { //MAG
      uint64_t timestamp    = captain.parse_llong();
      uint64_t sec = timestamp / 1000000;
      uint64_t usec = timestamp % 1000000;
      uint32_t sequence     = captain.parse_long();
      float magX            = captain.parse_float();
      float magY            = captain.parse_float();
      float magZ            = captain.parse_float();

      sensor_msgs::MagneticField msg;
      msg.header.stamp = ros::Time(sec,usec*1000);
      msg.header.seq = sequence;
      msg.magnetic_field.x = magX;
      msg.magnetic_field.y = magY;
      msg.magnetic_field.z = magZ;
      //msg.magnetic_field_covariance = 0;
      rosInterface.magnetometer_pub.publish(msg);
    }
    break;
    case 20: { //PRESSURE
      uint64_t timestamp    = captain.parse_llong();
      uint64_t sec = timestamp / 1000000;
      uint64_t usec = timestamp % 1000000;
      uint32_t sequence     = captain.parse_long();
      float pressure        = captain.parse_float();

      sensor_msgs::FluidPressure msg;
      msg.header.stamp = ros::Time(sec,usec*1000);
      msg.header.seq = sequence;
      msg.fluid_pressure = pressure;
      msg.variance = 0;
      rosInterface.pressure_pub.publish(msg);
    }
    break;
  }
}


int main(int argc, char *argv[]) {

  printf("Set callback\n");
  captain.setCallback(callback_captain);

  printf("setup\n");
  captain.setup(IPADDRESS, PORT); //Dont start until captain is connected and running.

  if(captain.connected())
    printf("main::Connected\n");
  else {
    printf("main::Not connceted\n");
    //return 0;
  }

  printf("main::ros init\n");

  ros::init(argc,argv, "CaptainInterface");

  ros::NodeHandle n;
  //Init subscribers and publishers
  rosInterface.init(&n);

  printf("Loop starting\n");
  ros::Rate loop_rate(1000);
  while(ros::ok()) {
    captain.loop(); //should be improved.
    ros::spinOnce();
  }
}
