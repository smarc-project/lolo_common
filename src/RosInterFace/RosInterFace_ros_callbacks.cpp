#include "captain_interface/RosInterFace/RosInterFace.h"
#include "captain_interface/UTM.h"
extern UTM utmConverter_fromutm;

void RosInterFace::ros_callback_heartbeat(const std_msgs::Empty::ConstPtr &_msg) {
  captain->new_package(SC_HEARTBEAT); // Heartbeat message
  captain->send_package();
};

void RosInterFace::ros_callback_abort(const std_msgs::Empty::ConstPtr &_msg) {
  //TODO decide what happens.
};


void RosInterFace::ros_callback_done(const std_msgs::Empty::ConstPtr &_msg) {
  //Scientist is done and wants captain to take back control of the vehicle
};


void RosInterFace::ros_callback_waypoint(const cola2_msgs::DecimalLatLon::ConstPtr &_msg) {
  double lat = _msg->latitude;
  double lon = _msg->longitude;
  captain->new_package(SC_SET_TARGET_WAYPOINT); // set target waypoint
  captain->add_double(lat);
  captain->add_double(lon);
  captain->send_package();
};

/*
void RosInterFace::ros_callback_UTMwaypoint(const ??::UTMpoint::ConstPtr &_msg) {
  //Convert
  utmConverter_fromutm.UTM2Geo(_msg->easting, _msg->northing, _msg->zone, _msg->band);
  captain->new_package(SC_SET_TARGET_WAYPOINT); // set target waypoint
  captain->add_double(utmConverter_fromutm.Lat);
  captain->add_double(utmConverter_fromutm.Lon);
  captain->send_package();
};
*/

void RosInterFace::ros_callback_speed(const std_msgs::Float32::ConstPtr &_msg) {
  float targetSpeed = _msg->data;
  captain->new_package(SC_SET_TARGET_SPEED);
  captain->add_float(targetSpeed);
  captain->send_package();
};

void RosInterFace::ros_callback_depth(const std_msgs::Float32::ConstPtr &_msg) {
  float targetDepth = _msg->data;
  captain->new_package(SC_SET_TARGET_DEPTH);
  captain->add_float(targetDepth);
  captain->send_package();
};

void RosInterFace::ros_callback_altitude(const std_msgs::Float32::ConstPtr &_msg) {
  float targetAltitude = _msg->data;
  captain->new_package(SC_SET_TARGET_ALTITUDE);
  captain->add_float(targetAltitude);
  captain->send_package();
};

void RosInterFace::ros_callback_yaw(const std_msgs::Float32::ConstPtr &_msg) {
  float targetYaw = _msg->data;
  captain->new_package(SC_SET_TARGET_YAW);
  captain->add_float(targetYaw);
  captain->send_package();
};

void RosInterFace::ros_callback_yawrate(const std_msgs::Float32::ConstPtr &_msg) {
  float targetYawRate = _msg->data;
  captain->new_package(SC_SET_TARGET_YAW_RATE);
  captain->add_float(targetYawRate);
  captain->send_package();
};

void RosInterFace::ros_callback_pitch(const std_msgs::Float32::ConstPtr &_msg) {
  float targetPitch = _msg->data;
  captain->new_package(SC_SET_TARGET_PITCH);
  captain->add_float(targetPitch);
  captain->send_package();
};

void RosInterFace::ros_callback_rpm(const std_msgs::Float32::ConstPtr &_msg) {
  float targetRPM = _msg->data;
  captain->new_package(SC_SET_TARGET_RPM);
  captain->add_float(targetRPM);
  captain->send_package();
};


void RosInterFace::ros_callback_rudder(const std_msgs::Float32::ConstPtr &_msg) {
  float angle = _msg->data;
  captain->new_package(SC_SET_RUDDER);
  captain->add_float(angle);
  captain->send_package();
};

void RosInterFace::ros_callback_elevator(const std_msgs::Float32::ConstPtr &_msg) {
  float angle = _msg->data;
  captain->new_package(SC_SET_ELEVATOR);
  captain->add_float(angle);
  captain->send_package();
};

void RosInterFace::ros_callback_thrusterPort(const std_msgs::Float32::ConstPtr &_msg) {
  captain->new_package(SC_SET_THRUSTER_PORT);
  captain->add_float(_msg->data);
  captain->send_package();
};

void RosInterFace::ros_callback_thrusterStrb(const std_msgs::Float32::ConstPtr &_msg) {
  captain->new_package(SC_SET_THRUSTER_STRB);
  captain->add_float(_msg->data);
  captain->send_package();
};

void RosInterFace::ros_callback_menu(const std_msgs::String::ConstPtr &_msg) {
  captain->new_package(SC_MENUSTREAM);
  std::string s = _msg->data;
  uint8_t bytes = std::min(200, (int) s.size());
  //printf("Tod send: %s  length: %d\n", s.c_str(), bytes);
  captain->add_byte(bytes);
  for(int i=0;i<bytes;i++) {
    captain->add_byte(s[i]);
  }
  captain->send_package();
};
