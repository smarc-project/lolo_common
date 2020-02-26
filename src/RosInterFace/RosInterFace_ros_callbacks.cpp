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


void RosInterFace::ros_callback_waypoint(const geometry_msgs::Point::ConstPtr &_msg) {
  float lat = _msg->x;
  float lon = _msg->y;
  captain->new_package(SC_SET_TARGET_WAYPOINT); // set target waypoint
  captain->add_double(lat);
  captain->add_double(lon);
  captain->send_package();
};

void RosInterFace::ros_callback_UTMwaypoint(const smarc_msgs::UTMpoint::ConstPtr &_msg) {
  
  //Convert
  utmConverter_fromutm.UTM2Geo(_msg->easting, _msg->northing, _msg->zone, _msg->band);
  captain->new_package(SC_SET_TARGET_WAYPOINT); // set target waypoint
  captain->add_double(utmConverter_fromutm.Lat);
  captain->add_double(utmConverter_fromutm.Lon);
  captain->send_package();
};

void RosInterFace::ros_callback_speed(const smarc_msgs::Float32Stamped::ConstPtr &_msg) {
  float targetSpeed = _msg->data;
  captain->new_package(SC_SET_TARGET_SPEED); // set target speed
  captain->add_float(targetSpeed);
  captain->send_package();
};

void RosInterFace::ros_callback_depth(const smarc_msgs::Float32Stamped::ConstPtr &_msg) {
  float targetDepth = _msg->data;

  //TODO check transform
  //Change sign if enu is used
  //std::string frame = _msg->header.frame_id;
  //if(frame == "world_utm") targetDepth = -targetDepth;

  captain->new_package(SC_SET_TARGET_DEPTH); // set target depth
  captain->add_float(targetDepth);
  captain->send_package();
};

void RosInterFace::ros_callback_altitude(const smarc_msgs::Float32Stamped::ConstPtr &_msg) {
  float targetAltitude = _msg->data;

  //No need to look at frame

  captain->new_package(SC_SET_TARGET_ALTITUDE); // set target altitude
  captain->add_float(targetAltitude);
  captain->send_package();
};

void RosInterFace::ros_callback_yaw(const smarc_msgs::Float32Stamped::ConstPtr &_msg) {
  float targetYaw = _msg->data;

  //TODO check transform
  //Change sign if enu is used
  std::string frame = _msg->header.frame_id;
  //if(frame == "world_utm") targetYaw = -targetYaw + pi/4;

  captain->new_package(SC_SET_TARGET_YAW); // set target yaw
  captain->add_float(targetYaw);
  captain->send_package();
};

void RosInterFace::ros_callback_yawrate(const smarc_msgs::Float32Stamped::ConstPtr &_msg) {
  float targetYawRate = _msg->data;

  //TODO check transform
  //Change sign if enu is used
  std::string frame = _msg->header.frame_id;
  //if(frame == "world_utm") targetYawRate = -targetYawRate;

  captain->new_package(SC_SET_TARGET_YAW_RATE); // set target yaw rate
  captain->add_float(targetYawRate);
  captain->send_package();
};

void RosInterFace::ros_callback_pitch(const smarc_msgs::Float32Stamped::ConstPtr &_msg) {
  float targetPitch = _msg->data;

  //TODO check transform
  //Change sign if enu is used
  std::string frame = _msg->header.frame_id;
  //if(frame == "world_utm") targetPitch = -targetPitch;

  captain->new_package(SC_SET_TARGET_PITCH); // set target pitch
  captain->add_float(targetPitch);
  captain->send_package();
};

void RosInterFace::ros_callback_rpm(const smarc_msgs::Float32Stamped::ConstPtr &_msg) {
  float targetRPM = _msg->data;

  //No need to look at frame

  captain->new_package(SC_SET_TARGET_RPM); // targetRPM
  captain->add_float(targetRPM);
  captain->send_package();
};


void RosInterFace::ros_callback_rudderPort(const smarc_msgs::Float32Stamped::ConstPtr &_msg) {
  float angle = _msg->data;

  //TODO check transform
  //Change sign if enu is used
  std::string frame = _msg->header.frame_id;
  //if(frame == "world_utm") angle = -angle;

  captain->new_package(SC_SET_RUDDER_PORT); // Control message
  captain->add_float(angle);
  captain->send_package();
};

void RosInterFace::ros_callback_rudderStrb(const smarc_msgs::Float32Stamped::ConstPtr &_msg) {
  float angle = _msg->data;

  //TODO check transform
  //Change sign if enu is used
  std::string frame = _msg->header.frame_id;
  //if(frame == "world_utm") angle = -angle;

  captain->new_package(SC_SET_RUDDER_STRB);
  captain->add_float(angle);
  captain->send_package();
};

void RosInterFace::ros_callback_elevator(const smarc_msgs::Float32Stamped::ConstPtr &_msg) {
  float angle = _msg->data;

  //TODO check transform
  //Change sign if enu is used
  std::string frame = _msg->header.frame_id;
  //if(frame == "world_utm") angle = -angle;

  captain->new_package(SC_SET_ELEVATOR);
  captain->add_float(angle);
  captain->send_package();
};

void RosInterFace::ros_callback_thrusterPort(const smarc_msgs::Float32Stamped::ConstPtr &_msg) {
  //printf("Port Thruster angle received %f\n", _msg->angle);
  captain->new_package(SC_SET_THRUSTER_PORT);
  captain->add_float(_msg->data);
  captain->send_package();
};

void RosInterFace::ros_callback_thrusterStrb(const smarc_msgs::Float32Stamped::ConstPtr &_msg) {
  //printf("Strb Thruster: %f\n", _msg->angle);
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
