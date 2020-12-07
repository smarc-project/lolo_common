#include "captain_interface/RosInterFace/RosInterFace.h"

void RosInterFace::ros_callback_heartbeat(const std_msgs::Empty::ConstPtr &_msg) {
  captain->new_package(SC_HEARTBEAT); // Heartbeat message
  captain->send_package();
};

void RosInterFace::ros_callback_abort(const std_msgs::Empty::ConstPtr &_msg) {
  captain->new_package(SC_ABORT); // Tell captain to go into emergency mode
  captain->send_package();
};


void RosInterFace::ros_callback_done(const std_msgs::Empty::ConstPtr &_msg) {
  captain->new_package(SC_DONE); // Tell captain that scientist is done
  captain->send_package();
};

void RosInterFace::ros_callback_waypoint_utm(const geometry_msgs::Point::ConstPtr &_msg) {
  //TODO use service to get lat lon from the utm point
  smarc_msgs::UTMToLatLon srv;
  srv.request.utm_point = *_msg;
  if (utm_to_latlon_client.call(srv))
  {
    ROS_INFO("Call to action server sucsessfull");
    boost::shared_ptr<geographic_msgs::GeoPoint> latlonmsg(new geographic_msgs::GeoPoint());
    latlonmsg->latitude = srv.response.lat_lon_point.latitude;
    latlonmsg->longitude = srv.response.lat_lon_point.longitude;
    latlonmsg->altitude = srv.response.lat_lon_point.altitude;
    ros_callback_waypoint(latlonmsg);
  }
  else
  {
    ROS_ERROR("Failed to call service to convert utm to latlon. waypoint not sent to the captain");
  }
}

void RosInterFace::ros_callback_waypoint(const geographic_msgs::GeoPoint::ConstPtr &_msg) {
  double lat = _msg->latitude;
  double lon = _msg->longitude;
  captain->new_package(SC_SET_TARGET_WAYPOINT); // set target waypoint
  captain->add_double((PI / 180) * lat);
  captain->add_double((PI / 180) * lon);
  captain->send_package();
};

void RosInterFace::ros_callback_speed(const std_msgs::Float64::ConstPtr &_msg) {
  float targetSpeed = _msg->data;
  captain->new_package(SC_SET_TARGET_SPEED);
  captain->add_float(targetSpeed);
  captain->send_package();
};

void RosInterFace::ros_callback_depth(const std_msgs::Float64::ConstPtr &_msg) {
  float targetDepth = _msg->data;
  captain->new_package(SC_SET_TARGET_DEPTH);
  captain->add_float(targetDepth);
  captain->send_package();
};

void RosInterFace::ros_callback_altitude(const std_msgs::Float64::ConstPtr &_msg) {
  float targetAltitude = _msg->data;
  captain->new_package(SC_SET_TARGET_ALTITUDE);
  captain->add_float(targetAltitude);
  captain->send_package();
};

void RosInterFace::ros_callback_yaw(const std_msgs::Float64::ConstPtr &_msg) {
  float targetYaw = _msg->data;
  captain->new_package(SC_SET_TARGET_YAW);
  captain->add_float(targetYaw);
  captain->send_package();
};

void RosInterFace::ros_callback_yawrate(const std_msgs::Float64::ConstPtr &_msg) {
  float targetYawRate = _msg->data;
  captain->new_package(SC_SET_TARGET_YAW_RATE);
  captain->add_float(targetYawRate);
  captain->send_package();
};

void RosInterFace::ros_callback_pitch(const std_msgs::Float64::ConstPtr &_msg) {
  float targetPitch = _msg->data;
  captain->new_package(SC_SET_TARGET_PITCH);
  captain->add_float(targetPitch);
  captain->send_package();
};

void RosInterFace::ros_callback_rpm(const std_msgs::Float64::ConstPtr &_msg) {
  float targetRPM = _msg->data;
  captain->new_package(SC_SET_TARGET_RPM);
  captain->add_float(targetRPM);
  captain->send_package();
};


void RosInterFace::ros_callback_rudder(const std_msgs::Float64::ConstPtr &_msg) {
  float angle = _msg->data;
  captain->new_package(SC_SET_RUDDER);
  captain->add_float(angle);
  captain->send_package();
};

void RosInterFace::ros_callback_elevator(const std_msgs::Float64::ConstPtr &_msg) {
  float angle = _msg->data;
  captain->new_package(SC_SET_ELEVATOR);
  captain->add_float(angle);
  captain->send_package();
};

void RosInterFace::ros_callback_thrusterPort(const std_msgs::Float64::ConstPtr &_msg) {
  captain->new_package(SC_SET_THRUSTER_PORT);
  captain->add_float(_msg->data);
  captain->send_package();
};

void RosInterFace::ros_callback_thrusterStrb(const std_msgs::Float64::ConstPtr &_msg) {
  captain->new_package(SC_SET_THRUSTER_STRB);
  captain->add_float(_msg->data);
  captain->send_package();
};

void RosInterFace::ros_callback_menu(const std_msgs::String::ConstPtr &_msg) {
  captain->new_package(SC_MENUSTREAM);
  std::string s = _msg->data;
  uint8_t bytes = std::min(200, (int) s.size());
  captain->add_byte(bytes);
  for(int i=0;i<bytes;i++) {
    captain->add_byte(s[i]);
  }
  captain->send_package();
};
