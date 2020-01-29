#include "captain_interface/RosInterFace/RosInterFace.h"

void RosInterFace::captain_callback_STATUS() {
uint64_t timestamp    = captain->parse_llong();
  uint64_t sec = timestamp / 1000000;
  uint64_t usec = timestamp % 1000000;
  uint32_t sequence     = captain->parse_long();

  float lat         = captain->parse_double();
  float lon         = captain->parse_double();
  float depth       = captain->parse_float();
  float altitude    = captain->parse_float();
  float sogX        = captain->parse_float();
  float sogY        = captain->parse_float();
  float sogZ        = captain->parse_float();
  float rotX        = captain->parse_float();
  float rotY        = captain->parse_float();
  float rotZ        = captain->parse_float();
  float pitch       = captain->parse_float();
  float roll        = captain->parse_float();
  float yaw         = captain->parse_float();
  float q1          = captain->parse_float();
  float q2          = captain->parse_float();
  float q3          = captain->parse_float();
  float q4          = captain->parse_float();

  //publish position (lon,lat,depth)
  geometry_msgs::PoseWithCovarianceStamped pos_msg;
  pos_msg.header.stamp = ros::Time(sec,usec*1000);
  pos_msg.header.seq = sequence;
  pos_msg.header.frame_id = "world_ned";
  pos_msg.pose.pose.position.x = lat;
  pos_msg.pose.pose.position.y = lon;
  pos_msg.pose.pose.position.z = depth;
  pos_msg.pose.pose.orientation.w = q1;
  pos_msg.pose.pose.orientation.x = q2;
  pos_msg.pose.pose.orientation.y = q3;
  pos_msg.pose.pose.orientation.z = q4;
  status_position_pub.publish(pos_msg);

  //Twist NED
  geometry_msgs::TwistWithCovarianceStamped twist_msg;
  twist_msg.header.stamp = ros::Time(sec,usec*1000);
  twist_msg.header.seq = sequence;
  twist_msg.header.frame_id = "local_imu";
  twist_msg.twist.twist.linear.x = sogX;
  twist_msg.twist.twist.linear.y = sogY;
  twist_msg.twist.twist.linear.z = sogZ;
  twist_msg.twist.twist.angular.x = rotX;
  twist_msg.twist.twist.angular.y = rotY;
  twist_msg.twist.twist.angular.z = rotZ;
  status_twist_pub.publish(twist_msg);

  //publish altitude
  smarc_msgs::Float32Stamped alt_msg;
  alt_msg.header.stamp = ros::Time(sec,usec*1000);
  alt_msg.header.seq = sequence;
  alt_msg.header.frame_id = "local_dvl";
  alt_msg.data = altitude;
  status_altitude_pub.publish(alt_msg);
}

void RosInterFace::captain_callback_CONTROL() {
  uint64_t timestamp    = captain->parse_llong();
  uint64_t sec = timestamp / 1000000;
  uint64_t usec = timestamp % 1000000;
  uint32_t sequence     = captain->parse_long();

  float source  = captain->parse_byte();
  float targetWaypoint_lat = captain->parse_double();
  float targetWaypoint_lon = captain->parse_double();
  float targetYaw      = captain->parse_float();
  float targetPitch    = captain->parse_float();
  float targetSpeed    = captain->parse_float();
  float targetRPM      = captain->parse_float();
  float targetDepth    = captain->parse_float();
  float targetAltitude = captain->parse_float();

  smarc_msgs::CaptainStatus msg;
  msg.header.stamp = ros::Time(sec,usec*1000);
  msg.header.seq = sequence;
  msg.header.frame_id = "world_ned";
  msg.active_control_input  = source;
  msg.targetWaypoint_lat    = targetWaypoint_lat;
  msg.targetWaypoint_lon    = targetWaypoint_lon;
  msg.targetYaw             = targetYaw;
  msg.targetPitch           = targetPitch;
  msg.targetSpeed           = targetSpeed;
  msg.targetRPM             = targetRPM;
  msg.targetDepth           = targetDepth;
  msg.targetAltitude        = targetAltitude;
  control_status_pub.publish(msg);
}

void RosInterFace::captain_callback_RUDDER_PORT() {
  uint64_t timestamp    = captain->parse_llong();
  uint32_t sequence     = captain->parse_long();
  float target_angle    = captain->parse_float();
  float current_angle   = captain->parse_float();

  std_msgs::Header header;
  uint64_t sec = timestamp / 1000000;
  uint64_t usec = timestamp % 1000000;
  header.stamp = ros::Time(sec,usec*1000);
  header.seq = sequence;

  smarc_msgs::Float32Stamped angle_message_ned;
  angle_message_ned.header = header;
  angle_message_ned.header.frame_id = "local_rudder_port";
  angle_message_ned.data = current_angle;
  rudderPort_angle_pub.publish(angle_message_ned);

  /*
  smarc_msgs::Float32Stamped current_message;
  current_message.header = header;
  current_message.header.frame_id = "local_rudder_port";
  current_message.data = current;
  rudderPort_current_pub.publish(current_message);
  */
}

void RosInterFace::captain_callback_RUDDER_STRB() {
  uint64_t timestamp    = captain->parse_llong();
  uint32_t sequence     = captain->parse_long();
  float target_angle    = captain->parse_float();
  float current_angle   = captain->parse_float();

  std_msgs::Header header;
  uint64_t sec = timestamp / 1000000;
  uint64_t usec = timestamp % 1000000;
  header.stamp = ros::Time(sec,usec*1000);
  header.seq = sequence;

  smarc_msgs::Float32Stamped angle_message_ned;
  angle_message_ned.header = header;
  angle_message_ned.header.frame_id = "local_rudder_strb";
  angle_message_ned.data = current_angle;
  rudderStrb_angle_pub.publish(angle_message_ned);

  /*
  smarc_msgs::Float32Stamped current_message;
  current_message.header = header;
  current_message.header.frame_id = "world";
  current_message.data = current;
  rudderPort_current_pub.publish(current_message);
  */
}

void RosInterFace::captain_callback_ELEVATOR() {
  uint64_t timestamp    = captain->parse_llong();
  uint32_t sequence     = captain->parse_long();
  float target_angle    = captain->parse_float();
  float current_angle   = captain->parse_float();

  std_msgs::Header header;
  uint64_t sec = timestamp / 1000000;
  uint64_t usec = timestamp % 1000000;
  header.stamp = ros::Time(sec,usec*1000);
  header.seq = sequence;

  smarc_msgs::Float32Stamped angle_message_ned;
  angle_message_ned.header = header;
  angle_message_ned.header.frame_id = "local_elevator";
  angle_message_ned.data = current_angle;
  elevator_angle_pub.publish(angle_message_ned);

  /*
  smarc_msgs::Float32Stamped current_message;
  current_message.header = header;
  current_message.header.frame_id = "local_elevator";
  current_message.data = current;
  rudderPort_current_pub.publish(current_message);
  */
}

void RosInterFace::captain_callback_ELEVON_PORT() {
  uint64_t timestamp    = captain->parse_llong();
  uint32_t sequence     = captain->parse_long();
  float target_angle    = captain->parse_float();
  float current_angle   = captain->parse_float();

  std_msgs::Header header;
  uint64_t sec = timestamp / 1000000;
  uint64_t usec = timestamp % 1000000;
  header.stamp = ros::Time(sec,usec*1000);
  header.seq = sequence;

  smarc_msgs::Float32Stamped angle_message_ned;
  angle_message_ned.header = header;
  angle_message_ned.header.frame_id = "local_elevon_port";
  angle_message_ned.data = current_angle;
  elevon_port_angle_pub.publish(angle_message_ned);

  /*
  smarc_msgs::Float32Stamped current_message;
  current_message.header = header;
  current_message.header.frame_id = "local_elevon_port";
  current_message.data = current;
  elevon_port_current_pub.publish(current_message);
  */
}

void RosInterFace::captain_callback_ELEVON_STRB() {
  uint64_t timestamp    = captain->parse_llong();
  uint32_t sequence     = captain->parse_long();
  float target_angle    = captain->parse_float();
  float current_angle   = captain->parse_float();

  std_msgs::Header header;
  uint64_t sec = timestamp / 1000000;
  uint64_t usec = timestamp % 1000000;
  header.stamp = ros::Time(sec,usec*1000);
  header.seq = sequence;

  smarc_msgs::Float32Stamped angle_message_ned;
  angle_message_ned.header = header;
  angle_message_ned.header.frame_id = "local_elevon_strb";
  angle_message_ned.data = current_angle;
  elevon_strb_angle_pub.publish(angle_message_ned);

  /*
  smarc_msgs::Float32Stamped current_message;
  current_message.header = header;
  current_message.header.frame_id = "local_elevon_strb";
  current_message.data = current;
  elevon_strb_angle_pub.publish(current_message);
  */
}

void RosInterFace::captain_callback_THRUSTER_PORT() {
  uint64_t timestamp    = captain->parse_llong();
  uint32_t sequence     = captain->parse_long();
  uint64_t sec = timestamp / 1000000;
  uint64_t usec = timestamp % 1000000;

  float rpm_setpoint    = captain->parse_float();
  float rpm             = captain->parse_float();
  float current         = captain->parse_float();
  float torque          = captain->parse_float();
  float energy          = captain->parse_float();
  float voltage         = captain->parse_float();

  std_msgs::Header header;
  header.stamp = ros::Time(sec,usec*1000);
  header.seq = sequence;
  header.frame_id = "local_thruster_port";

  smarc_msgs::Float32Stamped rpm_message; rpm_message.data = rpm; rpm_message.header = header;
  smarc_msgs::Float32Stamped current_message; current_message.data = current; current_message.header = header;
  smarc_msgs::Float32Stamped torque_message; torque_message.data = torque; torque_message.header = header;
  thrusterPort_rpm_pub.publish(rpm_message);
  thrusterPort_current_pub.publish(current_message);
  thrusterPort_torque_pub.publish(torque_message);
}

void RosInterFace::captain_callback_THRUSTER_STRB() {
  uint64_t timestamp    = captain->parse_llong();
  uint32_t sequence     = captain->parse_long();
  uint64_t sec = timestamp / 1000000;
  uint64_t usec = timestamp % 1000000;

  float rpm_setpoint    = captain->parse_float();
  float rpm             = captain->parse_float();
  float current         = captain->parse_float();
  float torque          = captain->parse_float();
  float energy          = captain->parse_float();
  float voltage         = captain->parse_float();

  std_msgs::Header header;
  header.stamp = ros::Time(sec,usec*1000);
  header.seq = sequence;
  header.frame_id = "local_thruster_strb";

  smarc_msgs::Float32Stamped rpm_message; rpm_message.data = rpm; rpm_message.header = header;
  smarc_msgs::Float32Stamped current_message; current_message.data = current; current_message.header = header;
  smarc_msgs::Float32Stamped torque_message; torque_message.data = torque; torque_message.header = header;
  thrusterStrb_rpm_pub.publish(rpm_message);
  thrusterStrb_current_pub.publish(current_message);
  thrusterStrb_torque_pub.publish(torque_message);
}

void RosInterFace::captain_callback_BATTERY() {
  //TODO something
}

void RosInterFace::captain_callback_DVL() {
  uint64_t timestamp    = captain->parse_llong();
  uint64_t sec = timestamp / 1000000;
  uint64_t usec = timestamp % 1000000;
  uint32_t sequence     = captain->parse_long();

  float sogX           = captain->parse_float();
  float sogY           = captain->parse_float();
  float sogZ           = captain->parse_float();
  float stwX           = captain->parse_float();
  float stwY           = captain->parse_float();
  float stwZ           = captain->parse_float();
  float range1         = captain->parse_float();
  float range2         = captain->parse_float();
  float range3         = captain->parse_float();
  float range4         = captain->parse_float();
  float range = 0.25*(range1+range2+range3+range4);

  //Covariance
  float c00            = captain->parse_float();
  float c10            = captain->parse_float();
  float c11            = captain->parse_float();
  float c20            = captain->parse_float();
  float c21            = captain->parse_float();
  float c22            = captain->parse_float();

  smarc_msgs::DVL msg;
  msg.header.stamp = ros::Time(sec,usec*1000);
  msg.header.seq = sequence;
  msg.header.frame_id = "local_dvl";
  msg.velocity.x = sogX;
  msg.velocity.y = sogY;
  msg.velocity.z = sogZ;
  msg.velocity_reference = smarc_msgs::DVL::VELOCITY_REFERENCE_BOTTOM;
  msg.velocity_covariance[0] = c00;
  msg.velocity_covariance[1] = c10;
  msg.velocity_covariance[2] = c20;
  msg.velocity_covariance[3] = c10;
  msg.velocity_covariance[4] = c11;
  msg.velocity_covariance[5] = c21;
  msg.velocity_covariance[6] = c20;
  msg.velocity_covariance[7] = c21;
  msg.velocity_covariance[8] = c22;
  msg.range = range;
  dvl_pub.publish(msg);
}

void RosInterFace::captain_callback_GPS() {
  uint64_t timestamp    = captain->parse_llong();
  uint64_t sec = timestamp / 1000000;
  uint64_t usec = timestamp % 1000000;
  uint32_t sequence     = captain->parse_long();

  float lat           = captain->parse_double();
  float lon           = captain->parse_double();
  float cog           = captain->parse_float();
  float sog           = captain->parse_float();

  sensor_msgs::NavSatFix msg;
  msg.header.stamp = ros::Time(sec,usec*1000);
  msg.header.seq = sequence;
  msg.header.frame_id = "local_gps";
  msg.latitude = lat;
  msg.longitude = lon;
  msg.status.status = 0;
  msg.status.service = 1;
      gps_pub.publish(msg);
}

void RosInterFace::captain_callback_IMU() {
  uint64_t timestamp    = captain->parse_llong();
  uint64_t sec = timestamp / 1000000;
  uint64_t usec = timestamp % 1000000;
  uint32_t sequence     = captain->parse_long();

  //float pitch           = captain->parse_float();
  //float roll            = captain->parse_float();
  //float yaw             = captain->parse_float();
  float Q1              = captain->parse_float();
  float Q2              = captain->parse_float();
  float Q3              = captain->parse_float();
  float Q4              = captain->parse_float();

  float accX            = captain->parse_float();
  float accY            = captain->parse_float();
  float accZ            = captain->parse_float();

  float rotX            = captain->parse_float();
  float rotY            = captain->parse_float();
  float rotZ            = captain->parse_float();

  //Accelerometer Covariance
  float a_c00            = captain->parse_float();
  float a_c10            = captain->parse_float();
  float a_c11            = captain->parse_float();
  float a_c20            = captain->parse_float();
  float a_c21            = captain->parse_float();
  float a_c22            = captain->parse_float();

  //Gyro Covariance
  float g_c00            = captain->parse_float();
  float g_c10            = captain->parse_float();
  float g_c11            = captain->parse_float();
  float g_c20            = captain->parse_float();
  float g_c21            = captain->parse_float();
  float g_c22            = captain->parse_float();

  //// NED ////

  sensor_msgs::Imu msg;
  msg.header.stamp = ros::Time(sec,usec*1000);
  msg.header.seq = sequence;
  msg.header.frame_id = "local_imu";

  msg.orientation.w = Q1;
  msg.orientation.x = Q2;
  msg.orientation.y = Q3;
  msg.orientation.z = Q4;

  msg.angular_velocity.x = rotX;
  msg.angular_velocity.y = rotY;
  msg.angular_velocity.z = rotZ;

  msg.angular_velocity_covariance[0] = g_c00;
  msg.angular_velocity_covariance[1] = g_c10;
  msg.angular_velocity_covariance[2] = g_c20;
  msg.angular_velocity_covariance[3] = g_c10;
  msg.angular_velocity_covariance[4] = g_c11;
  msg.angular_velocity_covariance[5] = g_c21;
  msg.angular_velocity_covariance[6] = g_c20;
  msg.angular_velocity_covariance[7] = g_c21;
  msg.angular_velocity_covariance[8] = g_c22;

  msg.linear_acceleration.x = accX;
  msg.linear_acceleration.y = accY;
  msg.linear_acceleration.z = accZ;

  msg.linear_acceleration_covariance[0] = a_c00;
  msg.linear_acceleration_covariance[1] = a_c10;
  msg.linear_acceleration_covariance[2] = a_c20;
  msg.linear_acceleration_covariance[3] = a_c10;
  msg.linear_acceleration_covariance[4] = a_c11;
  msg.linear_acceleration_covariance[5] = a_c21;
  msg.linear_acceleration_covariance[6] = a_c20;
  msg.linear_acceleration_covariance[7] = a_c21;
  msg.linear_acceleration_covariance[8] = a_c22;
  imu_pub.publish(msg);
}

void RosInterFace::captain_callback_MAG() {
  uint64_t timestamp    = captain->parse_llong();
  uint64_t sec = timestamp / 1000000;
  uint64_t usec = timestamp % 1000000;
  uint32_t sequence     = captain->parse_long();
  float magX            = captain->parse_float();
  float magY            = captain->parse_float();
  float magZ            = captain->parse_float();

  //Magnetometer Covariance
  float m_c00            = captain->parse_float();
  float m_c10            = captain->parse_float();
  float m_c11            = captain->parse_float();
  float m_c20            = captain->parse_float();
  float m_c21            = captain->parse_float();
  float m_c22            = captain->parse_float();

  sensor_msgs::MagneticField msg;
  msg.header.stamp = ros::Time(sec,usec*1000);
  msg.header.seq = sequence;
  msg.header.frame_id = "local_imu";
  msg.magnetic_field.x = magX;
  msg.magnetic_field.y = magY;
  msg.magnetic_field.z = magZ;
  msg.magnetic_field_covariance[0] = m_c00;
  msg.magnetic_field_covariance[1] = m_c10;
  msg.magnetic_field_covariance[2] = m_c20;
  msg.magnetic_field_covariance[3] = m_c10;
  msg.magnetic_field_covariance[4] = m_c11;
  msg.magnetic_field_covariance[5] = m_c21;
  msg.magnetic_field_covariance[6] = m_c20;
  msg.magnetic_field_covariance[7] = m_c21;
  msg.magnetic_field_covariance[8] = m_c22;
  magnetometer_pub.publish(msg);
}

void RosInterFace::captain_callback_PRESSURE() {
  uint64_t timestamp    = captain->parse_llong();
  uint64_t sec = timestamp / 1000000;
  uint64_t usec = timestamp % 1000000;
  uint32_t sequence     = captain->parse_long();
  float pressure        = captain->parse_float();
  float variance        = captain->parse_float();

  sensor_msgs::FluidPressure msg;
  msg.header.stamp = ros::Time(sec,usec*1000);
  msg.header.seq = sequence;
  msg.header.frame_id = "local_pressure";
  msg.fluid_pressure = pressure;
  msg.variance = variance;
  pressure_pub.publish(msg);
}

void RosInterFace::captain_callback_TEXT() {
  int length = captain->parse_byte();
  std::string text = captain->parse_string(length);
  std_msgs::String msg;
  msg.data = text.c_str();
  text_pub.publish(msg);
}

void RosInterFace::captain_callback_MENUSTREAM() {
  int length = captain->parse_byte();
  std::string text = captain->parse_string(length);
  printf("%s\n",text.c_str());
  std_msgs::String msg;
  msg.data = text.c_str();
  menu_pub.publish(msg);
}
