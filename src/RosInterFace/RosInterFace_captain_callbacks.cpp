#include "captain_interface/RosInterFace/RosInterFace.h"

void RosInterFace::captain_callback_LEAK() {
  smarc_msgs::Leak msg;
  leak_dome.publish(msg);
}

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

  //publish orientation
  geometry_msgs::QuaternionStamped orientation_msg;
  orientation_msg.header.stamp = ros::Time(sec,usec*1000);
  orientation_msg.header.seq = sequence;
  orientation_msg.header.frame_id = "dome";
  
  orientation_msg.quaternion.w = q1;
  orientation_msg.quaternion.x = q2;
  orientation_msg.quaternion.y = q3;
  orientation_msg.quaternion.z = q4;
  status_orientation_pub.publish(orientation_msg);

  /* moved to callback_position
  //publish position (lat lon)
  geographic_msgs::GeoPointStamped pos_msg;

  pos_msg.header.stamp = ros::Time(sec,usec*1000);
  pos_msg.header.seq = sequence;
  pos_msg.header.frame_id = "dome";
  
  pos_msg.position.latitude = lat*(180.0 / PI);
  pos_msg.position.longitude = lon * (180.0 / PI);
  status_position_pub.publish(pos_msg);
  */

  /*
  //publish depth
  smarc_msgs::FloatStamped depth_msg;
  depth_msg.data = depth;
  depth_msg.header.stamp = ros::Time(sec,usec*1000);
  depth_msg.header.seq = sequence;
  depth_msg.header.frame_id = "dome";
  status_depth_pub.publish(depth_msg);
  */

  //publish altitude
  smarc_msgs::FloatStamped alt_msg;
  alt_msg.data = altitude;
  alt_msg.header.stamp = ros::Time(sec,usec*1000);
  alt_msg.header.seq = sequence;
  alt_msg.header.frame_id = "dome";
  status_altitude_pub.publish(alt_msg);

  //Twist NED
  geometry_msgs::TwistWithCovarianceStamped twist_msg;
  twist_msg.header.stamp = ros::Time(sec,usec*1000);
  twist_msg.header.seq = sequence;
  twist_msg.header.frame_id = "dome";
  twist_msg.twist.twist.linear.x = sogX;
  twist_msg.twist.twist.linear.y = sogY;
  twist_msg.twist.twist.linear.z = sogZ;
  twist_msg.twist.twist.angular.x = rotX;
  twist_msg.twist.twist.angular.y = rotY;
  twist_msg.twist.twist.angular.z = rotZ;
  status_twist_pub.publish(twist_msg);
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

  lolo_msgs::CaptainStatus msg;
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

void RosInterFace::captain_callback_RUDDER() {
  uint64_t timestamp    = captain->parse_llong();
  uint32_t sequence     = captain->parse_long();
  uint64_t sec = timestamp / 1000000;
  uint64_t usec = timestamp % 1000000;

  float target_angle    = captain->parse_float();
  float current_angle   = captain->parse_float();

  smarc_msgs::FloatStamped angle_message;
  angle_message.data = current_angle;
  angle_message.header.stamp = ros::Time(sec,usec*1000);
  angle_message.header.seq = sequence;
  angle_message.header.frame_id = "rudder";
  rudder_angle_pub.publish(angle_message);
}

void RosInterFace::captain_callback_ELEVATOR() {
  uint64_t timestamp    = captain->parse_llong();
  uint32_t sequence     = captain->parse_long();
  uint64_t sec = timestamp / 1000000;
  uint64_t usec = timestamp % 1000000;

  float target_angle    = captain->parse_float();
  float current_angle   = captain->parse_float();

  smarc_msgs::FloatStamped angle_message;
  angle_message.data = current_angle;
  angle_message.header.stamp = ros::Time(sec,usec*1000);
  angle_message.header.seq = sequence;
  angle_message.header.frame_id = "elvator";
  elevator_angle_pub.publish(angle_message);
}

void RosInterFace::captain_callback_ELEVON_PORT() {
  uint64_t timestamp    = captain->parse_llong();
  uint32_t sequence     = captain->parse_long();
  uint64_t sec = timestamp / 1000000;
  uint64_t usec = timestamp % 1000000;

  float target_angle    = captain->parse_float();
  float current_angle   = captain->parse_float();

  smarc_msgs::FloatStamped angle_message;
  angle_message.data = current_angle;
  angle_message.header.stamp = ros::Time(sec,usec*1000);
  angle_message.header.seq = sequence;
  angle_message.header.frame_id = "elevon_port";
  elevon_port_angle_pub.publish(angle_message);
}

void RosInterFace::captain_callback_ELEVON_STRB() {
  uint64_t timestamp    = captain->parse_llong();
  uint32_t sequence     = captain->parse_long(); 
  uint64_t sec = timestamp / 1000000;
  uint64_t usec = timestamp % 1000000;

  float target_angle    = captain->parse_float();
  float current_angle   = captain->parse_float();

  smarc_msgs::FloatStamped angle_message;
  angle_message.data = current_angle;
  angle_message.header.stamp = ros::Time(sec,usec*1000);
  angle_message.header.seq = sequence;
  angle_message.header.frame_id = "elevon_strb";
  elevon_strb_angle_pub.publish(angle_message);
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

  smarc_msgs::ThrusterFeedback thruster_msg;
  thruster_msg.header.stamp = ros::Time(sec,usec*1000);
  thruster_msg.header.seq = sequence;
  thruster_msg.header.frame_id = "thruster_port";
  thruster_msg.rpm.rpm = rpm;
  thruster_msg.current = current;
  thruster_msg.torque = torque;
  thrusterPort_pub.publish(thruster_msg);
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
  
  smarc_msgs::ThrusterFeedback thruster_msg;
  thruster_msg.header.stamp = ros::Time(sec,usec*1000);
  thruster_msg.header.seq = sequence;
  thruster_msg.header.frame_id = "thruster_strb";
  thruster_msg.rpm.rpm = rpm;
  thruster_msg.current = current;
  thruster_msg.torque = torque;
  thrusterStrb_pub.publish(thruster_msg);
}

void RosInterFace::captain_callback_BATTERY() {
  uint64_t timestamp    = captain->parse_llong();
  uint32_t sequence     = captain->parse_long();
  uint64_t sec = timestamp / 1000000;
  uint64_t usec = timestamp % 1000000;

  uint64_t timestampHealth = captain->parse_llong();
  float voltage = captain->parse_float();
  float current = captain->parse_float();
  float energy = captain->parse_float();
  uint8_t Batterypacks = captain->parse_float();

  /* TODO do something with this data
  struct tempData {
    uint64_t ts = 0;
    float temp1 = 0.0;
    float temp2 = 0.0;
    float temp3 = 0.0;
    float temp4 = 0.0; 
  }

  tempData temp[Batterypacks]{
    temp[1],
    temp[2]
  };

  for (int i = 0; i<Batterypacks; i++){
    temp[i].ts = captain->parse_llong();
    temp[i].temp1 = captain->parse_float();
    temp[i].temp2 = captain->parse_float();
    temp[i].temp3 = captain->parse_float();
    temp[i].temp4 = captain->parse_float():
  }
  */
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

  cola2_msgs::DVL msg;
  msg.header.stamp = ros::Time(sec,usec*1000);
  msg.header.seq = sequence;
  msg.header.frame_id = "dvl";
  msg.velocity.x = sogX;
  msg.velocity.y = sogY;
  msg.velocity.z = sogZ;
  msg.velocity_covariance[0] = c00;
  msg.velocity_covariance[1] = c10;
  msg.velocity_covariance[2] = c20;
  msg.velocity_covariance[3] = c10;
  msg.velocity_covariance[4] = c11;
  msg.velocity_covariance[5] = c21;
  msg.velocity_covariance[6] = c20;
  msg.velocity_covariance[7] = c21;
  msg.velocity_covariance[8] = c22;
  msg.altitude = range;
  dvl_pub.publish(msg);
  
  //TODO add DVLbeam
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
  msg.header.frame_id = "gps";
  msg.latitude = 180.0*lat / PI;
  msg.longitude = 180.0*lon / PI;
  msg.status.status = 0; //TODO set status
  msg.status.service = 1; //TODO set service
  gps_pub.publish(msg);
}

void RosInterFace::captain_callback_IMU() {
  uint64_t timestamp    = captain->parse_llong();
  uint64_t sec = timestamp / 1000000;
  uint64_t usec = timestamp % 1000000;
  uint32_t sequence     = captain->parse_long();

  float Q1              = captain->parse_float();
  float Q2              = captain->parse_float();
  float Q3              = captain->parse_float();
  float Q4              = captain->parse_float();

  float accX            = captain->parse_float();
  float accY            = captain->parse_float();
  float accZ            = captain->parse_float();
  float LAccX           = captain->parse_float();
  float LAccY           = captain->parse_float();
  float LAccZ           = captain->parse_float();

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

  sensor_msgs::Imu msg;
  msg.header.stamp = ros::Time(sec,usec*1000);
  msg.header.seq = sequence;
  msg.header.frame_id = "dome";

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

  msg.linear_acceleration.x = LAccX;
  msg.linear_acceleration.y = LAccY;
  msg.linear_acceleration.z = LAccZ;

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
  msg.header.frame_id = "dome";
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
  msg.header.frame_id = "dome";
  msg.fluid_pressure = pressure;
  msg.variance = variance;
  pressure_pub.publish(msg);
}

void RosInterFace::captain_callback_VBS() {
  /*
  //Front tank
  int newData_front = captain->parse_byte();
  uint64_t timestamp_front_tank = captain->parse_llong();            //timestamp from ISB
  float vbs_front_tank_percent_current = captain->parse_float();     //Front tank volume [0-1]
  float vbs_front_tank_percent_target = captain->parse_float();      //Front tank target volume [0-1]
  float vbs_front_tank_pressure = captain->parse_float();            //Front tank pressure
  float vbs_front_tank_volume = captain->parse_float();              //Front tank volume
  uint64_t front_tank_sec = timestamp_front_tank / 1000000;
  uint64_t front_tank_usec = timestamp_front_tank % 1000000;

  if(newData_front) {
    lolo_msgs::VbsTank msg;
    msg.header.stamp = ros::Time(front_tank_sec,front_tank_usec*1000);
    msg.header.frame_id = "VBS";
    msg.precent_current = vbs_front_tank_percent_current;
    msg.precent_target = vbs_front_tank_percent_target;
    msg.pressure = vbs_front_tank_pressure;
    msg.volume = vbs_front_tank_volume;
    VBS_front_tank_pub.publish(msg);
  }

  //Aft tank
  int newData_aft = captain->parse_byte();
  uint64_t timestamp_aft_tank = captain->parse_llong();              //timestamp from ISB
  float vbs_aft_tank_percent_current = captain->parse_float();       //Aft tank volume [0-1]
  float vbs_aft_tank_percent_target = captain->parse_float();        //Aft tank target volume [0-1]
  float vbs_aft_tank_pressure = captain->parse_float();              //Aft tank pressure
  float vbs_aft_tank_volume = captain->parse_float();                //Aft tank volume
  uint64_t aft_tank_sec = timestamp_aft_tank / 1000000;
  uint64_t aft_tank_usec = timestamp_aft_tank % 1000000;

  if(newData_aft) {
    lolo_msgs::VbsTank msg;
    msg.header.stamp = ros::Time(aft_tank_sec,aft_tank_usec*1000);
    msg.header.frame_id = "VBS";
    msg.precent_current = vbs_aft_tank_percent_current;
    msg.precent_target = vbs_aft_tank_percent_target;
    msg.pressure = vbs_aft_tank_pressure;
    msg.volume = vbs_aft_tank_volume;
    VBS_aft_tank_pub.publish(msg);
    
  }
  
  //Valve stuff
  int valves = captain->parse_byte();                                //Valve information
  //TODO parse this.

  //Motor stuff
  int newData_motor = captain->parse_byte();
  uint64_t vbs_motor_ts = captain->parse_llong();                    //timestamp from ISB
  float vbs_motor_rpm_target = captain->parse_float();               //Target RPM
  float vbs_motor_rpm = captain->parse_float();                      //current RPM
  float vbs_motor_current = captain->parse_float();                  //motor current (A)
  float vbs_motor_voltage = captain->parse_float();                  //motor voltage (V) (not used)
  float vbs_motor_torque = captain->parse_float();                   //motor torque (Nm)
  uint64_t motor_sec = vbs_motor_ts / 1000000;
  uint64_t motor_usec = vbs_motor_ts % 1000000;

  if(newData_motor) {
    smarc_msgs::ThrusterFeedback msg;
    msg.header.stamp = ros::Time(motor_sec,motor_usec*1000);
    msg.header.frame_id = "VBS";
    msg.rpm.rpm = vbs_motor_rpm;
    msg.current = vbs_motor_current;
    msg.torque = vbs_motor_torque;

    VBS_motor_pub.publish(msg);
  }
  */
};

void RosInterFace::captain_callback_POSITION() {

  uint64_t timestamp    = captain->parse_llong();
  uint64_t sec = timestamp / 1000000;
  uint64_t usec = timestamp % 1000000;
  uint32_t sequence     = captain->parse_long();
  
  double lat        = captain->parse_double();
  double lon        = captain->parse_double();
  float depth       = captain->parse_float();

  float SOGX        = captain->parse_float();
  float SOGY        = captain->parse_float();
  float SOGZ        = captain->parse_float();

  //publish position (lat lon)
  geographic_msgs::GeoPointStamped pos_msg;

  pos_msg.header.stamp = ros::Time(sec,usec*1000);
  pos_msg.header.seq = sequence;
  pos_msg.header.frame_id = "dome";
  
  pos_msg.position.latitude = lat*(180.0 / PI);
  pos_msg.position.longitude = lon * (180.0 / PI);
  status_position_pub.publish(pos_msg);
 
  //publish depth
  smarc_msgs::FloatStamped depth_msg;
  depth_msg.data = depth;
  depth_msg.header.stamp = ros::Time(sec,usec*1000);
  depth_msg.header.seq = sequence;
  depth_msg.header.frame_id = "dome";
  status_depth_pub.publish(depth_msg);
};

void RosInterFace::captain_callback_FLS() {
  //FLS
  uint64_t timestamp    = captain->parse_llong();
  uint64_t sec = timestamp / 1000000;
  uint64_t usec = timestamp % 1000000;
  uint32_t sequence     = captain->parse_long();
  
  float range        = captain->parse_float();
  float confidence   = captain->parse_float();
  
  //publish depth
  smarc_msgs::FloatStamped fls_msg;
  fls_msg.data = range;
  fls_msg.header.stamp = ros::Time(sec,usec*1000);
  fls_msg.header.seq = sequence;
  fls_msg.header.frame_id = "front";
  fls_pub.publish(fls_msg);

};

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
