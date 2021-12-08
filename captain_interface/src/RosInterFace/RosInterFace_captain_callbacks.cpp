#include "captain_interface/RosInterFace/RosInterFace.h"
#include <limits.h>

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
  alt_msg.header.frame_id = "lolo/dvl_link";
  status_altitude_pub.publish(alt_msg);

  /*
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
  */
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
  angle_message.header.frame_id = "lolo/rudder_port";
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
  angle_message.header.frame_id = "lolo/elvator";
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
  angle_message.header.frame_id = "lolo/elevon_port";
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
  angle_message.header.frame_id = "lolo/elevon_stbd";
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
  thruster_msg.header.frame_id = "lolo/thruster_port";
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
  thruster_msg.header.frame_id = "lolo/thruster_stbd";
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
  uint8_t batterypacks = captain->parse_byte();

  //TODO parse data from battery packs

  sensor_msgs::BatteryState msg;
  msg.header.stamp = ros::Time(sec,usec*1000);
  msg.header.seq = sequence;
  msg.voltage = voltage;                                  // Voltage in Volts (Mandatory)
  //msg.temperature = NaN;                                // Temperature in Degrees Celsius (If unmeasured NaN)
  msg.current = -current;                                 // Negative when discharging (A)  (If unmeasured NaN)
  msg.charge = std::numeric_limits<float>::quiet_NaN();  // Current charge in Ah  (If unmeasured NaN)
  msg.capacity = 2.0 * 5.0*12.0;                          // Capacity in Ah (last full capacity)  (If unmeasured NaN)
  msg.design_capacity = 2.0 * 5.0*12.0;                   // Capacity in Ah (design capacity)  (If unmeasured NaN)
  msg.percentage = (42.0-36.0) / (voltage - 36.0);        // Charge percentage on 0 to 1 range  (If unmeasured NaN)
  msg.power_supply_status = msg.POWER_SUPPLY_STATUS_DISCHARGING;
  msg.power_supply_health = msg.POWER_SUPPLY_HEALTH_UNKNOWN;
  msg.power_supply_technology = msg.POWER_SUPPLY_TECHNOLOGY_LIPO;
  msg.present = true;

  //msg.cell_voltage   # An array of individual cell voltages for each cell in the pack
  //                        # If individual voltages unknown but number of cells known set each to NaN
  //msg.cell_temperature  # An array of individual cell temperatures for each cell in the pack
  //                            # If individual temperatures unknown but number of cells known set each to NaN
  //msg.location          # The location into which the battery is inserted. (slot number or plug)
  //msg.serial_number     # The best approximation of the battery serial number
  battery_pub.publish(msg);
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

  int beams=0;
  float beams_sum = 0;
  if(range1 != 0) {beams++; beams_sum+=range1;};
  if(range2 != 0) {beams++; beams_sum+=range2;};
  if(range3 != 0) {beams++; beams_sum+=range3;};
  if(range4 != 0) {beams++; beams_sum+=range4;};

  float range = beams_sum / (beams*10);

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
  msg.header.frame_id = "lolo/dvl_link";
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


void RosInterFace::captain_callback_DVL_FIXED() {
  lolo_msgs::PD0_Fixedleader msg;
  msg.timeStamp  = captain->parse_llong();
  msg.CPU_VER  = captain->parse_byte();
  msg.CPU_REV  = captain->parse_byte();
  msg.SYSTEM_CONFIG  = captain->parse_int();
  msg.REAL_FLAG  = captain->parse_byte();
  msg.LAG_LENGTH = captain->parse_byte();
  msg.NR_BEAMS = captain->parse_byte();
  msg.NR_CELLS = captain->parse_byte();
  msg.PINGS_PER_ENSEMBLE = captain->parse_int();
  msg.DEPTH_CELL_LENGTH  = captain->parse_int();
  msg.BLANK_AFTER_TRANSMIT = captain->parse_int();
  msg.PROFILING_MODE = captain->parse_byte();
  msg.LOW_CORR_THRESHOLD = captain->parse_byte();
  msg.NO_CODE_REPS = captain->parse_byte();
  msg.ERROR_VEL_MAXIMUM  = captain->parse_int();
  msg.TPP_MINUTES  = captain->parse_byte();
  msg.TPP_SECONDS  = captain->parse_byte();
  msg.TPP_HUDREDTHS  = captain->parse_byte();
  msg.COORDINATE_TRANSFORM = captain->parse_byte();
  msg.HEADLING_ALIGNMENT = captain->parse_int();
  msg.HEADLING_BIAS  = captain->parse_int();
  msg.SENSOR_SOURCE  = captain->parse_byte();
  msg.SENSOR_AVAILABLE = captain->parse_byte();
  msg.BIN1_DISTANCE  = captain->parse_int();
  msg.XMIT_PULSE_LENGTH  = captain->parse_int();
  msg.FALSE_TARGET_THRESHOLD = captain->parse_byte();
  msg.TRANSMIT_LAG_DISTANCE  = captain->parse_int();
  msg.SYSTEM_BANDWIDTH = captain->parse_int();
  msg.SYSTEM_SERAL_NUMBER  = captain->parse_long();
  dvl_PD0_Fixedleader_pub.publish(msg);
}

void RosInterFace::captain_callback_DVL_VARIABLE() {
  lolo_msgs::PD0_Variableleader msg;
  msg.timeStamp                 = captain->parse_llong();
  msg.ENSEMBLE_NUMBER           = captain->parse_int();
  msg.BIT_RESULT                = captain->parse_int();
  msg.SPEED_OF_SOUND            = captain->parse_int();
  msg.DEPTH_OF_TRANSDUCER       = captain->parse_int();
  msg.HEADING                   = captain->parse_int();
  msg.PITCH                     = captain->parse_int();
  msg.ROLL                      = captain->parse_int();
  msg.SAILINITY                 = captain->parse_int();
  msg.TEMPERATURE               = captain->parse_int();
  msg.MPT_MINUTES               = captain->parse_byte();
  msg.MPT_SECONDS               = captain->parse_byte();
  msg.MPT_HUNDREDTHS            = captain->parse_byte();
  msg.HD_STD_DEV                = captain->parse_byte();
  msg.PITCH_STD_DEV             = captain->parse_byte();
  msg.ROLL_STD_DEV              = captain->parse_byte();
  msg.ADC_CHANNEL_0             = captain->parse_byte();
  msg.ADC_CHANNEL_1             = captain->parse_byte();
  msg.ADC_CHANNEL_2             = captain->parse_byte();
  msg.ADC_CHANNEL_3             = captain->parse_byte();
  msg.ADC_CHANNEL_4             = captain->parse_byte();
  msg.ADC_CHANNEL_5             = captain->parse_byte();
  msg.ADC_CHANNEL_6             = captain->parse_byte();
  msg.ADC_CHANNEL_7             = captain->parse_byte();
  msg.ERROR_STATUS_WORD         = captain->parse_long();
  msg.PRESSURE                  = captain->parse_long();
  msg.PRESSURE_SENSOR_VARIANCE  = captain->parse_long();
  msg.LEAK_STATUS               = captain->parse_byte();
  msg.LEAK_A_COUNT              = captain->parse_int();
  msg.LEAK_B_COUNT              = captain->parse_int();
  msg.TX_VOLTAGE                = captain->parse_int();
  msg.TX_CURRENT                = captain->parse_int();
  msg.TRANCDUCER_IMPEDANCE      = captain->parse_int();      
  dvl_PD0_Varaibleleader_pub.publish(msg);
}
void RosInterFace::captain_callback_DVL_BOTTOMTRACK() {
  lolo_msgs::PD0_Bottomtrack msg;
  msg.timeStamp             = captain->parse_llong();
  msg.BEAM_1_BT_VEL         = captain->parse_float();
  msg.BEAM_1_BT_RANGE       = captain->parse_float();
  msg.BEAM_1_REF_LAYER_VEL  = captain->parse_float();
  msg.BEAM_1_EVAL_AMP       = captain->parse_byte();
  msg.BEAM_1_BT_CORR        = captain->parse_byte();
  msg.BM_1_REF_CORR         = captain->parse_byte();
  msg.BM_1_REF_INT          = captain->parse_byte();
  msg.BM_1_PERCENT_GOOD     = captain->parse_byte();
  msg.BM_1_RSSI_AMP         = captain->parse_byte();
  msg.BEAM_2_BT_VEL         = captain->parse_float();
  msg.BEAM_2_BT_RANGE       = captain->parse_float();
  msg.BEAM_2_REF_LAYER_VEL  = captain->parse_float();
  msg.BEAM_2_EVAL_AMP       = captain->parse_byte();
  msg.BEAM_2_BT_CORR        = captain->parse_byte();
  msg.BM_2_REF_CORR         = captain->parse_byte();
  msg.BM_2_REF_INT          = captain->parse_byte();
  msg.BM_2_PERCENT_GOOD     = captain->parse_byte();
  msg.BM_2_RSSI_AMP         = captain->parse_byte();
  msg.BEAM_3_BT_VEL         = captain->parse_float();
  msg.BEAM_3_BT_RANGE       = captain->parse_float();
  msg.BEAM_3_REF_LAYER_VEL  = captain->parse_float();
  msg.BEAM_3_EVAL_AMP       = captain->parse_byte();
  msg.BEAM_3_BT_CORR        = captain->parse_byte();
  msg.BM_3_REF_CORR         = captain->parse_byte();
  msg.BM_3_REF_INT          = captain->parse_byte();
  msg.BM_3_PERCENT_GOOD     = captain->parse_byte();
  msg.BM_3_RSSI_AMP         = captain->parse_byte();
  msg.BEAM_4_BT_VEL         = captain->parse_float();
  msg.BEAM_4_BT_RANGE       = captain->parse_float();
  msg.BEAM_4_REF_LAYER_VEL  = captain->parse_float();
  msg.BEAM_4_EVAL_AMP       = captain->parse_byte();
  msg.BEAM_4_BT_CORR        = captain->parse_byte();
  msg.BM_4_REF_CORR         = captain->parse_byte();
  msg.BM_4_REF_INT          = captain->parse_byte();
  msg.BM_4_PERCENT_GOOD     = captain->parse_byte();
  msg.BM_4_RSSI_AMP         = captain->parse_byte();
  msg.PINGS_PER_ENSEMBLE    = captain->parse_int();
  msg.BT_MAX_DEPTH          = captain->parse_int();
  msg.REF_LAYER_MIN         = captain->parse_int();
  msg.REF_LAYER_NEAR        = captain->parse_int();
  msg.REF_LAYER_FAR         = captain->parse_int();
  msg.GAIN                  = captain->parse_byte();
  dvl_PD0_Bottomtrack_pub.publish(msg);
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
  char status         = captain->parse_byte();

  sensor_msgs::NavSatFix msg;
  msg.header.stamp = ros::Time(sec,usec*1000);
  msg.header.seq = sequence;
  msg.header.frame_id = "lolo/gps_link";
  msg.latitude = 180.0*lat / PI;
  msg.longitude = 180.0*lon / PI;
  msg.status.status = status; //Set as the status char from GPS 
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
  msg.header.frame_id = "lolo/imu_link";

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
  msg.header.frame_id = "lolo/compass_link";
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
  float temperature     = captain->parse_float();
  float temperature_var = captain->parse_float();

  sensor_msgs::FluidPressure msg;
  msg.header.stamp = ros::Time(sec,usec*1000);
  msg.header.seq = sequence;
  msg.header.frame_id = "lolo/pressure_link";
  msg.fluid_pressure = pressure;
  msg.variance = variance;
  pressure_pub.publish(msg);

  sensor_msgs::Temperature temp_msg;
  temp_msg.header.stamp = ros::Time(sec,usec*1000);
  temp_msg.header.seq = sequence;
  temp_msg.header.frame_id = "lolo/pressure_link";
  temp_msg.temperature = temperature;
  temp_msg.variance = temperature_var;
  watertemp_pub.publish(temp_msg);
}

void RosInterFace::captain_callback_VBS() {

  //TODO add another message for this and add real data.
  lolo_msgs::VbsMode mode_msg;
  mode_msg.current_mode = lolo_msgs::VbsMode::VBS_MODE_AUTO;
  VBS_mode_pub.publish(mode_msg);
  
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
    msg.header.frame_id = "/lolo/vbs_front_link";
    msg.percent_current = vbs_front_tank_percent_current;
    msg.percent_target = vbs_front_tank_percent_target;
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
    msg.header.frame_id = "/lolo/vbs_aft_link";
    msg.percent_current = vbs_aft_tank_percent_current;
    msg.percent_target = vbs_aft_tank_percent_target;
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

  float ROTX        = captain->parse_float();
  float ROTY        = captain->parse_float();
  float ROTZ        = captain->parse_float();

  float q1          = captain->parse_float();
  float q2          = captain->parse_float();
  float q3          = captain->parse_float();
  float q4          = captain->parse_float();

  float position_cov_xx = captain->parse_float();
  float position_cov_yy = captain->parse_float();
  float position_cov_xy = captain->parse_float();


  //publish position (lat lon)
  geographic_msgs::GeoPoint pos_msg;

  //pos_msg.header.stamp = ros::Time(sec,usec*1000);
  //pos_msg.header.seq = sequence;
  //pos_msg.header.frame_id = "lolo/base_link";
  
  pos_msg.latitude = lat*(180.0 / PI);
  pos_msg.longitude = lon * (180.0 / PI);
  status_position_pub.publish(pos_msg);
 
  //publish depth
  smarc_msgs::FloatStamped depth_msg;
  depth_msg.data = depth;
  depth_msg.header.stamp = ros::Time(sec,usec*1000);
  depth_msg.header.seq = sequence;
  depth_msg.header.frame_id = "lolo/base_link";
  status_depth_pub.publish(depth_msg);

  //publish velocities
  geometry_msgs::TwistWithCovarianceStamped twist_msg;
  twist_msg.header.stamp = ros::Time(sec,usec*1000);
  twist_msg.header.seq = sequence;
  twist_msg.header.frame_id = "lolo/base_link";
  twist_msg.twist.twist.linear.x = SOGX;
  twist_msg.twist.twist.linear.y = SOGY;
  twist_msg.twist.twist.linear.z = SOGZ;
  twist_msg.twist.twist.angular.x = ROTX;
  twist_msg.twist.twist.angular.y = ROTY;
  twist_msg.twist.twist.angular.z = ROTZ;
  status_twist_pub.publish(twist_msg);

  //Quaternion message
  //publish orientation
  geometry_msgs::Quaternion orientation_msg;  
  orientation_msg.w = q1;
  orientation_msg.x = q2;
  orientation_msg.y = q3;
  orientation_msg.z = q4;

  smarc_msgs::LatLonToUTMOdometry srv;
  //TODO header
  //srv.request.lat_lon_odom.header.stamp = ros::Time(sec,usec*1000);
  //srv.request.lat_lon_odom.header.seq = sequence;
  //srv.request.lat_lon_odom.header.frame_id = "lolo/base_link";
  srv.request.lat_lon_odom.lat_lon_pose.position = pos_msg;
  srv.request.lat_lon_odom.lat_lon_pose.orientation = orientation_msg;
  srv.request.lat_lon_odom.twist = twist_msg.twist;
  
  if (odom_client.call(srv)) {
    //ROS_INFO("Call to service sucsessfull");
    //Add header
    srv.response.odom.header.stamp = ros::Time(sec,usec*1000);
    srv.response.odom.header.seq = sequence;
    srv.response.odom.header.frame_id = "lolo/base_link";
    //Add depth
    srv.response.odom.pose.pose.position.z = -depth;

    srv.response.odom.pose.covariance[0] = position_cov_xx;
    srv.response.odom.pose.covariance[1] = position_cov_xy;
    srv.response.odom.pose.covariance[6] = position_cov_xy;
    srv.response.odom.pose.covariance[7] = position_cov_yy;

    odom_pub.publish(srv.response.odom);
  }
  else
  {
    //ROS_ERROR("Failed to call service to convert position to odom");
  }
};

void RosInterFace::captain_callback_FLS() {
  //FLS
  uint64_t timestamp    = captain->parse_llong();
  uint64_t sec = timestamp / 1000000;
  uint64_t usec = timestamp % 1000000;
  uint32_t sequence     = captain->parse_long();
  
  float range        = captain->parse_float();
  float confidence   = captain->parse_float();
  
  //publish depth if confidence = 100%
  if(confidence > 99) {
    smarc_msgs::FloatStamped fls_msg;
    fls_msg.data = range;
    fls_msg.header.stamp = ros::Time(sec,usec*1000);
    fls_msg.header.seq = sequence;
    fls_msg.header.frame_id = "lolo/fls_link";
    fls_pub.publish(fls_msg);  
  }
};

void RosInterFace::captain_callback_SENSOR_STATUS() {
  bool DVL_OK = captain->parse_byte();

  smarc_msgs::SensorStatus dvl_msg;
  dvl_msg.sensor_status = DVL_OK;
  dvl_msg.service_name = "/lolo/core/toggle_dvl";
  dvl_status_pub.publish(dvl_msg);
};

void RosInterFace::captain_callback_CTRL_STATUS() {
  bool scientistinterface_enable_waypoint = captain->parse_byte();
  bool scientistinterface_enable_yaw      = captain->parse_byte();
  bool scientistinterface_enable_yawrate  = captain->parse_byte();
  bool scientistinterface_enable_depth    = captain->parse_byte();
  bool scientistinterface_enable_altitude = captain->parse_byte();
  bool scientistinterface_enable_pitch    = captain->parse_byte();
  bool scientistinterface_enable_speed    = captain->parse_byte();
  bool scientistinterface_enable_rpm      = captain->parse_byte();
  bool scientistinterface_enable_rpm_strb = captain->parse_byte();
  bool scientistinterface_enable_rpm_port = captain->parse_byte();
  bool scientistinterface_enable_elevator = captain->parse_byte();
  bool scientistinterface_enable_rudder   = captain->parse_byte();
  bool scientistinterface_enable_VBS      = captain->parse_byte();

  smarc_msgs::ControllerStatus msg_waypoint;
  msg_waypoint.control_status = scientistinterface_enable_waypoint;
  msg_waypoint.service_name = "/lolo/ctrl/toggle_onboard_waypoint_ctrl";
  ctrl_status_waypoint_pub.publish(msg_waypoint);

  smarc_msgs::ControllerStatus msg_yaw;
  msg_yaw.control_status = scientistinterface_enable_yaw;
  msg_yaw.service_name = "/lolo/ctrl/toggle_onboard_yaw_ctrl";
  ctrl_status_yaw_pub.publish(msg_waypoint);

  smarc_msgs::ControllerStatus msg_yawrate;
  msg_yawrate.control_status = scientistinterface_enable_yawrate;
  msg_yawrate.service_name = "/lolo/ctrl/toggle_onboard_yawrate_ctrl";
  ctrl_status_yawrate_pub.publish(msg_yawrate);

  smarc_msgs::ControllerStatus msg_depth;
  msg_depth.control_status = scientistinterface_enable_depth;
  msg_depth.service_name = "/lolo/ctrl/toggle_onboard_depth_ctrl";
  ctrl_status_depth_pub.publish(msg_depth);

  smarc_msgs::ControllerStatus msg_altitude;
  msg_altitude.control_status = scientistinterface_enable_altitude;
  msg_altitude.service_name = "/lolo/ctrl/toggle_onboard_altitude_ctrl";
  ctrl_status_altitude_pub.publish(msg_altitude);

  smarc_msgs::ControllerStatus msg_pitch;
  msg_pitch.control_status = scientistinterface_enable_pitch;
  msg_pitch.service_name = "/lolo/ctrl/toggle_onboard_pitch_ctrl";
  ctrl_status_pitch_pub.publish(msg_pitch);

  smarc_msgs::ControllerStatus msg_speed;
  msg_speed.control_status = scientistinterface_enable_speed;
  msg_speed.service_name = "/lolo/ctrl/toggle_onboard_speed_ctrl";
  ctrl_status_speed_pub.publish(msg_speed);
};


void RosInterFace::captain_callback_SERVICE() {
  std::cout << "Received service response from captain" << std::endl;
  lolo_msgs::CaptainService msg;
  msg.ref = captain->parse_int();
  msg.reply = captain->parse_byte();
  //TODO Add data to array if it ever gets used
  service_pub.publish(msg);
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
