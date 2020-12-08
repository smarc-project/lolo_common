#include "captain_interface/RosInterFace/RosInterFace.h"

void RosInterFace::init(ros::NodeHandle* nh, CaptainInterFace* cap) { 
  n = nh; captain = cap; 

  utm_to_latlon_client = n->serviceClient<smarc_msgs::UTMToLatLon>("utm_to_lat_lon");
  odom_client = n->serviceClient<smarc_msgs::LatLonToUTMOdometry>("lat_lon_to_utm_odom");

  //==================================//
  //=========== Subscribers ==========//
  //==================================//

  //information / other things
  heartbeat_sub  = n->subscribe<std_msgs::Empty>("/lolo/core/heartbeat", 1, &RosInterFace::ros_callback_heartbeat, this);
  done_sub  = n->subscribe<std_msgs::Empty>("/lolo/core/mission_complete", 1, &RosInterFace::ros_callback_done, this);
  done_sub  = n->subscribe<std_msgs::Empty>("/lolo/core/abort", 1, &RosInterFace::ros_callback_abort, this);

  //Control commands: High level
  waypoint_sub_UTM  = n->subscribe<geometry_msgs::Point> ("/lolo/core/waypoint_setpoint_utm" ,1, &RosInterFace::ros_callback_waypoint_utm, this);
  waypoint_sub  = n->subscribe<geographic_msgs::GeoPoint>("/lolo/ctrl/waypoint_setpoint"  ,1, &RosInterFace::ros_callback_waypoint, this);
  speed_sub     = n->subscribe<std_msgs::Float64>("/lolo/ctrl/speed_setpoint"       ,1, &RosInterFace::ros_callback_speed,this);
  depth_sub     = n->subscribe<std_msgs::Float64>("/lolo/ctrl/depth_setpoint"       ,1, &RosInterFace::ros_callback_depth,this);
  altitude_sub  = n->subscribe<std_msgs::Float64>("/lolo/ctrl/altitude_setpoint"    ,1, &RosInterFace::ros_callback_altitude,this);

  //Control commands medium level
  yaw_sub       = n->subscribe<std_msgs::Float64>("/lolo/ctrl/yaw_setpoint"          ,1, &RosInterFace::ros_callback_yaw,this);
  yawrate_sub   = n->subscribe<std_msgs::Float64>("/lolo/ctrl/yawrate_setpoint"      ,1, &RosInterFace::ros_callback_yawrate,this);
  pitch_sub     = n->subscribe<std_msgs::Float64>("/lolo/ctrl/pitch_setpoint"        ,1, &RosInterFace::ros_callback_pitch,this);
  rpm_sub       = n->subscribe<std_msgs::Float64>("/lolo/ctrl/rpm_setpoint"          ,1, &RosInterFace::ros_callback_rpm, this);

  //Control commands low level
  //Thruster
  thrusterPort_sub = n->subscribe<std_msgs::Float64>("/lolo/core/thruster_1_cmd", 1, &RosInterFace::ros_callback_thrusterPort, this);
  thrusterStrb_sub = n->subscribe<std_msgs::Float64>("/lolo/core/thruster_2_cmd", 1, &RosInterFace::ros_callback_thrusterStrb, this);

  //control surfaces
  rudder_sub      = n->subscribe<std_msgs::Float64>("/lolo/core/rudder_cmd"   ,1, &RosInterFace::ros_callback_rudder, this);
  elevator_sub    = n->subscribe<std_msgs::Float64>("/lolo/core/elevator_cmd" ,1, &RosInterFace::ros_callback_elevator, this);

  //menu
  menu_sub        = n->subscribe<std_msgs::String>("/lolo/console_in", 1, &RosInterFace::ros_callback_menu, this);

  //==================================//
  //=========== Publishers ===========//
  //==================================//
  // --- Thrusters --- //
  thrusterPort_pub     = n->advertise<smarc_msgs::ThrusterFeedback>("/lolo/core/thruster_1_fb", 10);
  thrusterStrb_pub     = n->advertise<smarc_msgs::ThrusterFeedback>("/lolo/core/thruster_2_fb", 10);

  // --- Rudders --- //
  rudder_angle_pub    = n->advertise<smarc_msgs::FloatStamped>("/lolo/core/rudder_fb", 10);

  // --- Elevator --- //
  elevator_angle_pub      = n->advertise<smarc_msgs::FloatStamped>("/lolo/core/elevator_fb", 10);

  // --- Elevons --- //
  elevon_port_angle_pub   = n->advertise<smarc_msgs::FloatStamped>("/lolo/core/elevon_port_fb", 10);
  elevon_strb_angle_pub   = n->advertise<smarc_msgs::FloatStamped>("/lolo/core/elevon_strb_fb", 10);

  //VBS
  /*
  VBS_front_tank_pub = n->advertise<lolo_msgs::VbsTank>("/lolo/core/vbs/front_tank_fb", 10);
  VBS_aft_tank_pub = n->advertise<lolo_msgs::VbsTank>("/lolo/core/vbs/aft_tank_fb", 10);
  VBS_valves_pub = n->advertise<lolo_msgs::VbsValves>("/lolo/core/vbs/valves_fb", 10);
  VBS_motor_pub = n->advertise<smarc_msgs::ThrusterFeedback>("/lolo/core/VBS/motor_fb", 10);
  */

  //Leak sensors
  leak_dome   = n->advertise<smarc_msgs::Leak>("/lolo/core/leak", 10);


  //sensors
  imu_pub           = n->advertise<sensor_msgs::Imu>("/lolo/core/imu", 10);
  magnetometer_pub  = n->advertise<sensor_msgs::MagneticField>("/lolo/core/compass", 10);
  dvl_pub           = n->advertise<smarc_msgs::DVL>("/lolo/core/dvl", 10);
  gps_pub           = n->advertise<sensor_msgs::NavSatFix>("/lolo/core/gps", 10);
  pressure_pub      = n->advertise<sensor_msgs::FluidPressure>("/lolo/core/pressure", 10);
  fls_pub           = n->advertise<smarc_msgs::FloatStamped>("/lolo/core/fls", 10);

  //control / status
  status_orientation_pub  = n->advertise<geometry_msgs::QuaternionStamped>("/lolo/core/state/orientation", 10);
  status_altitude_pub     = n->advertise<smarc_msgs::FloatStamped>("/lolo/core/state/altitude", 10);
  status_position_pub     = n->advertise<geographic_msgs::GeoPointStamped>("/lolo/dr/lat_lon",10);
  status_depth_pub        = n->advertise<smarc_msgs::FloatStamped>("/lolo/core/state/depth",10);
  status_twist_pub        = n->advertise<geometry_msgs::TwistWithCovarianceStamped>("lolo/core/state/twist",10);
  control_status_pub      = n->advertise<lolo_msgs::CaptainStatus>("/lolo/core/control_status", 10);

  //Odometry
  odom_pub                = n->advertise<nav_msgs::Odometry>("/lolo/dr/odom", 10);

  //General purpose text message
  text_pub   = n->advertise<std_msgs::String>("/lolo/text", 10);

  //Lolo console menu
  menu_pub  = n->advertise<std_msgs::String>("/lolo/console_out", 10);
};