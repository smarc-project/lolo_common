#include "captain_interface/RosInterFace/RosInterFace.h"

void RosInterFace::init(ros::NodeHandle* nh, CaptainInterFace* cap) { 
  n = nh; captain = cap; 

  //==================================//
  //=========== Subscribers ==========//
  //==================================//

  //information / other things
  heartbeat_sub  = n->subscribe<std_msgs::Empty>("/lolo/Heartbeat", 1, &RosInterFace::ros_callback_heartbeat, this);
  done_sub  = n->subscribe<std_msgs::Empty>("/lolo/done", 1, &RosInterFace::ros_callback_done, this);
  done_sub  = n->subscribe<std_msgs::Empty>("/lolo/abort", 1, &RosInterFace::ros_callback_abort, this);

  //Control commands: High level
  waypoint_sub  = n->subscribe<geometry_msgs::Point>("/lolo/core/waypoint_cmd"          ,1, &RosInterFace::ros_callback_waypoint, this);
  //waypoint_sub_UTM  = n->subscribe<smarc_msgs::UTMpoint>("/lolo/core/UTMwaypoint_cmd"   ,1, &RosInterFace::ros_callback_UTMwaypoint, this);
  speed_sub     = n->subscribe<std_msgs::Float32>("/lolo/core/speed_cmd"       ,1, &RosInterFace::ros_callback_speed,this);
  depth_sub     = n->subscribe<std_msgs::Float32>("/lolo/core/depth_cmd"       ,1, &RosInterFace::ros_callback_depth,this);
  altitude_sub  = n->subscribe<std_msgs::Float32>("/lolo/core/altitude_cmd"    ,1, &RosInterFace::ros_callback_altitude,this);

  //Control commands medium level
  yaw_sub       = n->subscribe<std_msgs::Float32>("/lolo/core/yaw_cmd"          ,1, &RosInterFace::ros_callback_yaw,this);
  yawrate_sub   = n->subscribe<std_msgs::Float32>("/lolo/core/yawrate_cmd"      ,1, &RosInterFace::ros_callback_yawrate,this);
  pitch_sub     = n->subscribe<std_msgs::Float32>("/lolo/core/pitch_cmd"        ,1, &RosInterFace::ros_callback_pitch,this);
  rpm_sub       = n->subscribe<std_msgs::Float32>("/lolo/core/rpm_cmd"          ,1, &RosInterFace::ros_callback_rpm, this);

  //Control commands low level
  //Thruster
  thrusterPort_sub = n->subscribe<std_msgs::Float32>("/lolo/core/thruster_port_cmd", 1, &RosInterFace::ros_callback_thrusterPort, this);
  thrusterStrb_sub = n->subscribe<std_msgs::Float32>("/lolo/core/thruster_strb_cmd", 1, &RosInterFace::ros_callback_thrusterStrb, this);

  //control surfaces
  rudder_sub  = n->subscribe<std_msgs::Float32>("/lolo/core/rudder_cmd", 1, &RosInterFace::ros_callback_rudder, this);
  elevator_sub    = n->subscribe<std_msgs::Float32>("/lolo/core/elevator_cmd", 1, &RosInterFace::ros_callback_elevator, this);

  //menu
  menu_sub        = n->subscribe<std_msgs::String>("/lolo/console_in", 1, &RosInterFace::ros_callback_menu, this);

  //==================================//
  //=========== Publishers ===========//
  //==================================//
  // --- Thrusters --- //
  thrusterPort_rpm_pub     = n->advertise<std_msgs::Float32>("/lolo/core/thruster_port_fb/rpm", 10);
  thrusterPort_current_pub = n->advertise<std_msgs::Float32>("/lolo/core/thruster_port_fb/current", 10);
  thrusterPort_torque_pub  = n->advertise<std_msgs::Float32>("/lolo/core/thruster_port_fb/torque", 10);

  thrusterStrb_rpm_pub     = n->advertise<std_msgs::Float32>("/lolo/core/thruster_strb_fb/rpm", 10);
  thrusterStrb_current_pub = n->advertise<std_msgs::Float32>("/lolo/core/thruster_strb_fb/current", 10);
  thrusterStrb_torque_pub  = n->advertise<std_msgs::Float32>("/lolo/core/thruster_strb_fb/torque", 10);

  // --- Rudders --- //
  rudder_angle_pub    = n->advertise<std_msgs::Float32>("/lolo/core/rudder_fb/angle", 10);
  rudder_current_pub  = n->advertise<std_msgs::Float32>("/lolo/core/rudder_fb/current", 10);

  // --- Elevator --- //
  elevator_angle_pub      = n->advertise<std_msgs::Float32>("/lolo/core/elevator_fb/angle", 10);
  elevator_current_pub    = n->advertise<std_msgs::Float32>("/lolo/core/elevator_fb/current", 10);

  // --- Elevons --- //
  elevon_port_angle_pub   = n->advertise<std_msgs::Float32>("/lolo/core/elevon_port_fb/angle", 10);
  elevon_port_current_pub = n->advertise<std_msgs::Float32>("/lolo/core/elevon_port_fb/current", 10);

  elevon_strb_angle_pub   = n->advertise<std_msgs::Float32>("/lolo/core/elevon_strb_fb/angle", 10);
  elevon_strb_current_pub = n->advertise<std_msgs::Float32>("/lolo/core/elevon_strb_fb/current", 10);


  //sensors
  imu_pub           = n->advertise<sensor_msgs::Imu>("/lolo/core/imu", 10);
  magnetometer_pub  = n->advertise<sensor_msgs::MagneticField>("/lolo/core/mag", 10);
  dvl_pub           = n->advertise<smarc_msgs::DVL>("/lolo/core/dvl", 10);
  gps_pub           = n->advertise<sensor_msgs::NavSatFix>("/lolo/core/gps", 10);
  pressure_pub      = n->advertise<sensor_msgs::FluidPressure>("/lolo/core/pressure", 10);

  //control / status
  status_altitude_pub     = n->advertise<std_msgs::Float32>("/lolo/core/state/altitude", 10);
  status_position_pub     = n->advertise<geometry_msgs::PoseWithCovarianceStamped>("/lolo/core/state/position",10);
  //status_position_pub_UTM = n->advertise<smarc_msgs::UTMposeStamped>("/lolo/core/state/position_UTM",10);
  status_twist_pub        = n->advertise<geometry_msgs::TwistWithCovarianceStamped>("lolo/core/state/twist",10);
  control_status_pub      = n->advertise<captain_interface::CaptainStatus>("/lolo/core/control_status", 10);

  //General purpose text message
  text_pub   = n->advertise<std_msgs::String>("/lolo/text", 10);

  //Lolo console menu
  menu_pub  = n->advertise<std_msgs::String>("/lolo/console_out", 10);
};