#include "captain_interface/RosInterFace/RosInterFace.h"

void RosInterFace::init(ros::NodeHandle* nh, CaptainInterFace* cap) { 
  n = nh; captain = cap; 

  //==================================//
  //=========== Subscribers ==========//
  //==================================//

  //information / other things
  heartbeat_sub  = n->subscribe<std_msgs::Empty>("/lolo/heartbeat", 1, &RosInterFace::ros_callback_heartbeat, this);
  done_sub  = n->subscribe<std_msgs::Empty>("/lolo/mission_complete", 1, &RosInterFace::ros_callback_done, this);
  done_sub  = n->subscribe<std_msgs::Empty>("/lolo/abort", 1, &RosInterFace::ros_callback_abort, this);

  //Control commands: High level
  waypoint_sub  = n->subscribe<smarc_msgs::LatLonStamped>("/lolo/core/waypoint_cmd"          ,1, &RosInterFace::ros_callback_waypoint, this);
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
  rudder_sub      = n->subscribe<std_msgs::Float32>("/lolo/core/rudder_cmd"   ,1, &RosInterFace::ros_callback_rudder, this);
  elevator_sub    = n->subscribe<std_msgs::Float32>("/lolo/core/elevator_cmd" ,1, &RosInterFace::ros_callback_elevator, this);

  //menu
  menu_sub        = n->subscribe<std_msgs::String>("/lolo/console_in", 1, &RosInterFace::ros_callback_menu, this);

  //==================================//
  //=========== Publishers ===========//
  //==================================//
  // --- Thrusters --- //
  thrusterPort_pub     = n->advertise<smarc_msgs::ThrusterFeedback>("/lolo/core/thruster_port_fb", 10);
  thrusterStrb_pub     = n->advertise<smarc_msgs::ThrusterFeedback>("/lolo/core/thruster_strb_fb", 10);

  // --- Rudders --- //
  rudder_angle_pub    = n->advertise<smarc_msgs::FloatStamped>("/lolo/core/rudder_fb/angle", 10);

  // --- Elevator --- //
  elevator_angle_pub      = n->advertise<smarc_msgs::FloatStamped>("/lolo/core/elevator_fb/angle", 10);

  // --- Elevons --- //
  elevon_port_angle_pub   = n->advertise<smarc_msgs::FloatStamped>("/lolo/core/elevon_port_fb/angle", 10);
  elevon_strb_angle_pub   = n->advertise<smarc_msgs::FloatStamped>("/lolo/core/elevon_strb_fb/angle", 10);

  //Leak sensors
  leak_dome   = n->advertise<smarc_msgs::Leak>("/lolo/leak", 10);


  //sensors
  imu_pub           = n->advertise<sensor_msgs::Imu>("/lolo/core/imu", 10);
  magnetometer_pub  = n->advertise<sensor_msgs::MagneticField>("/lolo/core/mag", 10);
  dvl_pub           = n->advertise<cola2_msgs::DVL>("/lolo/core/dvl", 10);
  gps_pub           = n->advertise<sensor_msgs::NavSatFix>("/lolo/core/gps", 10);
  pressure_pub      = n->advertise<sensor_msgs::FluidPressure>("/lolo/core/pressure", 10);

  //control / status
  status_orientation_pub  = n->advertise<geometry_msgs::QuaternionStamped>("/lolo/core/state/orientation", 10);
  status_altitude_pub     = n->advertise<smarc_msgs::FloatStamped>("/lolo/core/state/altitude", 10);
  status_position_pub     = n->advertise<smarc_msgs::LatLonStamped>("/lolo/core/state/position",10);
  status_depth_pub        = n->advertise<smarc_msgs::FloatStamped>("/lolo/core/state/depth",10);
  status_twist_pub        = n->advertise<geometry_msgs::TwistWithCovarianceStamped>("lolo/core/state/twist",10);
  control_status_pub      = n->advertise<lolo_msgs::CaptainStatus>("/lolo/core/control_status", 10);

  //General purpose text message
  text_pub   = n->advertise<std_msgs::String>("/lolo/text", 10);

  //Lolo console menu
  menu_pub  = n->advertise<std_msgs::String>("/lolo/console_out", 10);
};