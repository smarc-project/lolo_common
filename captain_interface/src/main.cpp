#include "ros/ros.h"
#include <stdio.h>
#include <sstream>
#include <boost/chrono.hpp>
#include <boost/thread/thread.hpp>
#include "captain_interface/RosInterFace/RosInterFace.h"
//#include "captain_interface/TcpInterFace/TcpInterFace.h"
#include "captain_interface/UDPInterface/UDPInterface.h"
#include <stdint.h>

#define PORT 8888

UDPInterface captain;
RosInterFace rosInterface;

//TODO use boost::bind to skip this step
void callback_captain() { rosInterface.captain_callback(); };

#include <iostream>
#include <boost/asio.hpp>

using namespace boost::asio;
using ip::udp;
using std::string;
using std::cout;
using std::endl;


int main(int argc, char *argv[]) {

  printf("main::ros init\n");

  ros::init(argc,argv, "CaptainInterface");

  //Init subscribers and publishers
  ros::NodeHandle n;
  rosInterface.init(&n, &captain);

  //Set callback
  captain.setCallback(callback_captain);

  //TODO get this from rosparam
  ip::address lolo_ip = ip::address::from_string("192.168.0.90");
  int lolo_port = 8888;

  //Create udp socket
  boost::asio::io_service io_service;
  udp::endpoint receiver_endpoint;
  receiver_endpoint.address(lolo_ip);
  receiver_endpoint.port(lolo_port);

  //udp::socket socket(io_service);
  //socket.open(udp::v4());
  udp::socket socket(io_service, udp::endpoint(udp::v4(), 8888));
  //socket.open(udp::v4());
  captain.setup(&socket, &receiver_endpoint);
  
  ros::Rate loop_rate(100);
  while(ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
    //captain.loop(); //send heartbeat to lolo?
  }

  captain.stop();
  //Clear UDP socket
 return 0;
}
