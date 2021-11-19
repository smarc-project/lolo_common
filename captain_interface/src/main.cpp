#include "ros/ros.h"
#include <stdio.h>
#include <sstream>
#include <boost/chrono.hpp>
#include <boost/thread/thread.hpp>
#include "captain_interface/RosInterFace/RosInterFace.h"
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

  //parameters
  int lolo_port = 8888;
  std::string lolo_ip_str;

  ros::param::param<std::string>("~captain_ip", lolo_ip_str, "192.168.1.90");
  ip::address lolo_ip = ip::address::from_string(lolo_ip_str);

  ROS_INFO("Captain ip address: %s", lolo_ip_str.c_str());

  //Create udp socket
  boost::asio::io_service io_service;
  udp::endpoint receiver_endpoint;
  receiver_endpoint.address(lolo_ip);
  receiver_endpoint.port(lolo_port);

  udp::socket socket(io_service, udp::endpoint(udp::v4(), 8888));
  captain.setup(&socket, &receiver_endpoint);
  
  //Send something to the captain so it can get the ip of the scientist computer
  captain.new_package(0);
  captain.send_package();
  
  int i=0;
  ros::Rate loop_rate(1000);
  while(ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
    //captain.loop(); //send heartbeat to lolo?

    if(i > 1000) {
      //Send something to the captain so it can get the ip of the scientist computer
      captain.new_package(0);
      captain.send_package();
      i=0;
    }
    i++;
  }

  captain.stop();
  //Clear UDP socket
 return 0;
}
