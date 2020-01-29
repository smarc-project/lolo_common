#include "ros/ros.h"
#include <stdio.h>
#include <sstream>
#include <boost/chrono.hpp>
#include <boost/thread/thread.hpp>
#include "captain_interface/RosInterFace/RosInterFace.h"
#include "captain_interface/TcpInterFace/TcpInterFace.h"
#include <stdint.h>

#define PORT 8888

TcpInterFace captain;
RosInterFace rosInterface;

//TODO use boost::bind to skip this step
void callback_captain() { rosInterface.captain_callback(); };

#include <iostream>
#include <boost/asio.hpp>

using namespace boost::asio;
using ip::tcp;
using std::string;
using std::cout;
using std::endl;


int main(int argc, char *argv[]) {

  printf("main::ros init\n");

  ros::init(argc,argv, "CaptainInterface");

  ros::NodeHandle n;
  //Init subscribers and publishers
  rosInterface.init(&n, &captain);

  printf("Set callback\n");
  captain.setCallback(callback_captain);

  printf("setup\n");
  
  boost::asio::io_service io_service;
  //listen for new connection
  tcp::acceptor acceptor_(io_service, tcp::endpoint(tcp::v4(), PORT ));
  //socket creation
  tcp::socket socket_(io_service);

  ros::Rate loop_rate(100);
  while(ros::ok()) {
    //waiting for connection
    printf("Waiting for connection\n");
    acceptor_.accept(socket_);
    captain.setup(&socket_);
    while(ros::ok() && socket_.is_open()) {
      ros::spinOnce();
      loop_rate.sleep();
    }
    socket_.close();
  }
 return 0;
}
