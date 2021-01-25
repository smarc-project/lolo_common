#include <captain_interface/UDPInterface/UDPInterface.h>

#include <iostream>
#include <boost/asio.hpp>
#include <stdio.h>
#include <boost/thread.hpp>
#include <boost/array.hpp>

//boost::asio::ip::tcp::socket* tcpSocket;
boost::thread* readThread;
boost::array<char, 256> rbuf; //receive buffer

// Constructor
UDPInterface::UDPInterface() {
  //
};

void UDPInterface::setup(boost::asio::ip::udp::socket* socket, boost::asio::ip::udp::endpoint* endpoint) {
  udpSocket = socket;
  lolo_endpoint = endpoint;
  //start thread for reading
  readThread = new boost::thread(boost::bind(&UDPInterface::readData, this));
};

void UDPInterface::loop(){ /*DO something?*/ };

void UDPInterface::readData() {
  printf("Reading started\n");
  bool ok = true;
  while(ok && !stopped) {
    try
    {
      udp::endpoint sender_endpoint;
      size_t len = udpSocket->receive_from(boost::asio::buffer(rbuf,256), sender_endpoint);
      //printf("Received %d bytes\n", (int) len);
      for(uint16_t i =0;i<len;i++) {
        parse_data(rbuf[i]);
      }
    }
    catch (std::exception& e)
    {
      std::cerr << e.what() << std::endl;
    }
  }
  printf("Reading done!\n");
}

bool UDPInterface::send_data(char* buf, uint8_t len) {
  //printf("Sending data\n");
  udpSocket->send_to(boost::asio::buffer(buf,len), *lolo_endpoint);
  
  return true;
}
