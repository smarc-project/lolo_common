#include "UDPInterface.h"

#include <iostream>
#include <stdio.h>


// Constructor
UDPInterface::UDPInterface() {
  //
};

void UDPInterface::setup(boost::asio::ip::udp::socket* socket, boost::asio::ip::udp::endpoint* endpoint) {
  
  udpSocket = socket;
  lolo_endpoint = endpoint;
  //start thread for reading
  readThread = new std::thread(std::bind(&UDPInterface::readData, this));
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
