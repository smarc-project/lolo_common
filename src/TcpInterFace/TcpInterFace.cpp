#include <captain_interface/TcpInterFace/TcpInterFace.h>

#include <iostream>
#include <boost/asio.hpp>
#include <stdio.h>
#include <boost/thread.hpp>
#include <boost/array.hpp>

boost::asio::ip::tcp::socket* tcpSocket;
boost::thread* readThread;
boost::array<char, 128> rbuf; //receive buffer

// Constructor
TcpInterFace::TcpInterFace() {
  //
};

void TcpInterFace::setup(boost::asio::ip::tcp::socket* socket) {
  tcpSocket = socket;
  //start thread for reading
  readThread = new boost::thread(boost::bind(&TcpInterFace::readData, this));
};

void TcpInterFace::loop(){ /*DO something?*/ };

void TcpInterFace::readData() {
  printf("Reading started\n");
  bool ok = true;
  while(tcpSocket->is_open() && ok) {
    boost::system::error_code error;
    int n = boost::asio::read(*tcpSocket, boost::asio::buffer(rbuf), boost::asio::transfer_at_least(1), error);
    //printf("n %d\n",n);
    if (error) {
      printf("Read error\n");
      if (error == boost::asio::error::eof) {
        ok = false;
        printf("Connection closed cleanly by peer\n");
        tcpSocket->close();
      }
      else {
        ok = false;
        printf("Some other error\n");
        tcpSocket->close();
      }
      //else throw boost::system::system_error(error); // Some other error.
    }
    else {
      for(int i=0;i<n;i++) {
        //printf("received: %d\n", rbuf[i]);
        parse_data(rbuf[i]);
      }
    }
  }
  printf("Connection lost\n");
}

bool TcpInterFace::send_data(char* buf, uint8_t len) {
  //printf("Sending data\n");
  boost::system::error_code error;
  int sent = tcpSocket->write_some(boost::asio::buffer(buf, len), error);
  if(error) {
    if (error == boost::asio::error::eof) {
      printf("Connection closed cleanly by peer\n");
      throw boost::system::system_error(error); // Some other error.
      return false;
    }
    else {
      throw boost::system::system_error(error); // Some other error.
    }
  }
  return true;
}
