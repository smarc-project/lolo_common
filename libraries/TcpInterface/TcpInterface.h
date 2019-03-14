/*------------------------------------------------------------------------------------
	Captain scientist interface V0.2: TCP
------------------------------------------------------------------------------------*/

#ifndef TcpInterFace_h
#define TcpInterFace_h

#include "CaptainInterFace.h"
#include "Arduino.h"

#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <thread>
#include <iostream>
#include <boost/chrono.hpp>

#include <stdio.h>
#include <sstream>

//----------------------------------------------------------------
class TcpInterFace : public CaptainInterFace {

  enum TcpConnection {
      NOT_CONNECTED,
      CONNECTING,
      CONNECTED,
      FAILED
  };
  int connection_status = NOT_CONNECTED;

  boost::array<char, 128> rbuf; //receive buffer
  boost::asio::io_service ios;
  boost::asio::ip::tcp::endpoint* endpoint;
  boost::asio::ip::tcp::socket* socket;

  void connect_handler(const boost::system::error_code& error);

protected:
  bool send_data(char* buf, uint8_t len);

public:
  TcpInterFace();
  void setup(std::string ip, int port);
  void loop();
  bool connected();
};
//----------------------------------------------------------------
#endif
