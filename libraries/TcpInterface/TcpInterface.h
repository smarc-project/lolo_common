/*------------------------------------------------------------------------------------
	Captain scientist interface V0.2: TCP
------------------------------------------------------------------------------------*/

#ifndef TcpInterFace_h
#define TcpInterFace_h

#include "CaptainInterFace.h"
#include "Arduino.h"
#include <iostream>
#include <boost/asio.hpp>

using namespace boost::asio;
using ip::tcp;
using std::string;
using std::cout;
using std::endl;

//----------------------------------------------------------------
class TcpInterFace : public CaptainInterFace {
  void readData();
protected:
  bool send_data(char* buf, uint8_t len);

public:
  TcpInterFace();
  void setup(tcp::socket* socket);
  void loop();
};
//----------------------------------------------------------------
#endif
