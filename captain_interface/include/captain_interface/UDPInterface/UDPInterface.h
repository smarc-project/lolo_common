/*------------------------------------------------------------------------------------
	Captain scientist interface V0.3: UDP
------------------------------------------------------------------------------------*/

#ifndef UDPInterface_h
#define UDPInterface_h

#include "../CaptainInterFace/CaptainInterFace.h"
#include <iostream>
#include <boost/asio.hpp>

using namespace boost::asio;
using ip::udp;
using std::string;
using std::cout;
using std::endl;


//----------------------------------------------------------------
class UDPInterface : public CaptainInterFace {
  void readData();
  bool stopped = false;
  udp::socket* udpSocket;
  udp::endpoint* lolo_endpoint;
protected:
  bool send_data(char* buf, uint8_t len);

public:
  UDPInterface();
  void setup(udp::socket* socket, udp::endpoint* endpoint);
  void loop();
  void stop() {stopped = true;};
};
//----------------------------------------------------------------
#endif
