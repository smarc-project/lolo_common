#include <TcpInterface.h>

#define IP "192.168.1.177" // "192.168.1.64"
#define P 8888

// Constructor
TcpInterFace::TcpInterFace() {
  //Do something?
};

bool TcpInterFace::connected() {
  return socket->is_open();
}

void TcpInterFace::setup(std::string ip, int port) {
  //Connect to captain
  endpoint = new boost::asio::ip::tcp::endpoint(boost::asio::ip::address::from_string(IP), P);
  socket = new boost::asio::ip::tcp::socket(ios);
	//socket->connect(*endpoint);
};

void TcpInterFace::loop(){
  if(connection_status == TcpConnection::NOT_CONNECTED) {
    connection_status = TcpConnection::CONNECTING;
    printf("Connecting\n");
    socket->async_connect(*endpoint, boost::bind(&TcpInterFace::connect_handler, this, boost::asio::placeholders::error));
    //ios.run(); //blocking :/
  }

  //do connect stuff
  if(connection_status == TcpConnection::CONNECTING) {
    ios.run_one();
  }

  //connected
  if(connection_status == TcpConnection::CONNECTED) {
    boost::system::error_code error2;
    int n = boost::asio::read(
    *socket, boost::asio::buffer(rbuf),
    boost::asio::transfer_at_least(1), error2);
    if (error2) {
      printf("Read error\n");
    }
    else {
      for(int i=0;i<n;i++) {
        //printf("received: %d\n", rbuf[i]);
        parse_data(rbuf[i]);
      }
    }
  }
};

bool TcpInterFace::send_data(char* buf, uint8_t len) {
  //send data
  boost::system::error_code error;
  socket->write_some(boost::asio::buffer(buf, len), error);
}


//--------Handlers--------//
void TcpInterFace::connect_handler(const boost::system::error_code& error)
{
  if (!error)
  {
    // Connect succeeded.
    connection_status = TcpConnection::CONNECTED;
    printf("Handler::Connected\n");
  }
  else {
    connection_status = TcpConnection::FAILED;
    printf("Handler::Failed to connect\n");
  }
}
