/*------------------------------------------------------------------------------------
	Captain scientist interface V0.2: TCP
------------------------------------------------------------------------------------*/

#ifndef CaptainInterFace_h
#define CaptainInterFace_h
#include <stdint.h>
#include "../CircleBuffer.h"
#include <string>

typedef union { char bytes[4]; long  mylong;  } conversionLong;    // Used for conversion
typedef union { char bytes[2]; int   myInt;   } conversionInt;     // Used for conversion
typedef union { char bytes[4]; float myFloat; } conversionFloat;   // Used for conversion

//----------------------------------------------------------------
class CaptainInterFace {

  conversionLong     myUnionLong;                       // Help variable for bytes-long conversion
  conversionInt      myUnionInt;                        // Help variable for bytes-inte conversion
  conversionFloat    myUnionFloat;                      // Help variable for bytes-float conversion

  //buffers
  char send_buffer[255] = {0};                    //Buffer for outgoing data
  char* send_ptr = send_buffer;
  CircleBuffer receive_buffer;                    //Buffer for incoming data
  uint8_t unpack_index;

  bool package_available = false;
  bool waitForCS = false;

  //Parse data from incoming buffer and call callback function if needed
  bool parse_package();
  uint8_t calc_checksum(char* buffer, uint8_t len);

  //msg ID
  uint8_t msgID = 255;

protected:
  //called when data is received from hardware layer
  bool parse_data(char c);

  //send data. Implemented on the hardware_layer
  virtual bool send_data(char* buf, uint8_t len) = 0;

public:
  CaptainInterFace();

  void (*cb)() = NULL;
  void setCallback(void (*f)()) {cb = f;}

  bool send_package();
  void new_package(uint8_t msgID);

  uint8_t       messageID() {return msgID;};

  void          add_byte(uint8_t b);               //
  void          add_string(std::string s);         //
  void          add_float(float val);              //
  void          add_double(double val);            //
  void          add_long(uint32_t val);            //
  void          add_llong(uint64_t val);           //
  void          add_int(int val);                  //

  void          clear_package();                   // clear incoming package
  uint8_t       parse_byte();                      //
  std::string   parse_string(int Nchars);          //
  float         parse_float();                     //
  double        parse_double();                    //
  uint32_t      parse_long();                      //
  uint64_t      parse_llong();                     //
  int           parse_int();                       //
};
//----------------------------------------------------------------
#endif
