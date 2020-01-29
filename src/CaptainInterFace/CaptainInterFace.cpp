#include <captain_interface/CaptainInterFace/CaptainInterFace.h>

//#include <Arduino.h>


CaptainInterFace::CaptainInterFace() {
  //Do something?
};

//----------------------------------------------------------------
uint8_t CaptainInterFace::calc_checksum(char* buffer, uint8_t len) {
  uint8_t chk = 0;
  for (int ii = 0; ii < len; ii++) { chk = chk ^ buffer[ii]; } // XOR
  return chk;
}

bool CaptainInterFace::parse_data(char c) {
  //Serial.print("received: 0x"); Serial.println(c,HEX);
  receive_buffer.put(c);
  package_available = false; //New data added. so the old package has been overwrittern
  if(waitForCS) {if(parse_package()) {waitForCS=false; return true;}} // c was CS. check if the message is correct and set package_available to TRUE
  if(c == 0x2A) {waitForCS = true; } //next byte is the Checksum
  else waitForCS = false;
  return true;
}

bool CaptainInterFace::parse_package() {
  uint8_t CS = receive_buffer.get(0);           // newest byte is CS
                                                // 2:nd newest byte is '*'
  uint8_t length = receive_buffer.get(2);       // 3:nd newest byte is length
  uint8_t start = receive_buffer.get(length-1); // start byte. Should be '#'

  //Error checks
  if(start != '#'){
    //Serial.print("not a valid message. Start=0x"); Serial.print(start,HEX); Serial.print(" length: "); Serial.println(length);
    return false;
  }//Something is wrong with the package.

  uint8_t checksum = 0;
  for (int ii = length-1; ii >0 ; ii--) { checksum = checksum ^ receive_buffer.get(ii); } // XOR
  if(checksum != CS) {/*Serial.println("Checksum error"); Serial.print("CS Should be: "); Serial.println(checksum); */return false; };

  //Serial.println("Package received");
  unpack_index = length-2;
  package_available = true;

  msgID = parse_byte();

  if(cb != NULL) {
    cb();
    clear_package();
  }

  return true;
}

void CaptainInterFace::clear_package() {
  unpack_index = 0;
  package_available = false;
}


void CaptainInterFace::new_package(uint8_t _msgID) {
  send_ptr = send_buffer; //reset send buffer
  add_byte('#'); //Add start byte
  add_byte(_msgID);
}

bool CaptainInterFace::send_package() {
  //Calcualte length
  uint8_t len = (send_ptr - send_buffer) + 3;
  add_byte(len);        //Add length

  //Calcualte Checksum
  add_byte('*');        //Add '*'
  uint8_t cs = calc_checksum(send_buffer,len-1);
  add_byte(cs);         //Add CS

  //Serial.print("Package size: "); Serial.println(len);
  bool success = send_data(send_buffer, len);
}

//----------------------------------------------------------------
//-----------------------Add data to package----------------------
//----------------------------------------------------------------
void CaptainInterFace::add_byte(uint8_t b) {
  if(send_ptr - send_buffer < 255) {
    *send_ptr = b;
    send_ptr++;
  }
}
//----------------------------------------------------------------
void CaptainInterFace::add_float(float val){
  // Concatinates a float to package (4 bytes)
  myUnionFloat.myFloat = val;
  add_byte(myUnionFloat.bytes[0]);
  add_byte(myUnionFloat.bytes[1]);
  add_byte(myUnionFloat.bytes[2]);
  add_byte(myUnionFloat.bytes[3]);
}
//----------------------------------------------------------------
void CaptainInterFace::add_double(double val){
  // Concatinates a double to package (8 bytes)
  uint8_t* data = (uint8_t*) &val;
  for(uint8_t i=0;i<8;i++)
    add_byte(data[i]);
}
//----------------------------------------------------------------
void CaptainInterFace::add_long(uint32_t val){
   // Concatinates a Long int to package (4 bytes)
  myUnionLong.mylong    = val;
  add_byte(myUnionLong.bytes[0]);
  add_byte(myUnionLong.bytes[1]);
  add_byte(myUnionLong.bytes[2]);
  add_byte(myUnionLong.bytes[3]);
}
//----------------------------------------------------------------
void CaptainInterFace::add_llong(uint64_t val) {
  // Concatinates a Long long int to package (8 bytes)
  add_long((uint32_t) ((uint64_t) val >> 32) );
  add_long((uint32_t) val);
}
//----------------------------------------------------------------
void CaptainInterFace::add_int(int val){
   // Concatinates a int to package (2 bytes)
  myUnionInt.myInt = val;
  add_byte(myUnionInt.bytes[0]);
  add_byte(myUnionInt.bytes[1]);
}


//----------------------------------------------------------------
//-----------------------get data from package--------------------
//----------------------------------------------------------------
uint8_t CaptainInterFace::parse_byte() {
  uint8_t b = receive_buffer.get(unpack_index);
  if(unpack_index > 0) unpack_index--;
  return b;
}
//----------------------------------------------------------------
std::string CaptainInterFace::parse_string(int Nchars){
  std::string s = "";
  for(int i=0;i<Nchars;i++) s += ((char) parse_byte());
  return s;
};          //
//----------------------------------------------------------------
float CaptainInterFace::parse_float(){
  // Converts 4 bytes to a float
  myUnionFloat.bytes[0] = parse_byte();
  myUnionFloat.bytes[1] = parse_byte();
  myUnionFloat.bytes[2] = parse_byte();
  myUnionFloat.bytes[3] = parse_byte();
  return myUnionFloat.myFloat;
};
//----------------------------------------------------------------
double CaptainInterFace::parse_double(){
  // Converts 8 bytes to a double
  double value = 0;
  uint8_t* data = (uint8_t*) &value;
  for(int i=0;i<8;i++)
    data[i] = parse_byte();
  return value;
};
//----------------------------------------------------------------
uint32_t  CaptainInterFace::parse_long(){
  // Converts 4 bytes to a long integer (signed)
  myUnionLong.bytes[0] = parse_byte();
  myUnionLong.bytes[1] = parse_byte();
  myUnionLong.bytes[2] = parse_byte();
  myUnionLong.bytes[3] = parse_byte();
  return myUnionLong.mylong;
};
//----------------------------------------------------------------
uint64_t CaptainInterFace::parse_llong() {
  //converts 8 bytes to long long integer (signed)
  uint32_t msb = parse_long();
  uint32_t lsb = parse_long();
  uint64_t val = (((uint64_t) msb) << 32 ) +  lsb;
  return val;
}
//----------------------------------------------------------------
int CaptainInterFace::parse_int(){
  // Converts 2 bytes to an int integer (signed)
  myUnionInt.bytes[0] = parse_byte();
  myUnionInt.bytes[1] = parse_byte();
  return myUnionInt.myInt;
}
