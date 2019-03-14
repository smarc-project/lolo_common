#ifndef Arduino_H
#define Arduino_H

#define LOW 0
#define HIGH 1

//#include <string.h>
#include <string>
#include <cmath>
#include <stdint.h>


using namespace std;

typedef bool boolean;
typedef uint8_t byte;
//typedef std::string String;

class String {
  string _str = "";
public:

  String(const char* c) {
    _str = c;
  };

  String(std::string s) {
    _str = s;
  };
  ~String(){};

  int length() {
    return _str.length();
  };

  char charAt(int i) {
    if (i<length())
      return _str[i];
    return 0;
  };

  void concat(char c) {
    _str += c;
  };
};


static void pinMode(int a, int b) {
    //bla bla
};

//TODO do this
static uint16_t millis() {
  return 3;
};

static uint8_t lowByte(uint16_t in) {
 return in & 0b0000000011111111;
}

static uint8_t highByte(uint16_t in) {
  return in >> 8;
}

static void digitalWrite(int Tx_pin, int value) {

};

static void delayMicroseconds(uint16_t us) {
  return;
}

/*
template <class T>
T abs(T in) {
  if (in < 0)
    return -in;
  return in;
}
*/

#endif //Arduino_H
