#ifndef CIRCLEBUFFER_H
#define CIRCLEBUFFER_H

#define CIRCLEBUFFER_SIZE 255
//----------------------------------------------------------------
//-------------Circular buffer for incoming data------------------
//----------------------------------------------------------------
class CircleBuffer {
  private:
  uint8_t buffer[CIRCLEBUFFER_SIZE];
  uint16_t head;

  public:

  CircleBuffer() {
    head = 0;
  };

  void put(uint8_t data) {
    head = (head+1) % CIRCLEBUFFER_SIZE;
    buffer[head] = data;
  };

  uint8_t get(int16_t index) {
    int16_t real_index = (head - index);
    if(real_index > 0) real_index = real_index % CIRCLEBUFFER_SIZE;
    while(real_index < 0) real_index+=CIRCLEBUFFER_SIZE;
    return buffer[real_index];
  };

  /*
  //TODO fast copy function
  void print(Stream &s) {
    s.println("CircleBuffer");
    s.print("Head : "); Serial.println(head);
    Serial.print("Data: ");
    for(int i=0;i<CIRCLEBUFFER_SIZE;i++) {
      if(i == head) s.print('|');
      s.print("0x"); s.print(buffer[i],HEX);
      if(i == head) s.print('|');
      else s.print(',');
    }
    Serial.println("");
  };
  */
};
//----------------------------------------------------------------
#endif
