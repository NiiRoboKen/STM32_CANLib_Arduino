//stty -F /dev/ttyACM0 115200 raw -echo && cat /dev/ttyACM0

#include <Arduino.h>
#include "CANf303.hpp"

STM32CAN can;

void RxCallBack(twai_message_t msg){
  
}

volatile bool a = false;
void mainLoop(){
  if(!a){
    Serial.println("mainloop call!");

    twai_message_t msg{};

    msg.extd = STANDARD_FORMAT;
    msg.rtr = DATA_FRAME;
    msg.identifier = 0x123;
    msg.data_length_code = 8;

    for (int i = 0; i < 8; i++) {
      msg.data[i] = i;
    }

    Serial.println(can.send(msg));

    a=true;
  }
}

void setup(){
  Serial.begin(115200);
  can.onReceive(&RxCallBack);
  can.onMainLoop(&mainLoop);
  can.begin(1000000, PA12_PA11);
}

void loop(){}
