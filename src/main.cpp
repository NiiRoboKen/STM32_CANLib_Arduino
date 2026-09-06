//stty -F /dev/ttyACM0 115200 raw -echo && cat /dev/ttyACM0

#include <Arduino.h>
#include "CANf303.hpp"

STM32CAN can;

void RxCallBack(twai_message_t msg){
  //10進数表示
  Serial.println(msg.identifier); //291
  //16進数表示
  Serial.println(msg.identifier, HEX); //123
  
  Serial.println(msg.data[0]); //0
  Serial.println(msg.data[1]); //1
  Serial.println(msg.data[2]); //2
  Serial.println(msg.data[3]); //3
  Serial.println(msg.data[4]); //4
  Serial.println(msg.data[5]); //5
  Serial.println(msg.data[6]); //6
  Serial.println(msg.data[7]); //7
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
