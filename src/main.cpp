#include "STM32F303K8_CAN.hpp"

STM32CAN can;

void setup(){
  Serial.begin(115200);
  can.begin(1000000, PA11_PA12);
}

void loop(){
    CAN_msg_t rx;

    if(can.receive(&rx)){
        Serial.println(rx.id, HEX);
        Serial.println(rx.data[0], HEX);
        Serial.println(rx.data[1], HEX);
    }

    CAN_msg_t msg;

    msg.id = 0x123;
    msg.format = STANDARD_FORMAT;
    msg.type = DATA_FRAME;
    msg.len = 2;
    msg.data[0] = 0x11;
    msg.data[1] = 0x22;


    can.send(msg);
    delay(1000);
}
