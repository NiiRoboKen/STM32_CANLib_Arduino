#include <Arduino.h>
#include "STM32_CAN.hpp"

CanDriver can;

void canCallback(twai_message_t msg) {
  Serial.printf("RX <- ID:0x%lX DLC:%d DATA:", msg.identifier, msg.data_length_code);
  for (int i = 0; i < msg.data_length_code; i++) {
    Serial.printf(" %02X", msg.data[i]);
  }
  Serial.println();
}

void setup() {
  if(can.begin(1000E3, PA12_PA11)) {
    Serial.println("OK");
  }
  can.onReceive(canCallback);
}

void loop() {
  uint32_t id = 0x00;
  uint8_t data[8] = {1,2,3,4,5,6,7,8};
  can.send(id, data, sizeof(data));
  delay(1000);
}