#include "CANf303.hpp"

STM32CAN can;

volatile uint32_t receivedCount = 0;

void onCanReceive(twai_message_t msg)
{
  
  /*
    receivedCount++;

    Serial.println("=== can RX ===");
    Serial.printf("ID   : 0x%lX\n", msg.identifier);
    Serial.printf("EXTD : %lu\n", msg.extd);
    Serial.printf("RTR  : %lu\n", msg.rtr);
    Serial.printf("DLC  : %u\n", msg.data_length_code);

    Serial.print("DATA : ");

    for (uint8_t i = 0; i < msg.data_length_code; i++) {
        Serial.printf("%02X ", msg.data[i]);
    }

    Serial.println();*/
}

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println("can loopback test");

    if (!can.begin(500000, PA12_PA11)) {
        Serial.println("失敗can initialization failed");
        while (1);
    }

    can.onReceive(onCanReceive);

    Serial.println("成功can initialized");
}

void loop() {
    Serial.println("before send");

    twai_message_t msg = {};
    msg.extd = STANDARD_FORMAT;
    msg.rtr = DATA_FRAME;
    msg.identifier = 0x123;
    msg.data_length_code = 8;

    for (int i = 0; i < 8; i++) {
        msg.data[i] = i;
    }

    Serial.println("calling send");
    bool result = can.send(msg);
    Serial.println("send returned");

    Serial.printf("send result = %d\n", result);

    delay(1000);
}