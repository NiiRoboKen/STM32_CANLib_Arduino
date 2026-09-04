//stty -F /dev/ttyACM0 115200 raw -echo && cat /dev/ttyACM0

#include <Arduino.h>
#include "CANf303.hpp"

STM32CAN can;

volatile uint32_t receivedCount = 0;

twai_message_t receive;
void onCanReceive(twai_message_t msg){

    receivedCount++;

    //receive.identifier = msg.identifier;
    //receive.extd = msg.extd;
    //receive.rtr = msg.rtr;
    //receive.data_length_code = msg.data_length_code;
    //memcpy(receive.data, msg.data, msg.data_length_code);
    /*
    Serial.println("=== can RX ===");
    Serial.printf("ID   : 0x%lX\n", msg.identifier);
    Serial.printf("EXTD : %lu\n", msg.extd);
    Serial.printf("RTR  : %lu\n", msg.rtr);
    Serial.printf("DLC  : %u\n", msg.data_length_code);

    Serial.print("DATA : ");

    for (uint8_t i = 0; i < msg.data_length_code; i++) {
        Serial.printf("%02X ", msg.data[i]);
    }
    Serial.println();    
    */

    
}

void setup(){
    Serial.begin(115200);
    delay(1000);

    Serial.println("can loopback test");

    if (!can.begin(500000, PA12_PA11)) {
        Serial.println("CANの初期化失敗");
        while (1);
    }

    can.onReceive(onCanReceive);

    Serial.println("CANの初期化成功");
}

void loop() {

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
    Serial.printf("send result = %d\n", result);

    delay(1000);

    /*
    Serial.printf(
        "TSR=0x%08lX IER=0x%08lX MSR=0x%08lX\n",
        CAN1->TSR,
        CAN1->IER,
        CAN1->MSR
    );
    Serial.printf(
        "TMEIE=%d\n",
        (CAN1->IER & CAN_IER_TMEIE) != 0
    );*/
    //Serial.printf("txIrqCount = %lu\n", txIrqCount);
    //Serial.printf("received count: %d\n", receivedCount);
    //Serial.printf("can.available: \n", can.available());
    //Serial.print("received ID:");
    //Serial.println(receive.identifier);
}