#pragma once

#include <Arduino.h>

//STM32_CAN.hpp

/*
残りのタスク

骨組みの構築


*/

//twai_msg_tの構造体をESP32の場合と同じように定義

struct twai_msg_t{
  uint32_t extd = 1;
  uint32_t rtr = 1;
  uint32_t ss = 1;
  uint32_t self = 1;
  uint32_t dlc_non_comp = 1;
  uint32_t reserved = 27;
  uint32_t flags;
  uint32_t identifier;

  uint8_t  data_length_code;
  uint8_t  data[8];
};


namespace stm32_can{

class CanDriver{
  public:
    bool begin(long baudRate, uint8_t tx, uint8_t rx);

    bool send(uint16_t id, uint8_t data[8], uint8_t dlc);
    bool send(uint16_t id, uint8_t data[8]);

    void onReceive(void (*callback)(twai_msg_t msg));
  
  private:
    void convBaud(long baud);
    void setupCAN(uint8_t tx, uint8_t rx);
    bool sendDataToFIFO(twai_msg_t msgData);
    
};

//public

bool CanDriver::begin(long baudRate, uint8_t tx, uint8_t rx){
  
}

bool CanDriver::send(uint16_t id, uint8_t data[8], uint8_t dlc){
  
}
//dlc無し版
bool CanDriver::send(uint16_t id, uint8_t data[8]){
  
}

void CanDriver::onReceive(void (*callback)(twai_msg_t msg)){

}

//private

//通信速度ごとに適切なクロック設定に変換するデータ
void CanDriver::convBaud(long baud){
  
}


}

