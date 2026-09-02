#pragma once

#include <Arduino.h>

//STM32_CAN.hpp

/*
残りのタスク

骨組みの構築

関数内の処理の実装
レジスタの操作
*/

//twai_msg_tの構造体をESP32の環境と同じように定義

struct twai_message_t{
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

//ピン選択用のenum(それぞれtx,rxの並びになっています)
#if defined(STM32F3xx)
enum CANPinTypes {PA12_PA11}; //F303K8
#elif defined(STM32F4xx)
enum CANPinTypes {PA12_PA11, PB13_PB12}; //F446RE
#endif

namespace stm32_can{

class CanDriver{
  public:
    bool begin(long baudRate, CANPinTypes pins);

    bool send(uint16_t id, uint8_t data[8], uint8_t dlc);
    bool send(uint16_t id, uint8_t data[8]);

    void onReceive(void (*callback)(twai_message_t msg));
  
  private:
    void (*rxCallback)(twai_message_t msg) = nullptr;

    void convBaud(long baud);
    void setupCANpins(CANPinTypes pins);
    bool sendDataToFIFO(twai_message_t msgData);
    
};


//public

bool CanDriver::begin(long baudRate, CANPinTypes pins){
  // CANクロックの有効化
  RCC->APB1ENR |= 0x2000000UL;

  //ピンの設定
  #if defined(STM32F3xx)
  //STM32F303K8はCANに使用可能なピンがPA12_PA11のみのためそれ以外を除外
  if(pins == PA12_PA11){
    RCC->AHBENR |= 0x20000UL;           // GPIOAクロックの有効化
    //各ピンの設定
    setupCANpins(pins);
  }else{
    return false;
  }
  #elif defined(STM32F4xx)
  //STM32F446REはCANに使用可能なピンがPA12_PA11, PB13_PB12の二つであるため、それ以外を除外
  if(pins == PA12_PA11){
    RCC->AHBENR |= 0x20000UL;           // GPIOAクロックの有効化
    //各ピンの設定
    setupCANpins(pins);
  }else if(pins = PB13_PB12){
    RCC->AHBENR |= 0x40000UL;           // GPIOBクロックを有効化
    //各ピンの設定
    setupCANpins(pins);
  }else {
    return false;
  }
  #endif


  return true;
}


bool CanDriver::send(uint16_t id, uint8_t data[8], uint8_t dlc){
  return true;
}

//dlc無し版(データ帳を自動検出)
bool CanDriver::send(uint16_t id, uint8_t data[8]){
  uint8_t dlc = sizeof(data)/sizeof(data[0]);

  return true;
}


void CanDriver::onReceive(void (*callback)(twai_message_t msg)){
  rxCallback = callback;
}

/*
void CanDriver::rxTask(void* param) {
    CanDriver* self = static_cast<CanDriver*>(param);
    twai_message_t msg;

    while (true) {
        if (twai_receive(&msg, portMAX_DELAY) == ESP_OK) {
            if (self->rxCallback) {
                self->rxCallback(msg);
            }
        }
    }
}
*/


//private

//通信速度ごとに適切なクロック設定に変換するデータ
void CanDriver::convBaud(long baud){
  
}

void CanDriver::setupCANpins(CANPinTypes pins){
  
}

bool CanDriver::sendDataToFIFO(twai_message_t msgData){
  return true;
}

}

