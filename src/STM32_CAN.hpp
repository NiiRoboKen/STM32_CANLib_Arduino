#pragma once

#include <Arduino.h>

//f303とf446用のドライバ

//ピンの設定のenumは次のようになっています
//f303- PA12_PA11
//f446- PA12_PA11, PB13_PB12

#if defined(STM32F3xx)
  #include "CANf303.hpp"
#elif defined(STM32F4xx)
  #include "CANf446.hpp"
  //enum CANPinTypes {PA12_PA11, PB13_PB12}; //F446RE
#endif

//STM32_CAN.hpp

/*
残りのタスク

骨組みの構築

f303とf406のドライバをclassでラップする

*/

//ラッパークラス
class CanDriver{
  public:
    bool begin(long baudRate, CANPinTypes pins);

    bool send(uint16_t id, uint8_t data[8], uint8_t dlc);
    bool send(uint16_t id, uint8_t data[8]);

    void onReceive(void (*callback)(twai_message_t msg));
  
  private:
    void (*rxCallback)(twai_message_t msg) = nullptr;

    int WhichCanUsing(bool CAN1USE, bool CAN2USE){
      if(CAN1USE){
        CAN1USING = true;
      }else if(CAN2USE){
        CAN2USING = true;      
      }
    };
    
    bool CAN1USING = false;
    bool CAN2USING = false;
};


#if defined(STM32F3xx)
STM32CAN Can1;

void CanDriver::onReceive(void (*callback)(twai_message_t msg)){
  rxCallback = callback;

  //コールバックが登録されていたら登録
  if(rxCallback) Can1.onReceive(rxCallback);
}

bool CanDriver::begin(long baudRate, CANPinTypes pins){
  WhichCanUsing(true, false);
  //初期化
  return Can1.begin(baudRate, pins);
}

bool CanDriver::send(uint16_t id, uint8_t data[8], uint8_t dlc){
  twai_message_t msg;
  //msgにidとdataの中身、dlcをセットする
  msg.identifier = id;
  msg.extd = 0;
  msg.rtr = 0;
  msg.data_length_code = dlc;
  memcpy(msg.data, data, dlc);

  return Can1.send(msg);
}

#elif defined(STM32f4xx)

//446にはCAN1とCAN2があるため、そこをしっかり状態管理すること

//先に作られたインスタンスをCAN1として扱う
#ifndef CAN1USE
#define CAN1USE
STM32CAN Can1;
#endif
#ifndef CAN2USE
#define CAN2USE
STM32CAN Can2;
#endif

void CanDriver::onReceive(void (*callback)(twai_message_t msg)){
  rxCallback = callback;

  //コールバックが登録されていたら登録
  #ifdef CAN1USE
  if(rxCallback) Can1.onReceive(rxCallback);
  #endif
  #ifdef CAN2USE
  if(rxCallback) Can2.onReceive(rxCallback);
  #endif
}

bool CanDriver::begin(long baudRate, CANPinTypes pins){
  //初期化
  
  #ifdef CAN1USE
  WhichCanUsing(true, false);
  return Can1.begin(baudRate, pins, false);
  #endif
  #ifdef CAN2USE
  WhichCanUsing(false, true);
  return Can2.begin(baudRate, pins, true);
  #endif
}

bool CanDriver::send(uint16_t id, uint8_t data[8], uint8_t dlc){
  twai_message_t msg;
  //msgにidとdataの中身、dlcをセットする
  msg.identifier = id;
  msg.extd = 0;
  msg.rtr = 0;
  msg.data_length_code = dlc;
  memcpy(msg.data, data, dlc);

  #ifdef CAN1USE
  return Can1.send(msg);
  #endif
  #ifdef CAN2USE
  return Can2.send(msg);
  #endif
}

#endif