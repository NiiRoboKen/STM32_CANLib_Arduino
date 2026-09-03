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

/*
//定数
constexpr uint8_t STM32_AF7 = 0x07;
constexpr uint8_t STM32_AF9 = 0x09;

constexpr uint8_t CAN_TX_QUEUE_SIZE = 16;
constexpr uint8_t CAN_RX_QUEUE_SIZE = 16;
*/

#if false
struct CAN_msg_t{
  uint32_t id;        /* 29 bit identifier                      */
  uint8_t  data[8];   /* Data field                             */
  uint8_t  len;       /* Length of data field in bytes          */
  uint8_t  format;    /* 0 - STANDARD, 1- EXTENDED IDENTIFIER   */
  uint8_t  type;      /* 0 - DATA FRAME, 1 - REMOTE FRAME       */
};
#endif

/*
struct twai_message_t{        //CAN_msg_tでは
    uint32_t extd;            //format
    uint32_t rtr;             //type
    uint32_t identifier;      //id
    uint8_t data_length_code; //len
    uint8_t data[8];          //data[8]
};
*/

//twai_msg_tの構造体をESP32の環境と同じように定義
/*struct twai_message_t{
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
};*/


class CanDriver{
  public:
    bool begin(long baudRate, CANPinTypes pins);

    bool send(uint16_t id, uint8_t data[8], uint8_t dlc);
    bool send(uint16_t id, uint8_t data[8]);

    void onReceive(void (*callback)(twai_message_t msg));
  
  private:
    void (*rxCallback)(twai_message_t msg) = nullptr;
};

//ラッパー
#if defined(STM32F3xx)
STM32CAN Can1;

void CanDriver::onReceive(void (*callback)(twai_message_t msg)){
  rxCallback = callback;

  //コールバックが登録されていたら登録
  if(rxCallback) Can1.onReceive(rxCallback);
}

bool CanDriver::begin(long baudRate, CANPinTypes pins){
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

#endif