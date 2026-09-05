#pragma once 

#include <STM32FreeRTOS.h>
#include <Arduino.h>

//定数
constexpr uint8_t STM32_AF7 = 0x07;
constexpr uint8_t STM32_AF9 = 0x09;

constexpr uint8_t CAN_TX_QUEUE_SIZE = 14;
constexpr uint8_t CAN_RX_QUEUE_SIZE = 14;

/* CANメッセージのフォーマットを表す記号名 */
enum CAN_FORMAT {STANDARD_FORMAT = 0, EXTENDED_FORMAT};

/* CANメッセージの種類を表す記号名 */
enum CAN_FRAME {DATA_FRAME = 0, REMOTE_FRAME};

enum CANPinTypes {PA12_PA11, PB13_PB12};//446はこの二つ

struct twai_message_t{        //CAN_msg_tでは
  uint32_t extd;            //format
  uint32_t rtr;             //type
  uint32_t identifier;      //id
  uint8_t data_length_code; //len
  uint8_t data[8];          //data[8]
};

struct CAN_bit_timing_config_t{
  uint8_t TS2;
  uint8_t TS1;
  uint8_t BRP;
};


class STM32CAN{
  public:
    static STM32CAN* instance;
    STM32CAN(){
      instance = this;
    }

    bool begin(long bitrate, CANPinTypes SelectPin){
      return CANInit(bitrate, SelectPin);
    }

    bool send(const twai_message_t& msg){
    }

    bool receive(twai_message_t* msg){
      return true;
    }

    uint8_t available(){
      return 0;
    }

    //コールバック
    void onReceive(void (*callback)(twai_message_t msg)){
      rxCallback = callback;
    }

    void processTxQueue();

    void CANReceiveHardware(twai_message_t* msg);

    void handleRxInterrupt(){
      
    }

  private:
    //コールバック関数のポインタ
    void (*rxCallback)(twai_message_t msg) = nullptr;

    //内部関数を追加
    bool CANSendToFreeMailbox(twai_message_t* msg);


    void CANSetGpio(
      GPIO_TypeDef* addr,
      uint8_t index,
      uint8_t afry,
      uint8_t speed
    );

    void CANSetFilter(
      uint8_t index,
      uint8_t scale,
      uint8_t mode,
      uint8_t fifo,
      uint32_t bank1,
      uint32_t bank2
    );

    CAN_bit_timing_config_t ConvBaudrate(long baud);

    bool CANInit(long bitrate, CANPinTypes selectPin);

    static void rxTask(void* param);

    //RingBuffer<twai_message_t, CAN_TX_QUEUE_SIZE> txQueue;
    //RingBuffer<twai_message_t, CAN_RX_QUEUE_SIZE> rxQueue;
    volatile bool txBusy = false;
};