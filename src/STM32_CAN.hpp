#pragma once

#include <Arduino.h>

//STM32_CAN.hpp

/*
残りのタスク

骨組みの構築

関数内の処理の実装
レジスタの操作
*/

//定数

constexpr uint8_t STM32_AF7 = 0x07;
constexpr uint8_t STM32_AF9 = 0x09;

constexpr uint8_t CAN_TX_QUEUE_SIZE = 16;
constexpr uint8_t CAN_RX_QUEUE_SIZE = 16;

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


struct CAN_bit_timing_config_t{
  uint8_t TS2;
  uint8_t TS1;
  uint8_t BRP;
};

//ピン選択用のenum(それぞれtx,rxの並びになっています)
#if defined(STM32F3xx)
enum CANPinTypes {PA12_PA11}; //F303K8
#elif defined(STM32F4xx)
enum CANPinTypes {PA12_PA11, PB13_PB12}; //F446RE
#endif


class CanDriver{
  public:
    bool begin(long baudRate, CANPinTypes pins);

    bool send(uint16_t id, uint8_t data[8], uint8_t dlc);
    bool send(uint16_t id, uint8_t data[8]);

    void onReceive(void (*callback)(twai_message_t msg));
  
  private:
    void (*rxCallback)(twai_message_t msg) = nullptr;

    CAN_bit_timing_config_t convBaud(long baud);

    void CANSetGpio(GPIO_TypeDef * addr, uint8_t index, uint8_t afry, uint8_t speed = 3);
    void setupCANpins(CANPinTypes pins);
    void CANSetFilter(uint8_t index, uint8_t scale, uint8_t mode, uint8_t fifo, uint32_t bank1, uint32_t bank2);

    bool sendDataToFIFO(twai_message_t msgData);
    
};


//public

bool CanDriver::begin(long baudRate, CANPinTypes pins){
  // CANクロックの有効化
  RCC->APB1ENR |= 0x2000000UL;

  bool isCAN1 = false;
  //ピンの設定
  #if defined(STM32F3xx)
  //STM32F303K8はCANに使用可能なピンがPA12_PA11のみのためそれ以外を除外
  if(pins == PA12_PA11){
    RCC->AHBENR |= 0x20000UL;           // GPIOAクロックの有効化
    //各ピンの設定
    CANSetGpio(GPIOA, 12, STM32_AF9);         // STM32_AF9にPA12を設定
    CANSetGpio(GPIOA, 11, STM32_AF9);         // STM32_AF9にPA11を設定
  }else{
    return false;
  }
  #elif defined(STM32F4xx)
  //STM32F446REはCANに使用可能なピンがPA12_PA11, PB13_PB12の二つであるため、それ以外を除外
  if(pins == PA12_PA11){
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // GPIOAクロックの有効化
    //各ピンの設定
    CANSetGpio(GPIOA, 12, STM32_AF9);         // STM32_AF9にPA12を設定
    CANSetGpio(GPIOA, 11, STM32_AF9);         // STM32_AF9にPA11を設定
  }else if(pins == PB13_PB12){
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; // GPIOBクロックを有効化
    //各ピンの設定
    CANSetGpio(GPIOB, 13, STM32_AF9);         // STM32_AF9にPA13を設定
    CANSetGpio(GPIOB, 12, STM32_AF9);         // STM32_AF9にPA12を設定
    isCAN1 = true;
  }else {
    return false;
  }
  #endif


  //長くなったので後でinline関数に置き換えること
  if(isCAN1){
    //CAN1
    CAN1->MCR |= 0x1UL;                   // CANを初期化状態にする
    while (!(CAN1->MSR & 0x1UL));         // 初期化状態になるのを待つ

    //CAN1->MCR = 0x51UL;                 // ハードウェアの初期化(自動的に再送信しない)
    //CAN1->MCR = 0x41UL;                   // ハードウェアの初期化(自動的に再送信する)
    CAN1->MCR = 0;
    CAN1->MCR |= CAN_MCR_ABOM;

    // ビットレートを設定 
    CAN_bit_timing_config_t configData = convBaud(baudRate);

    //IRNQを1にして書き込み可能にする
    CAN1->MCR |= CAN_MCR_INRQ;
  
    CAN1->BTR &= ~(((0x03) << 24) | ((0x07) << 20) | ((0x0F) << 16) | (0x3FF)); 
    CAN1->BTR |= (((configData.TS2-1) & 0x07) << 20) | (((configData.TS1-1) & 0x0F) << 16) | ((configData.BRP-1) & 0x3FF);

    #ifdef DEBUG
    //loopback
    //CAN1->BTR |= CAN_BTR_LBKM;
    #endif

    //書き込みを終了する
    CAN1->MCR &= ~CAN_MCR_INRQ;

    // フィルターをデフォルトの値に設定
    CAN1->FMR |=   0x1UL; // フィルターを初期化状態にする

    // Set fileter 0
    // Single 32-bit scale configuration 
    // Two 32-bit registers of filter bank x are in Identifier Mask mode
    // Filter assigned to FIFO 0 
    // Filter bank register to all 0
    CANSetFilter(0, 1, 0, 0, 0x0UL, 0x0UL);

    CAN1->FMR &= ~(0x1UL);                // Deactivate initialization mode

    bool can1 = false;
    CAN1->MCR &= ~(0x1UL);                // Require CAN1 to normal mode 

    //割り込み有効化

    //Time inperruptの有効化
    CAN1->IER |= CAN_IER_TMEIE;

    // RX FIFO0 message pending interrupt
    CAN1->IER |= CAN_IER_FMPIE0;

    #if defined(STM32F3xx)
    // TxのNVIC有効化
    NVIC_EnableIRQ(USB_HP_CAN_TX_IRQn);    
    // RxのNVIC有効化
    NVIC_EnableIRQ(USB_LP_CAN_RX0_IRQn);
    #elif defined(STM32F4xx)
    // TxのNVIC有効化
    NVIC_EnableIRQ(CAN1_TX_IRQn);
    // RxのNVIC有効化
    NVIC_EnableIRQ(CAN1_RX0_IRQn);
    #endif
    
    // Wait for normal mode
    // If the connection is not correct, it will not return to normal mode.
    uint16_t TimeoutMilliseconds = 1000;
    uint16_t wait_ack = 0;
    while(wait_ack < TimeoutMilliseconds){
      wait_ack++;
      if ((CAN1->MSR & 0x1UL) == 0) {
        can1 = true;
        break;
      }
      delay(1);
    }
    
    return !!can1;
  }else{
    //CAN2
    CAN2->MCR |= 0x1UL;                   // CANを初期化状態にする
    while (!(CAN2->MSR & 0x1UL));         // 初期化状態になるのを待つ

    //CAN2->MCR = 0x51UL;                 // ハードウェアの初期化(自動的に再送信しない)
    //CAN2->MCR = 0x41UL;                   // ハードウェアの初期化(自動的に再送信する)
    CAN2->MCR = 0;
    CAN2->MCR |= CAN_MCR_ABOM;

    // ビットレートを設定 
    CAN_bit_timing_config_t configData = convBaud(baudRate);

    //IRNQを1にして書き込み可能にする
    CAN2->MCR |= CAN_MCR_INRQ;
  
    CAN2->BTR &= ~(((0x03) << 24) | ((0x07) << 20) | ((0x0F) << 16) | (0x3FF)); 
    CAN2->BTR |= (((configData.TS2-1) & 0x07) << 20) | (((configData.TS1-1) & 0x0F) << 16) | ((configData.BRP-1) & 0x3FF);

    #ifdef DEBUG
    //loopback
    //CAN1->BTR |= CAN_BTR_LBKM;
    #endif

    //書き込みを終了する
    CAN2->MCR &= ~CAN_MCR_INRQ;


    // フィルターをデフォルトの値に設定
    CAN2->FMR |=   0x1UL; // フィルターを初期化状態にする

    // Set fileter 0
    // Single 32-bit scale configuration 
    // Two 32-bit registers of filter bank x are in Identifier Mask mode
    // Filter assigned to FIFO 0 
    // Filter bank register to all 0
    CANSetFilter(0, 1, 0, 0, 0x0UL, 0x0UL);

    CAN2->FMR &= ~(0x1UL);                // Deactivate initialization mode

    bool can2 = false;
    CAN2->MCR &= ~(0x1UL);                // Require CAN1 to normal mode 

    //割り込み有効化

    //Time inperruptの有効化
    CAN2->IER |= CAN_IER_TMEIE;

    // RX FIFO0 message pending interrupt
    CAN2->IER |= CAN_IER_FMPIE0;

    // TxのNVIC有効化
    NVIC_EnableIRQ(CAN2_TX_IRQn);
    // RxのNVIC有効化
    NVIC_EnableIRQ(CAN2_RX0_IRQn);
    
    // Wait for normal mode
    // If the connection is not correct, it will not return to normal mode.
    uint16_t TimeoutMilliseconds = 1000;
    uint16_t wait_ack = 0;
    while(wait_ack < TimeoutMilliseconds){
      wait_ack++;
      if ((CAN2->MSR & 0x1UL) == 0) {
        can2 = true;
        break;
      }
      delay(1);
    }
    
    return !!can2;
  }

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
CAN_bit_timing_config_t CanDriver::convBaud(long baud){
  
}


inline void CanDriver::CANSetGpio(GPIO_TypeDef * addr, uint8_t index, uint8_t afry, uint8_t speed = 3) {
    uint8_t _index2 = index * 2;
    uint8_t _index4 = index * 4;
    uint8_t ofs = 0;
    uint8_t setting;

    if (index > 7) {
      _index4 = (index - 8) * 4;
      ofs = 1;
    }

    uint32_t mask;
    mask = 0xF << _index4;
    addr->AFR[ofs]  &= ~mask;         // Reset alternate function
    //setting = 0x9;                    // STM32_AF9
    setting = afry;                   // Alternative function selection
    mask = setting << _index4;
    addr->AFR[ofs]  |= mask;          // Set alternate function
    
    mask = 0x3 << _index2;
    addr->MODER   &= ~mask;           // Reset mode
    setting = 0x2;                    // Alternate function mode
    mask = setting << _index2;
    addr->MODER   |= mask;            // Set mode
    
    mask = 0x3 << _index2;
    addr->OSPEEDR &= ~mask;           // Reset speed
    setting = speed;
    mask = setting << _index2;
    addr->OSPEEDR |= mask;            // Set speed
    
    mask = 0x1 << index;
    addr->OTYPER  &= ~mask;           // Reset Output push-pull
    
    mask = 0x3 << _index2;
    addr->PUPDR   &= ~mask;           // Reset port pull-up/pull-down
    
}


/**
 * Initializes the CAN filter registers.
 *
 * The bxCAN provides up to 14 scalable/configurable identifier filter banks, for selecting the incoming messages, that the software needs and discarding the others.
 *
 * @preconditions   - This register can be written only when the filter initialization mode is set (FINIT=1) in the CAN_FMR register.
 * @params: index   - Specified filter index. index 27:14 are available in connectivity line devices only.
 * @params: scale   - Select filter scale.
 *                    0: Dual 16-bit scale configuration
 *                    1: Single 32-bit scale configuration
 * @params: mode    - Select filter mode.
 *                    0: Two 32-bit registers of filter bank x are in Identifier Mask mode
 *                    1: Two 32-bit registers of filter bank x are in Identifier List mode
 * @params: fifo    - Select filter assigned.
 *                    0: Filter assigned to FIFO 0
 *                    1: Filter assigned to FIFO 1
 * @params: bank1   - Filter bank register 1
 * @params: bank2   - Filter bank register 2
 *
 */
inline void CanDriver::CANSetFilter(uint8_t index, uint8_t scale, uint8_t mode, uint8_t fifo, uint32_t bank1, uint32_t bank2){
  if (index > 13) return;

  CAN1->FA1R &= ~(0x1UL<<index);               // Deactivate filter

  if (scale == 0) {
    CAN1->FS1R &= ~(0x1UL<<index);             // Set filter to Dual 16-bit scale configuration
  } else {
    CAN1->FS1R |= (0x1UL<<index);              // Set filter to single 32 bit configuration
  }
  if (mode == 0) {
    CAN1->FM1R &= ~(0x1UL<<index);             // Set filter to Mask mode
  } else {
    CAN1->FM1R |= (0x1UL<<index);              // Set filter to List mode
  }

  if (fifo == 0) {
    CAN1->FFA1R &= ~(0x1UL<<index);            // Set filter assigned to FIFO 0
  } else {
    CAN1->FFA1R |= (0x1UL<<index);             // Set filter assigned to FIFO 1
  }

  CAN1->sFilterRegister[index].FR1 = bank1;    // Set filter bank registers1
  CAN1->sFilterRegister[index].FR2 = bank2;    // Set filter bank registers2

  CAN1->FA1R |= (0x1UL<<index);                // Activate filter

}


bool CanDriver::sendDataToFIFO(twai_message_t msgData){
  return true;
}
