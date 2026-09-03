
/*
参考プログラム
https://github.com/nopnop2002/Arduino-STM32-CAN/blob/master/stm32f303/stm32f303.ino
をclassにまとめました。

多少いじくりました（引数の抽象化、非同期送信、class化）
元のコメントは一部翻訳済み

CANメッセージのビットレートに指定できる数値の一覧
50000
100000
125000
250000
500000
1000000

CANのTx,Rxに使用するピンを設定します。
上のほうにあるenum CANPinTypesによって、このようなピンを設定できます。
PA11_PA12,
PB8_PB9
*/

#pragma once

#include <STM32FreeRTOS.h>
#include <Arduino.h>

#define DEBUG

//定数
constexpr uint8_t STM32_AF7 = 0x07;
constexpr uint8_t STM32_AF9 = 0x09;

constexpr uint8_t CAN_TX_QUEUE_SIZE = 16;
constexpr uint8_t CAN_RX_QUEUE_SIZE = 16;

/* CANメッセージのフォーマットを表す記号名 */
enum CAN_FORMAT {STANDARD_FORMAT = 0, EXTENDED_FORMAT};

/* CANメッセージの種類を表す記号名 */
enum CAN_FRAME {DATA_FRAME = 0, REMOTE_FRAME};

enum CANPinTypes {PA12_PA11};

struct twai_message_t{        //CAN_msg_tでは
    uint32_t extd;            //format
    uint32_t rtr;             //type
    uint32_t identifier;      //id
    uint8_t data_length_code; //len
    uint8_t data[8];          //data[8]
};

#if false
struct CAN_msg_t{
  uint32_t id;        /* 29 bit identifier                      */
  uint8_t  data[8];   /* Data field                             */
  uint8_t  len;       /* Length of data field in bytes          */
  uint8_t  format;    /* 0 - STANDARD, 1- EXTENDED IDENTIFIER   */
  uint8_t  type;      /* 0 - DATA FRAME, 1 - REMOTE FRAME       */
};
#endif

struct CAN_bit_timing_config_t{
  uint8_t TS2;
  uint8_t TS1;
  uint8_t BRP;
};


//汎用リングバッファー化用
template<typename T, uint8_t SIZE>
class RingBuffer {
  private:
    T buffer[SIZE];
    volatile uint8_t head = 0;
    volatile uint8_t tail = 0;
    volatile uint8_t count = 0;

  public:
    bool enqueue(const T& item){
      noInterrupts();

      if(count >= SIZE){
          interrupts();
          return false;
      }

      buffer[head] = item;
      head = (head + 1) % SIZE;
      count++;

      interrupts();
      return true;
    }

    bool dequeue(T* item){
      noInterrupts();

      if(count == 0){
          interrupts();
          return false;
      }

      *item = buffer[tail];
      tail = (tail + 1) % SIZE;
      count--;

      interrupts();
      return true;
    }

    uint8_t available(){
      noInterrupts();
      uint8_t c = count;
      interrupts();
      return c;
    }

    void clear(){
      noInterrupts();
      head = tail = count = 0;
      interrupts();
    }
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
      bool ok = txQueue.enqueue(msg);
      if(ok) processTxQueue();
      return ok;
    }

    bool receive(twai_message_t* msg){
      return rxQueue.dequeue(msg);
    }

    uint8_t available(){
      return rxQueue.available();
    }

    //コールバック
    void onReceive(void (*callback)(twai_message_t msg)){
      rxCallback = callback;
    }

    void processTxQueue();

    void CANReceiveHardware(twai_message_t* msg);

    void handleRxInterrupt(){
      while (CAN1->RF0R & 0x3UL){
        twai_message_t msg;
        CANReceiveHardware(&msg);
        if(!rxQueue.enqueue(msg)){
          // overflow
        }
      }
    }

    void rxTask(void* param){
      STM32CAN* self = static_cast<STM32CAN*>(param);
      twai_message_t msg;

      while(true){
        //受信結果にメッセージが存在するかを調べる
        if(receive(&msg)){
          if (self->rxCallback) {
            self->rxCallback(msg);
          }
        }
      }
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

  private:
    RingBuffer<twai_message_t, CAN_TX_QUEUE_SIZE> txQueue;
    RingBuffer<twai_message_t, CAN_RX_QUEUE_SIZE> rxQueue;
    volatile bool txBusy = false;

};

inline STM32CAN* STM32CAN::instance = nullptr;



//キューにあるものをMailboxへ送信
inline void STM32CAN::processTxQueue(){
    noInterrupts();

    if (txBusy) {
        interrupts();
        return;
    }

    txBusy = true;
    interrupts();
    twai_message_t msg;
    while (true){
      // 空きMailbox無し→終了
      if (!(CAN1->TSR & (CAN_TSR_TME0 | CAN_TSR_TME1 | CAN_TSR_TME2))) break;
      // queueが空→終了
      if (!txQueue.dequeue(&msg)) break;

      CANSendToFreeMailbox(&msg);
    }

    noInterrupts();
    txBusy = false;
    interrupts();
}


/**
 * Initializes the CAN GPIO registers.
 *
 * @params: addr    - Specified GPIO register address.
 * @params: index   - Specified GPIO index.
 * @params: afry    - Specified Alternative function selection AF0-AF15.
 * @params: speed   - Specified OSPEEDR register value.(Optional)
 *
 */
inline void STM32CAN::CANSetGpio(GPIO_TypeDef * addr, uint8_t index, uint8_t afry, uint8_t speed = 3) {
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
inline void STM32CAN::CANSetFilter(uint8_t index, uint8_t scale, uint8_t mode, uint8_t fifo, uint32_t bank1, uint32_t bank2) {
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

/*
struct CAN_bit_timing_config_t{
  uint8_t TS2;
  uint8_t TS1;
  uint8_t BRP;
};
*/

inline CAN_bit_timing_config_t STM32CAN::ConvBaudrate(long baud){
  switch(baud){
    case (long)50E3:
      return {2, 13, 45};
    case (long)100E3:
      return {2, 15, 20};
    case (long)125E3:
      return {2, 13, 18};
    case (long)250E3:
      return {2, 13, 9};
    case (long)500E3:
      return {2, 15, 4};
    case (long)1000E3:
      return {2, 13, 2};//return {2, 15, 2};
    default:
      return {2, 13, 45};
  }
}
    
/**
 * CANコントローラーの初期化及びピン設定
 *
 * @params: bitrate
 *   ビットレートに指定できる数値の一覧
 *   50000
 *   100000
 *   125000
 *   250000
 *   500000
 *   1000000
 * @params: SelectPin
 *   CANのTx,Rxに使用するピンを設定します。
 *   上のほうにあるenum CANPinTypesによって、このようなピンを設定できます。
 *   PA11_PA12,
 *   PB8_PB9
 * 
 */
inline bool STM32CAN::CANInit(long bitrate, CANPinTypes SelectPin){
  // リファレンスマニュアル
  // https://www.st.com/content/ccc/resource/technical/document/reference_manual/4a/19/6e/18/9d/92/43/32/DM00043574.pdf/files/DM00043574.pdf/jcr:content/translations/en.DM00043574.pdf

  //RCC->APB1ENR |= 0x2000000UL;          // CANクロックの有効化
  RCC->APB1ENR |= RCC_APB1ENR_CANEN;

  switch(SelectPin){
    case PA12_PA11:
      RCC->AHBENR |= 0x20000UL;           // GPIOAクロックの有効化
      CANSetGpio(GPIOA, 11, STM32_AF9);         // STM32_AF9にPA11を設定
      CANSetGpio(GPIOA, 12, STM32_AF9);         // STM32_AF9にPA12を設定
      break;
    default:
      return false;
      //例外値は無視
  }

  CAN1->MCR |= 0x1UL;                   // CANを初期化状態にする
  while (!(CAN1->MSR & 0x1UL));         // 初期化状態になるのを待つ
  //CAN1->MCR = 0x51UL;                   // ハードウェアの初期化(自動的に再送信しない)
  //CAN1->MCR = 0x41UL;                   // ハードウェアの初期化(自動的に再送信する)
  CAN1->MCR = 0;
  CAN1->MCR |= CAN_MCR_ABOM;

  //書き込み可能にする
  CAN1->MCR |= CAN_MCR_INRQ;

  // ビットレートを設定 
  CAN_bit_timing_config_t configData = ConvBaudrate(bitrate);

  //Serial.printf("TS2: %d\n",configData.TS2);
  //Serial.printf("TS1: %d\n",configData.TS1);
  //Serial.printf("BRP: %d\n",configData.BRP);
  
  CAN1->BTR &= ~(((0x03) << 24) | ((0x07) << 20) | ((0x0F) << 16) | (0x3FF)); 
  CAN1->BTR |= (((configData.TS2-1) & 0x07) << 20) | (((configData.TS1-1) & 0x0F) << 16) | ((configData.BRP-1) & 0x3FF);

  #if defined(DEBUG)
  Serial.println("ループバックを有効化します");
  CAN1->BTR |= CAN_BTR_LBKM;
  /*
  uint32_t btr = CAN1->BTR;

  uint32_t brp = (btr & 0x3FF) + 1;
  uint32_t ts1 = ((btr >> 16) & 0x0F) + 1;
  uint32_t ts2 = ((btr >> 20) & 0x07) + 1;
  uint32_t sjw = ((btr >> 24) & 0x03) + 1;
  
  Serial.printf("BRP = %lu\n", brp);
  Serial.printf("TS1 = %lu\n", ts1);
  Serial.printf("TS2 = %lu\n", ts2);
  Serial.printf("SJW = %lu\n", sjw);
  */
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

  // TxのNVIC有効化
  NVIC_EnableIRQ(USB_HP_CAN_TX_IRQn);

  // RxのNVIC有効化
  NVIC_EnableIRQ(USB_LP_CAN_RX0_IRQn);

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

  //受信タスクの追加
  xTaskCreate(rxTask, "CAN_RX_Task", 4096, this, 2, NULL);
  
  return !!can1;
}


#define STM32_CAN_TIR_TXRQ  (1U << 0U)  // Bit 0: Transmit Mailbox Request
#define STM32_CAN_RIR_RTR   (1U << 1U)  // Bit 1: Remote Transmission Request
#define STM32_CAN_RIR_IDE   (1U << 2U)  // Bit 2: Identifier Extension
#define STM32_CAN_TIR_RTR   (1U << 1U)  // Bit 1: Remote Transmission Request
#define STM32_CAN_TIR_IDE   (1U << 2U)  // Bit 2: Identifier Extension

#define CAN_EXT_ID_MASK     0x1FFFFFFFU
#define CAN_STD_ID_MASK     0x000007FFU
 
/**
 * ハードウェアFIFOから読み出す関数
 * 
 * @preconditions     - A valid CAN message is received
 * @params CAN_rx_msg - CAN message structure for reception
 * 
 */
inline void STM32CAN::CANReceiveHardware(twai_message_t* CAN_rx_msg){
  uint32_t id = CAN1->sFIFOMailBox[0].RIR;
  if ((id & STM32_CAN_RIR_IDE) == 0) { // Standard frame format
      CAN_rx_msg->extd = STANDARD_FORMAT;;
      CAN_rx_msg->identifier = (CAN_STD_ID_MASK & (id >> 21U));
  } 
  else {                               // Extended frame format
      CAN_rx_msg->extd = EXTENDED_FORMAT;;
      CAN_rx_msg->identifier = (CAN_EXT_ID_MASK & (id >> 3U));
  }

  if ((id & STM32_CAN_RIR_RTR) == 0) { // Data frame
      CAN_rx_msg->rtr = DATA_FRAME;
  }
  else {                               // Remote frame
      CAN_rx_msg->rtr = REMOTE_FRAME;
  }

  
  CAN_rx_msg->data_length_code = (CAN1->sFIFOMailBox[0].RDTR) & 0xFUL;
  
  CAN_rx_msg->data[0] = 0xFFUL &  CAN1->sFIFOMailBox[0].RDLR;
  CAN_rx_msg->data[1] = 0xFFUL & (CAN1->sFIFOMailBox[0].RDLR >> 8);
  CAN_rx_msg->data[2] = 0xFFUL & (CAN1->sFIFOMailBox[0].RDLR >> 16);
  CAN_rx_msg->data[3] = 0xFFUL & (CAN1->sFIFOMailBox[0].RDLR >> 24);
  CAN_rx_msg->data[4] = 0xFFUL &  CAN1->sFIFOMailBox[0].RDHR;
  CAN_rx_msg->data[5] = 0xFFUL & (CAN1->sFIFOMailBox[0].RDHR >> 8);
  CAN_rx_msg->data[6] = 0xFFUL & (CAN1->sFIFOMailBox[0].RDHR >> 16);
  CAN_rx_msg->data[7] = 0xFFUL & (CAN1->sFIFOMailBox[0].RDHR >> 24);

  // Release FIFO 0 output mailbox.
  // Make the next incoming message available.
  CAN1->RF0R |= 0x20UL;
}

//空きMainboxにデータを送る
inline bool STM32CAN::CANSendToFreeMailbox(twai_message_t* CAN_tx_msg){
    uint8_t mailbox;

    // 空きMailbox探索
    if (CAN1->TSR & CAN_TSR_TME0) {
        mailbox = 0;
    }
    else if (CAN1->TSR & CAN_TSR_TME1) {
        mailbox = 1;
    }
    else if (CAN1->TSR & CAN_TSR_TME2) {
        mailbox = 2;
    }
    else {
        return false;
    }

    uint32_t out = 0;

    // ID設定
    if (CAN_tx_msg->extd == EXTENDED_FORMAT) {
        out = ((CAN_tx_msg->identifier & CAN_EXT_ID_MASK) << 3U)
            | STM32_CAN_TIR_IDE;
    }
    else {
        out = ((CAN_tx_msg->identifier & CAN_STD_ID_MASK) << 21U);
    }

    // RTR
    if (CAN_tx_msg->rtr == REMOTE_FRAME) {
        out |= STM32_CAN_TIR_RTR;
    }

    // DLC
    CAN1->sTxMailBox[mailbox].TDTR =
        (CAN_tx_msg->data_length_code & 0xFUL);

    // DATA LOW
    CAN1->sTxMailBox[mailbox].TDLR =
        (((uint32_t)CAN_tx_msg->data[3] << 24) |
         ((uint32_t)CAN_tx_msg->data[2] << 16) |
         ((uint32_t)CAN_tx_msg->data[1] << 8 ) |
         ((uint32_t)CAN_tx_msg->data[0]));

    // DATA HIGH
    CAN1->sTxMailBox[mailbox].TDHR =
        (((uint32_t)CAN_tx_msg->data[7] << 24) |
         ((uint32_t)CAN_tx_msg->data[6] << 16) |
         ((uint32_t)CAN_tx_msg->data[5] << 8 ) |
         ((uint32_t)CAN_tx_msg->data[4]));

    // 送信開始
    CAN1->sTxMailBox[mailbox].TIR = out | STM32_CAN_TIR_TXRQ;

    return true;
}


// TxのISR定義
extern "C" void USB_HP_CAN_TX_IRQHandler(void){
  // Mailbox0
  if (CAN1->TSR & CAN_TSR_RQCP0) CAN1->TSR |= CAN_TSR_RQCP0;
  // Mailbox1
  if (CAN1->TSR & CAN_TSR_RQCP1) CAN1->TSR |= CAN_TSR_RQCP1;
  // Mailbox2
  if (CAN1->TSR & CAN_TSR_RQCP2) CAN1->TSR |= CAN_TSR_RQCP2;

  // 次のキューを送信
  if(STM32CAN::instance){
    STM32CAN::instance->processTxQueue();
  }
  //processTxQueue();
}

// RxのISR定義
extern "C" void USB_LP_CAN_RX0_IRQHandler(void){
    if(STM32CAN::instance){
        STM32CAN::instance->handleRxInterrupt();
    }
}
