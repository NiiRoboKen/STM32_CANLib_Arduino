#pragma once

#include <STM32FreeRTOS.h>
#include <Arduino.h>

#if defined(STM32F3xx)

//ループバックの有効化
//#define LOOPBACK

//定数
constexpr uint8_t STM32_AF9 = 0x09;

constexpr uint8_t CAN_TX_QUEUE_SIZE = 16;
constexpr uint8_t CAN_RX_QUEUE_SIZE = 16;

#define STM32_CAN_TIR_TXRQ  (1U << 0U)  // Bit 0: Transmit Mailbox Request
#define STM32_CAN_RIR_RTR   (1U << 1U)  // Bit 1: Remote Transmission Request
#define STM32_CAN_RIR_IDE   (1U << 2U)  // Bit 2: Identifier Extension
#define STM32_CAN_TIR_RTR   (1U << 1U)  // Bit 1: Remote Transmission Request
#define STM32_CAN_TIR_IDE   (1U << 2U)  // Bit 2: Identifier Extension

#define CAN_EXT_ID_MASK     0x1FFFFFFFU
#define CAN_STD_ID_MASK     0x000007FFU


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

//RXのメッセージを一時的に格納する
twai_message_t RxMsg;


struct CAN_bit_timing_config_t{
  uint8_t TS2;
  uint8_t TS1;
  uint16_t BRP;
};



class STM32CAN{
  public:
    static STM32CAN* instance;
    STM32CAN(){
      instance = this;
    }

    bool begin(long bitrate, CANPinTypes SelectPin);

    bool send(const twai_message_t& msg){
      if (txQueue == nullptr) {
        return false;
      }
      return xQueueSend(txQueue, &msg, 0) == pdPASS;
    }

    //コールバック
    void onReceive(void (*callback)(twai_message_t msg)){
      rxCallback = callback;
    }

    void onMainLoop(void (*callback)()){
      loopCallBack = callback;
    }

    TaskHandle_t RxTaskHandle = NULL;
    TaskHandle_t LoopTaskHandle = NULL;
    TaskHandle_t TxTaskHandle = NULL;
    
    QueueHandle_t txQueue = nullptr;
    
  private:
    //内部関数を追加
    bool CANSendToFreeMailbox(twai_message_t* msg);

    void CANReceiveHardware(twai_message_t* msg);

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

    bool CANinit(long bitrate, CANPinTypes selectPin);


    //受信コールバック関数のポインタ
    void (*rxCallback)(twai_message_t msg) = nullptr;
    //メインループのコールバック関数のポインタ
    void (*loopCallBack)() = nullptr;


    static void rxTask(void* param){
      //このタスクは割り込みから実行通知を受けさせる
      STM32CAN* self = static_cast<STM32CAN*>(param);
      twai_message_t msg;
      Serial.println("RXTASK WAKE");
      while(true){
        //CANのRx割り込みISRから実行通知が来るまでブロック
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        Serial.println("Rx割り込みからタスクに実行通知が送られました");

        while (CAN1->RF0R & 0x3UL) {
          self->CANReceiveHardware(&msg);
          if (self->rxCallback) {
            self->rxCallback(msg);
          }
        }

        // FIFOを処理し終わったのでRX IRQを再有効化
        CAN1->IER |= CAN_IER_FMPIE0;

      }
    }

    static void txTask(void* param){
      STM32CAN* self = static_cast<STM32CAN*>(param);
      twai_message_t msg;
      Serial.println("TXTASK WAKE");
      while (true) {
        // TX Queueにメッセージが入るまで待つ
        if (xQueueReceive(self->txQueue, &msg, portMAX_DELAY) == pdPASS){
          Serial.println("TX Queueからメッセージを取得");
          while (!self->CANSendToFreeMailbox(&msg)) {
            vTaskDelay(pdMS_TO_TICKS(1));
          }
          Serial.println("CAN mailboxへ投入");
        }
      }
    }

    static void mainLoop(void* param){
      STM32CAN* self = static_cast<STM32CAN*>(param);
      Serial.println("MAINLOOP WAKE");
      while(true){
        if(self->loopCallBack){
          self->loopCallBack();
        }
        vTaskDelay(pdMS_TO_TICKS(100));
      }
    }
};

inline STM32CAN* STM32CAN::instance = nullptr;



bool STM32CAN::begin(long bitrate, CANPinTypes SelectPin){
  if (!CANinit(bitrate, SelectPin)) {
    Serial.println("CAN初期化失敗");
    return false;
  }
  Serial.println("CAN初期化完了");

  //キューの初期化
  txQueue = xQueueCreate(CAN_TX_QUEUE_SIZE, sizeof(twai_message_t));

  if (txQueue == nullptr) {
    Serial.println("TX Queueの作成に失敗");
    return false;
  }

  //タスクを作成
  BaseType_t isMainLoopTaskCreated, isRxTaskCreated, isTxTaskCreated;

  
  isRxTaskCreated = xTaskCreate(rxTask, "CAN_RX_Task", 512, this, 2, &RxTaskHandle);
  Serial.print("受信タスクを作成しました: ");
  Serial.println(isRxTaskCreated);
  isTxTaskCreated = xTaskCreate(txTask, "CAN_TX_Task", 512, this, 2, &TxTaskHandle);
  Serial.print("送信タスクを作成しました: ");
  Serial.println(isTxTaskCreated);
  isMainLoopTaskCreated = xTaskCreate(mainLoop, "Main_Loop", 512, this, 1, &LoopTaskHandle);
  Serial.print("メインループのタスクを作成しました: ");
  Serial.println(isMainLoopTaskCreated);

  if(isMainLoopTaskCreated!=pdPASS || isRxTaskCreated!=pdPASS || isTxTaskCreated!=pdPASS){
    Serial.println("タスクの作成に失敗しました。");
    //return false;
  }

  vTaskStartScheduler();
  
  return true;
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
 * CANフィルタのレジスタを初期化します。
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

Serial.print("APB1 clock = ");
Serial.println(HAL_RCC_GetPCLK1Freq());

PCLK1 = 32MHz
BRP 1~1024
TS1 0~15
TS2 0~7

CAN bitrate = PCLK1 / (BRP × (1 + TS1 + TS2))
       1MHz = 32MHz / (2*(1+12+3))
     0.5MHz = 32MHz / (4*(1+12+3))
    0.25MHz = 32MHz / (8*(1+12+3))
   0.125MHz = 32MHz / (16*(1+12+3))
     0.1MHz = 32MHz / (20*(1+12+3))
    0.05MHz = 32MHz / (40*(1+12+3))
*/

//要調整
inline CAN_bit_timing_config_t STM32CAN::ConvBaudrate(long baud){
  switch(baud){
    case (long)50E3:
      return {3, 12, 40};
    case (long)100E3:
      return {3, 12, 20};
    case (long)125E3:
      return {3, 12, 16};
    case (long)250E3:
      return {3, 12, 8};
    case (long)500E3:
      return {3, 12, 4};
    case (long)1000E3:
      return {3, 12, 2};
    default:
      return {3, 12, 2};
  }
}


bool STM32CAN::CANinit(long bitrate, CANPinTypes selectPin){
  RCC->APB1ENR |= RCC_APB1ENR_CANEN;

  //Serial.print("APB1 clock = ");
  //Serial.println(HAL_RCC_GetPCLK1Freq());
  switch(selectPin){
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

  #if defined(LOOPBACK)
  Serial.println("ループバックを有効化します");
  CAN1->BTR |= CAN_BTR_LBKM;
  #endif
  
  //書き込みを終了する
  CAN1->MCR &= ~CAN_MCR_INRQ;

  // フィルターの設定
  CAN1->FMR |=   0x1UL; // フィルターを初期化状態にする

  // フィルター0を初期化
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
  //CAN1->IER |= CAN_IER_TMEIE;

  //Serial.printf("TX IER after enable = 0x%08lX\n", CAN1->IER);
  
  // RX FIFO0 message pending interrupt
  CAN1->IER |= CAN_IER_FMPIE0;

  //Serial.printf("RX IER after enable = 0x%08lX\n", CAN1->IER);

  // TxのNVICによる割り込み有効化
  //NVIC_EnableIRQ(USB_HP_CAN_TX_IRQn);

  // RxのNVICによる割り込み有効化
  NVIC_SetPriority(USB_LP_CAN_RX0_IRQn, 5);
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

  return !!can1;
}


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


extern "C" void USB_LP_CAN_RX0_IRQHandler(){
  BaseType_t higherPriorityTaskWoken = pdFALSE;
  CAN1->IER &= ~CAN_IER_FMPIE0; //割り込みの連鎖が起きるのを防ぐために一旦無効化
  // RX Taskに通知
  if(STM32CAN::instance->RxTaskHandle){
    vTaskNotifyGiveFromISR(
      STM32CAN::instance->RxTaskHandle,
      &higherPriorityTaskWoken
    );
  }
  portYIELD_FROM_ISR(higherPriorityTaskWoken);
}

#endif