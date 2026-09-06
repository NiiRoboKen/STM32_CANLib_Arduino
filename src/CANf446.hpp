//現在の状態: 初期化部分の制作中

#pragma once 

#include <STM32FreeRTOS.h>
#include <Arduino.h>

#if defined(STM32F4xx)

//定数
constexpr uint8_t STM32_AF7 = 0x07;
constexpr uint8_t STM32_AF9 = 0x09;

constexpr uint8_t CAN_TX_QUEUE_SIZE = 16;
constexpr uint8_t CAN_RX_QUEUE_SIZE = 16;

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

    bool begin(long bitrate, CANPinTypes SelectPin);

    bool send(const twai_message_t& msg){
      return false;
    }

    //コールバック
    void onReceive(void (*callback)(twai_message_t msg)){
      rxCallback = callback;
    }

    void onMainLoop(void (*callback)()){
      loopCallBack = callback;
    }

    
    void handleRxInterrupt(){
      
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

    //コールバック関数のポインタ
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

      }
    }

    static void txTask(void* param){
      STM32CAN* self = static_cast<STM32CAN*>(param);
      twai_message_t msg;
      Serial.println("TXTASK WAKE");
      while (true) {
        // TX Queueにメッセージが入るまで待つ

        vTaskDelay(pdMS_TO_TICKS(10));
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
  if(!CANinit(bitrate, SelectPin)){
    Serial.println(SelectPin==PB13_PB12?"CAN2の初期化に失敗":"CAN1の初期化に失敗");
    return false;
  }
  Serial.println(SelectPin==PB13_PB12?"CAN2の初期化に成功":"CAN1の初期化に成功");

  //タスクを作成
  BaseType_t isMainLoopTaskCreated, isRxTaskCreated, isTxTaskCreated;

  isMainLoopTaskCreated = xTaskCreate(mainLoop, "Main_Loop", 512, this, 1, &LoopTaskHandle);
  Serial.print("メインループのタスクを作成しました: ");
  Serial.println(isMainLoopTaskCreated);
  isRxTaskCreated = xTaskCreate(rxTask, "CAN_RX_Task", 512, this, 2, &RxTaskHandle);
  Serial.print("受信タスクを作成しました: ");
  Serial.println(isRxTaskCreated);
  isTxTaskCreated = xTaskCreate(txTask, "CAN_TX_Task", 512, this, 2, &TxTaskHandle);
  Serial.print("送信タスクを作成しました: ");
  Serial.println(isTxTaskCreated);

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
  if (index > 27) return; //446は0~27

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

//要調整
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


bool STM32CAN::CANinit(long bitrate, CANPinTypes selectPin){

  bool useCan2 = false;
  //ピンの設定
  RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;
  if(useCan2) RCC->APB1ENR |= RCC_APB1ENR_CAN2EN;

  switch(selectPin){
    case PA12_PA11:
      RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
      CANSetGpio(GPIOA, 11, STM32_AF9);
      CANSetGpio(GPIOA, 12, STM32_AF9);
      Serial.println("CAN1");
      break;

    case PB13_PB12:
      RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
      CANSetGpio(GPIOB, 13, STM32_AF9);
      CANSetGpio(GPIOB, 12, STM32_AF9);
      Serial.println("CAN2");
      useCan2=true;
      break;
    default:
      return false;
      //例外値は無視
  }



  if(!useCan2){
    //CAN1
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

    Serial.println("ループバックを有効化します");
    CAN1->BTR |= CAN_BTR_LBKM;
  
    //書き込みを終了する
    CAN1->MCR &= ~CAN_MCR_INRQ;

    //MSRのデバッグ
    Serial.print("CAN1 MCR = 0x");
    Serial.println(CAN1->MCR, HEX);

    Serial.print("CAN1 MSR = 0x");
    Serial.println(CAN1->MSR, HEX);

    Serial.print("CAN1 BTR = 0x");
    Serial.println(CAN1->BTR, HEX);

  }else if(useCan2){
    //CAN2
    //CAN1
    CAN2->MCR |= 0x1UL;                   // CANを初期化状態にする
    while (!(CAN2->MSR & 0x1UL));         // 初期化状態になるのを待つ
    //CAN1->MCR = 0x51UL;                   // ハードウェアの初期化(自動的に再送信しない)
    //CAN1->MCR = 0x41UL;                   // ハードウェアの初期化(自動的に再送信する)
    CAN2->MCR = 0;
    CAN2->MCR |= CAN_MCR_ABOM;

    //書き込み可能にする
    CAN2->MCR |= CAN_MCR_INRQ;

    // ビットレートを設定 
    CAN_bit_timing_config_t configData = ConvBaudrate(bitrate);

    CAN2->BTR &= ~(((0x03) << 24) | ((0x07) << 20) | ((0x0F) << 16) | (0x3FF)); 
    CAN2->BTR |= (((configData.TS2-1) & 0x07) << 20) | (((configData.TS1-1) & 0x0F) << 16) | ((configData.BRP-1) & 0x3FF);

    Serial.println("ループバックを有効化します");
    CAN2->BTR |= CAN_BTR_LBKM;
  
    //書き込みを終了する
    CAN2->MCR &= ~CAN_MCR_INRQ;

    /*Serial.print("CAN2 MCR = 0x");
    Serial.println(CAN2->MCR, HEX);

    Serial.print("CAN2 MSR = 0x");
    Serial.println(CAN2->MSR, HEX);

    Serial.print("CAN2 BTR = 0x");
    Serial.println(CAN2->BTR, HEX);*/
  }



  // フィルターの設定
  
  /*
    現段階ではフィルター0をCAN1、フィルター1をCAN2に割り当てています。
    デフォルト設定ではすべてのIDのメッセージを受信します。
    IDの振り分けをソフトウェアでやりたくない!
    というような場合にあとフィルターはあと26個拡張できます。(446のフィルターは全部で28個のため)
  */
  CAN1->FMR |= CAN_FMR_FINIT;//フィルター設定開始
  if (useCan2) {
    // Bank 0 → CAN1
    // Bank 1 → CAN2
    // Bank 2~27→ 未使用

    CAN1->FMR &= ~(0x3FUL << 8);
    CAN1->FMR |=  (1UL << 8);

    CANSetFilter(1, 1, 0, 0, 0x0UL, 0x0UL);
  } else {
    // CAN1 → Bank 0
    CANSetFilter(0, 1, 0, 0, 0x0UL, 0x0UL);
  }
  CAN1->FMR &= ~CAN_FMR_FINIT;//フィルター設定終了
  

  if(!useCan2){
    bool can1 = false;
    CAN1->MCR &= ~(0x1UL);                // Require CAN1 to normal mode 

    //割り込み有効化

    //Time inperruptの有効化
    //CAN1->IER |= CAN_IER_TMEIE;
    // RX FIFO0 message pending interrupt
    CAN1->IER |= CAN_IER_FMPIE0;

    // TxのNVICによる割り込み有効化
    //NVIC_EnableIRQ(USB_HP_CAN_TX_IRQn);

    // RxのNVICによる割り込み有効化
    NVIC_SetPriority(CAN1_RX0_IRQn, 5);
    NVIC_EnableIRQ(CAN1_RX0_IRQn);
    // Wait for normal mode
    // If the connection is not correct, it will not return to normal mode.
    uint16_t TimeoutMilliseconds = 1000;
    uint16_t wait_ack = 0;
    while(wait_ack < TimeoutMilliseconds){
      wait_ack++;
      if((CAN1->MSR & 0x1UL) == 0){
        can1 = true;
        break;
      }
      delay(1);
    }

    return !!can1;

  }else {
    bool can2 = false;
    CAN2->MCR &= ~(0x1UL);                // Require CAN1 to normal mode 

    //割り込み有効化

    //Time inperruptの有効化
    //CAN1->IER |= CAN_IER_TMEIE;
    // RX FIFO0 message pending interrupt
    CAN2->IER |= CAN_IER_FMPIE0;

    // TxのNVICによる割り込み有効化
    //NVIC_EnableIRQ(USB_HP_CAN_TX_IRQn);

    // RxのNVICによる割り込み有効化
    NVIC_SetPriority(CAN2_RX0_IRQn, 5);
    NVIC_EnableIRQ(CAN2_RX0_IRQn);
    // Wait for normal mode
    // If the connection is not correct, it will not return to normal mode.
    uint16_t TimeoutMilliseconds = 1000;
    uint16_t wait_ack = 0;
    while(wait_ack < TimeoutMilliseconds){
      wait_ack++;
      if((CAN2->MSR & 0x1UL) == 0){
        can2 = true;
        break;
      }
      delay(1);
    }

    return !!can2;

  }

  return false;
}

#endif