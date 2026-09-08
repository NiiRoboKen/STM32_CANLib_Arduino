//現在の状態: 初期化部分の制作中

#pragma once 

#include <STM32FreeRTOS.h>
#include <Arduino.h>

#if defined(STM32F4xx)

//定数
#define CAN_STD_ID_MASK  0x000007FFUL
#define CAN_EXT_ID_MASK  0x1FFFFFFFUL

#define STM32_CAN_TIR_IDE   (1UL << 2)
#define STM32_CAN_TIR_RTR   (1UL << 1)
#define STM32_CAN_TIR_TXRQ  (1UL << 0)

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
  uint16_t BRP;
};

//それぞれのCANの使用状態の管理(複数インスタンスでの同一CANの使用を防ぐため)
struct CAN_using{
  bool CAN1using = false;
  bool CAN2using = false;
};
inline CAN_using CAN_USING{};


class STM32CAN{
  public:
    static STM32CAN* can1Instance;
    static STM32CAN* can2Instance;

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
    bool useCan2 = false; //タスク内での判定でも使うのでここに昇格

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

        Serial.println("Rx割り込みからタスクに実行通知が送られました");

        if(!self->useCan2){
          while (CAN1->RF0R & 0x3UL) {
            self->CANReceiveHardware(&msg);
            if (self->rxCallback) {
              self->rxCallback(msg);
            }
          }
          // FIFOを処理し終わったのでRX IRQを再有効化
          CAN1->IER |= CAN_IER_FMPIE0;
        }else if(self->useCan2){
          while (CAN2->RF0R & 0x3UL) {
            self->CANReceiveHardware(&msg);
            if (self->rxCallback) {
              self->rxCallback(msg);
            }
          }
          // FIFOを処理し終わったのでRX IRQを再有効化
          CAN2->IER |= CAN_IER_FMPIE0;

        }
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

inline STM32CAN* STM32CAN::can1Instance = nullptr;
inline STM32CAN* STM32CAN::can2Instance = nullptr;

bool STM32CAN::begin(long bitrate, CANPinTypes SelectPin){
  //複数インスタンスで一つのCANを使用できないようにする
  if(SelectPin==PB13_PB12){
    if(!(CAN_USING.CAN2using)){
      CAN_USING.CAN2using = true;
      can2Instance=this;
    }else{
      Serial.println("複数インスタンスによる一つのCANバスの操作はサポートされていません");
      return false;
    }
  }else{
    if(!(CAN_USING.CAN1using)){
      CAN_USING.CAN1using = true;
      can1Instance=this;
    }else{
      Serial.println("複数インスタンスによる一つのCANバスの操作はサポートされていません");
      return false;
    }
  }

  if(!CANinit(bitrate, SelectPin)){
    Serial.println(SelectPin==PB13_PB12?"CAN2の初期化に失敗":"CAN1の初期化に失敗");
    if(useCan2){
      CAN_USING.CAN2using = false;
      can2Instance = nullptr;
    }else{
      CAN_USING.CAN1using = false;
      can1Instance = nullptr;
    }
    return false;
  }
  Serial.println(SelectPin==PB13_PB12?"CAN2の初期化に成功":"CAN1の初期化に成功");

  //キューの初期化
  txQueue = xQueueCreate(CAN_TX_QUEUE_SIZE, sizeof(twai_message_t));

  if (txQueue == nullptr) {
    Serial.println("TX Queueの作成に失敗");
    if(useCan2){
      CAN_USING.CAN2using = false;
      can2Instance = nullptr;
    }else{
      CAN_USING.CAN1using = false;
      can1Instance = nullptr;
    }
    return false;
  }

  //タスクを作成
  BaseType_t isMainLoopTaskCreated, isRxTaskCreated, isTxTaskCreated;

  isMainLoopTaskCreated = xTaskCreate(mainLoop, "Main_Loop", 512, this, 2, &LoopTaskHandle);
  Serial.print("メインループのタスクを作成しました: ");
  Serial.println(isMainLoopTaskCreated);
  isRxTaskCreated = xTaskCreate(rxTask, "CAN_RX_Task", 512, this, 1, &RxTaskHandle);
  Serial.print("受信タスクを作成しました: ");
  Serial.println(isRxTaskCreated);
  isTxTaskCreated = xTaskCreate(txTask, "CAN_TX_Task", 512, this, 1, &TxTaskHandle);
  Serial.print("送信タスクを作成しました: ");
  Serial.println(isTxTaskCreated);

  if(isMainLoopTaskCreated!=pdPASS || isRxTaskCreated!=pdPASS || isTxTaskCreated!=pdPASS){
    Serial.println("タスクの作成に失敗しました。");
    if(useCan2){
      CAN_USING.CAN2using = false;
      can2Instance = nullptr;
    }else{
      CAN_USING.CAN1using = false;
      can1Instance = nullptr;
    }
    return false;
  }

  //vTaskStartScheduler();

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
    addr->PUPDR |= 0x1 << _index2;  // Pull-Up
}


/**
 * CANフィルタのレジスタを初期化します。
 * 
 * The bxCAN provides up to 28 scalable/configurable identifier filter banks, for selecting the incoming messages, that the software needs and discarding the others.
 *
 * @preconditions   - This register can be written only when the filter initialization mode is set (FINIT=1) in the CAN_FMR register.
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
  uint16_t BRP;
};

BRP 1~1024
TS1 0~15
TS2 0~7

CAN bitrate = PCLK1 / (BRP × (1 + TS1 + TS2))
       1MHz = 45MHz / (3*(1+12+2))
     0.5MHz = 45MHz / (6*(1+12+2))
    0.25MHz = 45MHz / (12*(1+12+2))
    0.125MHz = 45MHz / (36*(1+7+2))
     0.1MHz = 45MHz / (30*(1+12+2))
    0.05MHz = 45MHz / (60*(1+12+2))
*/

inline CAN_bit_timing_config_t STM32CAN::ConvBaudrate(long baud){
  switch(baud){
    case (long)50E3:
      return {2, 12, 60};
    case (long)100E3:
      return {2, 12, 30};
    case (long)125E3:
      return {2, 7, 36};
    case (long)250E3:
      return {2, 12, 12};
    case (long)500E3:
      return {2, 12, 6};
    case (long)1000E3:
      return {2, 12, 3};
    default:
      return {2, 12, 3};
  }
}


bool STM32CAN::CANinit(long bitrate, CANPinTypes selectPin){
  if(selectPin==PB13_PB12) useCan2=true;
  
  //ピンの設定
  RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;
  if(useCan2) RCC->APB1ENR |= RCC_APB1ENR_CAN2EN;

  switch(selectPin){
    case PA12_PA11:
      SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN);
      CANSetGpio(GPIOA, 12, STM32_AF9);
      CANSetGpio(GPIOA, 11, STM32_AF9);
      //割り込み有効化

      // RX FIFO0 message pending interrupt
      SET_BIT(CAN1->IER, CAN_IER_FMPIE0);

      // RxのNVICによる割り込み有効化
      NVIC_SetPriority(CAN1_RX0_IRQn, 5);
      NVIC_EnableIRQ(CAN1_RX0_IRQn);
      break;
    case PB13_PB12:
      SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN);
      CANSetGpio(GPIOB, 13, STM32_AF9);
      CANSetGpio(GPIOB, 12, STM32_AF9);

      //割り込み有効化

      // RX FIFO0 message pending interrupt
      CAN2->IER |= CAN_IER_FMPIE0;

      // RxのNVICによる割り込み有効化
      NVIC_SetPriority(CAN2_RX0_IRQn, 5);
      NVIC_EnableIRQ(CAN2_RX0_IRQn);
      break;
    default:
      return false;
      //例外値は無視
  }

  // フィルターの設定
  
  /*
    現段階ではフィルター0をCAN1、フィルター1をCAN2に割り当てています。
    デフォルト設定ではすべてのIDのメッセージを受信します。
    あとフィルターバンクはあと26個拡張できます。(446のフィルターバンクは全部で28個のため)
  */
  SET_BIT(CAN1->FMR, CAN_FMR_FINIT); //フィルター設定開始
  if (useCan2) {
    // CAN2 → Bank1
    // Bank 2~27→ 未使用

    CLEAR_BIT(CAN1->FMR, (0x3FUL << 8));
    SET_BIT(CAN1->FMR, (1UL << 8));
    CANSetFilter(1, 1, 0, 0, 0x0UL, 0x0UL);
  } else {
    // CAN1 → Bank 0
    CANSetFilter(0, 1, 0, 0, 0x0UL, 0x0UL);
  }
  CLEAR_BIT(CAN1->FMR, CAN_FMR_FINIT); //フィルター設定終了

  //ビットレート、割り込みの設定
  if(!useCan2){
    //CAN1
    
    //スリープ解除
    CLEAR_BIT(CAN1->MCR, CAN_MCR_SLEEP);
    //while ((CAN1->MSR & CAN_MSR_SLAK) != 0); //SLEEPから起動するまで待つ
    SET_BIT(CAN1->MCR, CAN_MCR_INRQ); //CANを初期化状態にする
    while (!(CAN1->MSR & CAN_MSR_INAK)); //初期化状態になるのを待つ
    
    SET_BIT(CAN1->MCR, CAN_MCR_ABOM); //自動バスオフ管理を有効にする
    
    // ビットレートを設定 
    CAN_bit_timing_config_t configData = ConvBaudrate(bitrate);

    CLEAR_BIT(CAN1->BTR, ((0x03) << 24) | ((0x07) << 20) | ((0x0F) << 16) | (0x3FF));
    SET_BIT(CAN1->BTR, (((configData.TS2-1) & 0x07) << 20) | (((configData.TS1-1) & 0x0F) << 16) | ((configData.BRP-1) & 0x3FF));

    //Serial.println("ループバックを有効化します");
    //SET_BIT(CAN1->BTR, CAN_BTR_LBKM);
  
    Serial.print("BTR = 0x");
    Serial.println(CAN1->BTR, HEX);

    Serial.print("LBKM = ");
    Serial.println((CAN1->BTR & CAN_BTR_LBKM) ? 1 : 0);

    Serial.print("MCR = 0x");
    Serial.println(CAN1->MCR, HEX);

    Serial.print("Before MSR = 0x");
    Serial.println(CAN1->MSR, HEX);


    CLEAR_BIT(CAN1->MCR, CAN_MCR_INRQ); //書き込みを終了する


    Serial.print("After MSR = 0x");
    Serial.println(CAN1->MSR, HEX);

    Serial.print("INRQ = ");
    Serial.println((CAN1->MCR & CAN_MCR_INRQ) ? 1 : 0);

    Serial.print("SLEEP = ");
    Serial.println((CAN1->MCR & CAN_MCR_SLEEP) ? 1 : 0);

    Serial.print("SLAK = ");
    Serial.println((CAN1->MSR & CAN_MSR_SLAK) ? 1 : 0);

    Serial.print("INAK = ");
    Serial.println((CAN1->MSR & CAN_MSR_INAK) ? 1 : 0);

    // Wait for normal mode
    int timelimit = 0;
    while (CAN1->MSR & CAN_MSR_INAK) {
      delay(1);
      if (++timelimit > 1000) return false;
    }

    return true;
  }else if(useCan2){
    //CAN2
    CLEAR_BIT(CAN2->MCR, CAN_MCR_SLEEP); 
    SET_BIT(CAN2->MCR, CAN_MCR_INRQ); // CANを初期化状態にする
    while (!(CAN2->MSR & CAN_MSR_INAK)); // 初期化状態になるのを待つ

    SET_BIT(CAN2->MCR, CAN_MCR_ABOM);

    // ビットレートを設定 
    CAN_bit_timing_config_t configData = ConvBaudrate(bitrate);

    CLEAR_BIT(CAN2->BTR, ((0x03) << 24) | ((0x07) << 20) | ((0x0F) << 16) | (0x3FF));
    SET_BIT(CAN2->BTR, (((configData.TS2-1) & 0x07) << 20) | (((configData.TS1-1) & 0x0F) << 16) | ((configData.BRP-1) & 0x3FF));

    //Serial.println("ループバックを有効化します");
    //SET_BIT(CAN2->BTR, CAN_BTR_LBKM);
  
    CLEAR_BIT(CAN2->MCR, CAN_MCR_INRQ); //書き込みを終了する

    // Wait for normal mode
    int timelimit = 0;
    while(CAN2->MSR & CAN_MSR_INAK){
      delay(1);
      timelimit++;
      if(timelimit>1000) return false;
    }

    return true;
  }

  return false;
}



/**
 * ハードウェアFIFOから読み出す関数
 * 
 * @preconditions     - A valid CAN message is received
 * @params CAN_rx_msg - CAN message structure for reception
 * 
 */
inline void STM32CAN::CANReceiveHardware(twai_message_t* CAN_rx_msg){
  uint32_t id = useCan2 ? CAN2->sFIFOMailBox[0].RIR : CAN1->sFIFOMailBox[0].RIR;
  if ((id & CAN_RI0R_IDE) == 0) { // Standard frame format
      CAN_rx_msg->extd = STANDARD_FORMAT;
      CAN_rx_msg->identifier = (CAN_STD_ID_MASK & (id >> 21U));
  }
  else {                               // Extended frame format
      CAN_rx_msg->extd = EXTENDED_FORMAT;
      CAN_rx_msg->identifier = (CAN_EXT_ID_MASK & (id >> 3U));
  }

  if ((id & CAN_RI0R_RTR) == 0) { // Data frame
      CAN_rx_msg->rtr = DATA_FRAME;
  }
  else {                               // Remote frame
      CAN_rx_msg->rtr = REMOTE_FRAME;
  }

  if(useCan2){
    CAN_rx_msg->data_length_code = (CAN2->sFIFOMailBox[0].RDTR) & 0xFUL;
    CAN_rx_msg->data[0] = 0xFFUL &  CAN2->sFIFOMailBox[0].RDLR;
    CAN_rx_msg->data[1] = 0xFFUL & (CAN2->sFIFOMailBox[0].RDLR >> 8);
    CAN_rx_msg->data[2] = 0xFFUL & (CAN2->sFIFOMailBox[0].RDLR >> 16);
    CAN_rx_msg->data[3] = 0xFFUL & (CAN2->sFIFOMailBox[0].RDLR >> 24);
    CAN_rx_msg->data[4] = 0xFFUL &  CAN2->sFIFOMailBox[0].RDHR;
    CAN_rx_msg->data[5] = 0xFFUL & (CAN2->sFIFOMailBox[0].RDHR >> 8);
    CAN_rx_msg->data[6] = 0xFFUL & (CAN2->sFIFOMailBox[0].RDHR >> 16);
    CAN_rx_msg->data[7] = 0xFFUL & (CAN2->sFIFOMailBox[0].RDHR >> 24);
    
    // Release FIFO 0 output mailbox.
    // Make the next incoming message available.
    CAN2->RF0R |= 0x20UL;
  }else{
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
}


//空きMainboxにデータを送る
inline bool STM32CAN::CANSendToFreeMailbox(twai_message_t* CAN_tx_msg){
    uint8_t mailbox;

    if(useCan2){//inline関数なのでメンバ変数が使える
      if (CAN2->TSR & CAN_TSR_TME0) {
        mailbox = 0;
      }else if (CAN2->TSR & CAN_TSR_TME1) {
        mailbox = 1;
      }else if (CAN2->TSR & CAN_TSR_TME2) {
        mailbox = 2;
      }else {
        return false;
      }
    }else{
      if (CAN1->TSR & CAN_TSR_TME0) {
        mailbox = 0;
      }else if (CAN1->TSR & CAN_TSR_TME1) {
        mailbox = 1;
      }else if (CAN1->TSR & CAN_TSR_TME2) {
        mailbox = 2;
      }else {
        return false;
      }
    }
    // 空きMailbox探索
    

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

    if(useCan2){
      // DLC
      CAN2->sTxMailBox[mailbox].TDTR =
        (CAN_tx_msg->data_length_code & 0xFUL);

      // DATA LOW
      CAN2->sTxMailBox[mailbox].TDLR =
        (((uint32_t)CAN_tx_msg->data[3] << 24) |
         ((uint32_t)CAN_tx_msg->data[2] << 16) |
         ((uint32_t)CAN_tx_msg->data[1] << 8 ) |
         ((uint32_t)CAN_tx_msg->data[0]));

      // DATA HIGH
      CAN2->sTxMailBox[mailbox].TDHR =
        (((uint32_t)CAN_tx_msg->data[7] << 24) |
         ((uint32_t)CAN_tx_msg->data[6] << 16) |
         ((uint32_t)CAN_tx_msg->data[5] << 8 ) |
         ((uint32_t)CAN_tx_msg->data[4]));

      // 送信開始
      CAN2->sTxMailBox[mailbox].TIR = out | STM32_CAN_TIR_TXRQ;
    }else{
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
    }
    

    return true;
}


//RX割り込みISR関数
extern "C" void CAN1_RX0_IRQHandler(){
  BaseType_t higherPriorityTaskWoken = pdFALSE;
  // CAN1 FIFO0 RX割り込みを一旦無効化
  CAN1->IER &= ~CAN_IER_FMPIE0;
  if (STM32CAN::can1Instance && STM32CAN::can1Instance->RxTaskHandle) {
    vTaskNotifyGiveFromISR(STM32CAN::can1Instance->RxTaskHandle, &higherPriorityTaskWoken);
  }
  portYIELD_FROM_ISR(higherPriorityTaskWoken);
}

extern "C" void CAN2_RX0_IRQHandler(){
  BaseType_t higherPriorityTaskWoken = pdFALSE;
  // CAN2 FIFO0 RX割り込みを一旦無効化
  CAN2->IER &= ~CAN_IER_FMPIE0;
  if (STM32CAN::can2Instance && STM32CAN::can2Instance->RxTaskHandle) {
    vTaskNotifyGiveFromISR(STM32CAN::can2Instance->RxTaskHandle, &higherPriorityTaskWoken);
  }
  portYIELD_FROM_ISR(higherPriorityTaskWoken);
}

#endif