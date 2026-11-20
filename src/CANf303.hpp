#pragma once

#include <STM32FreeRTOS.h>
#include <Arduino.h>
#include <stm32f3xx_hal.h>
#include <stm32f3xx_hal_can.h>

#if defined(STM32F3xx)

//ループバックの有効化
#define LOOPBACK

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

    void handleHalIRQ(){
      HAL_CAN_IRQHandler(&canHandle);
    }

    void notifyRxFromISR(CAN_HandleTypeDef* handle){
      if (handle != &canHandle || RxTaskHandle == nullptr) {
        return;
      }
      BaseType_t higherPriorityTaskWoken = pdFALSE;
      vTaskNotifyGiveFromISR(RxTaskHandle, &higherPriorityTaskWoken);
      portYIELD_FROM_ISR(higherPriorityTaskWoken);
    }

    TaskHandle_t RxTaskHandle = NULL;
    TaskHandle_t LoopTaskHandle = NULL;
    TaskHandle_t TxTaskHandle = NULL;
    
    QueueHandle_t txQueue = nullptr;
    
  private:
    bool CANSendToFreeMailbox(twai_message_t* msg);

    void CANReceiveHardware(twai_message_t* msg);

    CAN_bit_timing_config_t ConvBaudrate(long baud);

    bool CANinit(long bitrate, CANPinTypes selectPin);

    CAN_HandleTypeDef canHandle{};


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

        while (HAL_CAN_GetRxFifoFillLevel(&self->canHandle, CAN_RX_FIFO0) > 0) {
          self->CANReceiveHardware(&msg);
          if (self->rxCallback) {
            self->rxCallback(msg);
          }
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
    return false;
  }

  vTaskStartScheduler();
  
  return true;
}


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
  if (selectPin != PA12_PA11) {
    return false;
  }

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_CAN1_CLK_ENABLE();

  GPIO_InitTypeDef gpioConfig{};
  gpioConfig.Pin = GPIO_PIN_11 | GPIO_PIN_12;
  gpioConfig.Mode = GPIO_MODE_AF_PP;
  gpioConfig.Pull = GPIO_NOPULL;
  gpioConfig.Speed = GPIO_SPEED_FREQ_HIGH;
  gpioConfig.Alternate = GPIO_AF9_CAN;
  HAL_GPIO_Init(GPIOA, &gpioConfig);

  CAN_bit_timing_config_t configData = ConvBaudrate(bitrate);

  canHandle.Instance = CAN1;
  canHandle.Init.Prescaler = configData.BRP;
  canHandle.Init.Mode = CAN_MODE_NORMAL;
  canHandle.Init.SyncJumpWidth = CAN_SJW_1TQ;
  canHandle.Init.TimeSeg1 = CAN_BS1_12TQ;
  canHandle.Init.TimeSeg2 = CAN_BS2_3TQ;
  canHandle.Init.TimeTriggeredMode = DISABLE;
  canHandle.Init.AutoBusOff = ENABLE;
  canHandle.Init.AutoWakeUp = DISABLE;
  canHandle.Init.AutoRetransmission = ENABLE;
  canHandle.Init.ReceiveFifoLocked = DISABLE;
  canHandle.Init.TransmitFifoPriority = DISABLE;

  #if defined(LOOPBACK)
  Serial.println("ループバックを有効化します");
  canHandle.Init.Mode = CAN_MODE_LOOPBACK;
  #endif

  if (HAL_CAN_Init(&canHandle) != HAL_OK) {
    return false;
  }

  CAN_FilterTypeDef filterConfig{};
  filterConfig.FilterBank = 0;
  filterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  filterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  filterConfig.FilterIdHigh = 0;
  filterConfig.FilterIdLow = 0;
  filterConfig.FilterMaskIdHigh = 0;
  filterConfig.FilterMaskIdLow = 0;
  filterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  filterConfig.FilterActivation = ENABLE;
  filterConfig.SlaveStartFilterBank = 14;
  if (HAL_CAN_ConfigFilter(&canHandle, &filterConfig) != HAL_OK ||
      HAL_CAN_ActivateNotification(&canHandle, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
    return false;
  }

  NVIC_SetPriority(USB_LP_CAN_RX0_IRQn, 5);
  NVIC_EnableIRQ(USB_LP_CAN_RX0_IRQn);
  if (HAL_CAN_Start(&canHandle) != HAL_OK) {
    return false;
  }
  return true;
}


/**
 * ハードウェアFIFOから読み出す関数
 * 
 * @preconditions     - A valid CAN message is received
 * @params CAN_rx_msg - CAN message structure for reception
 * 
 */
inline void STM32CAN::CANReceiveHardware(twai_message_t* CAN_rx_msg){
  CAN_RxHeaderTypeDef header{};
  uint8_t data[8]{};
  if (HAL_CAN_GetRxMessage(&canHandle, CAN_RX_FIFO0, &header, data) != HAL_OK) {
    return;
  }

  CAN_rx_msg->extd = header.IDE == CAN_ID_EXT ? EXTENDED_FORMAT : STANDARD_FORMAT;
  CAN_rx_msg->rtr = header.RTR == CAN_RTR_REMOTE ? REMOTE_FRAME : DATA_FRAME;
  CAN_rx_msg->identifier = header.IDE == CAN_ID_EXT ? header.ExtId : header.StdId;
  CAN_rx_msg->data_length_code = static_cast<uint8_t>(header.DLC);
  memcpy(CAN_rx_msg->data, data, sizeof(data));
}


//空きMainboxにデータを送る
inline bool STM32CAN::CANSendToFreeMailbox(twai_message_t* CAN_tx_msg){
  if (CAN_tx_msg->data_length_code > 8) {
    return false;
  }

  CAN_TxHeaderTypeDef header{};
  header.IDE = CAN_tx_msg->extd == EXTENDED_FORMAT ? CAN_ID_EXT : CAN_ID_STD;
  header.StdId = CAN_tx_msg->identifier & CAN_STD_ID_MASK;
  header.ExtId = CAN_tx_msg->identifier & CAN_EXT_ID_MASK;
  header.RTR = CAN_tx_msg->rtr == REMOTE_FRAME ? CAN_RTR_REMOTE : CAN_RTR_DATA;
  header.DLC = CAN_tx_msg->data_length_code;
  header.TransmitGlobalTime = DISABLE;

  uint32_t mailbox;
  return HAL_CAN_AddTxMessage(&canHandle, &header, CAN_tx_msg->data, &mailbox) == HAL_OK;
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* handle){
  if (STM32CAN::instance != nullptr) {
    STM32CAN::instance->notifyRxFromISR(handle);
  }
}

extern "C" void USB_LP_CAN_RX0_IRQHandler(){
  if (STM32CAN::instance != nullptr) {
    STM32CAN::instance->handleHalIRQ();
  }
}

#endif