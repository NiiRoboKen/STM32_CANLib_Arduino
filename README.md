# STM32_CANLib_Arduino
STM32 F303K8, F446RE 用のArduinoフレームワークCANライブラリ（F303とF446間での動作確認中）

---
未完成・改造中<br>
なるべくesp-canに似た操作感になるように頑張ります!

## 使い方

### 例文
```
#include <Arduino.h>
#include "STM32_CAN.hpp"

CanDriver can;

void canCallback(twai_message_t msg) {
  Serial.printf("RX <- ID:0x%lX DLC:%d DATA:", msg.identifier, msg.data_length_code);
  for (int i = 0; i < msg.data_length_code; i++) {
    Serial.print(" %02X", msg.data[i]);
  }
  Serial.println();
}

void setup() {
  if(can.begin(1000E3, PA12_PA11)) {
    Serial.println("OK");
  }
  can.onReceive(canCallback);
}

void loop() {
  uint32_t id = 0x00;
  uint8_t data[8] = {1,2,3,4,5,6,7,8};
  can.send(id, data, sizeof(data));
  delay(1000);
}
```

---
## 注意点
beginに渡すピンの設定
<br>
f303: PA12_PA11 (tx/rx, CAN1)<br>
f446: PA12_PA11 (tx/rx, CAN1)<br>
      PB13_PB12 (tx/rx, CAN2)<br>
ESP32の方と同じ変数名にするために、twai_message_tとしています。
