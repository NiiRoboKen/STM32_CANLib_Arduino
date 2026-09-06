# STM32_CANLib_Arduino
STM32 F303K8, F446RE 用のArduinoフレームワークCANライブラリ（F303とF446間での動作確認中）

未完成・改造中<br>
なるべくesp-canに似た操作感になるように頑張ります!

## 使い方

### 例文(完成予定の構文)
```cpp
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
  Serial.begin(115200);
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
beginに渡すピンの設定 enum CanPinTypes
<br>
|STM32|CAN1,2|PIN(tx_rx)|
|--|--|--|
|f303|CAN1|PA12_PA11|
|  |  |  |
|f446|CAN1|PA12_PA11|
|  |CAN2|PB13_PB12|

ESP32の方と同ように扱えるようにするために、twai_message_tとしています。中の変数名も揃えています。（一部未使用の変数は削除してあります）
