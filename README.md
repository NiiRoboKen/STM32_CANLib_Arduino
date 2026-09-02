# STM32_CANLib_Arduino
未完成・改造中
STM32 F303K8, F446RE 用のArduinoフレームワークCANライブラリ（F303とF446間での動作確認中）

---

## 使い方

なるべくesp-canに似た操作感になるように頑張ります!
---
### 例文
```
#include "STM32F303K8_CAN.hpp"

STM32CAN can;

void setup(){
  Serial.begin(115200);
  can.begin(1000000, PA11_PA12);
}

void loop(){
    CAN_msg_t rx;

    if(can.receive(&rx)){
        Serial.println(rx.id, HEX);
        Serial.println(rx.data[0], HEX);
        Serial.println(rx.data[1], HEX);
    }

    CAN_msg_t msg;

    msg.id = 0x123;
    msg.format = STANDARD_FORMAT;
    msg.type = DATA_FRAME;
    msg.len = 2; //dataの長さ
    msg.data[0] = 0x11;
    msg.data[1] = 0x22;


    can.send(msg);
    delay(1000);
}
```

---
## 注意点
未完成
