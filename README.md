# STM32_CANLib_Arduino
STM32 F303K8, F446RE 用のArduinoフレームワークCANライブラリ（F303とF446間での動作確認中）

未完成・改造中<br>
なるべくesp-canに似た操作感になるように頑張ります!

## 現在の状態
f303はループバックでは正常にテスト出来ていることを確認しました
次はCANトランシーバーを使用して送受信、通信速度が確かに反映されているのかを確認します。

f446の方は初期化過程でINAKビットが立っていて通常モードに入ってくれないバグがあるのでそこを修正中です。

それぞれのBTRレジスタに代入する設定値は一応再計算しました。
## 使い方

### 例文(完成予定の構文)
```cpp
#include <Arduino.h>
#include "STM32_CAN.hpp"

CanDriver can;

void RxCallBack(twai_message_t msg){
  //10進数表示
  Serial.println(msg.identifier); //291
  //16進数表示
  Serial.println(msg.identifier, HEX); //123
  
  Serial.println(msg.data[0]); //0
  Serial.println(msg.data[1]); //1
  Serial.println(msg.data[2]); //2
  Serial.println(msg.data[3]); //3
  Serial.println(msg.data[4]); //4
  Serial.println(msg.data[5]); //5
  Serial.println(msg.data[6]); //6
  Serial.println(msg.data[7]); //7
}

volatile bool a = false;
void mainLoop(){
  if(!a){
    Serial.println("mainloop call!");

    twai_message_t msg{};

    msg.extd = STANDARD_FORMAT;
    msg.rtr = DATA_FRAME;
    msg.identifier = 0x123;
    msg.data_length_code = 8;

    for (int i = 0; i < 8; i++) {
      msg.data[i] = i;
    }

    Serial.println(can.send(msg));

    a=true;
  }
}

void setup(){
  Serial.begin(115200);
  can.onReceive(&RxCallBack);
  can.onMainLoop(&mainLoop);
  can.begin(1000000, PA12_PA11);
}

//標準ループが使用不可?
void loop(){}
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

freeRTOSを積む意味はあったのだろうか...?
