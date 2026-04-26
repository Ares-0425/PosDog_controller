#include <Arduino.h>
#include <M5Unified.h>
#include "can.hpp"
#include "display.hpp"
#include "control.hpp"
#include "imu.hpp"

#ifdef M5ATOM_S3
  #define Serial USBSerial
#endif

int mode = 1;

void setup() {
  Serial.begin(115200); // シリアル通信速度の設定

  M5.begin(); // M5Stackの初期化
  canvas.setColorDepth(16);
  canvas.createSprite(M5.Display.width(), M5.Display.height());
  display_clear(); // ディスプレイをクリア
  display_println("Start..."); // 初期メッセージを表示

  Wire1.begin(21,22); // I2Cの初期化
  Wire1.setClock(400000); // I2Cのクロック速度を400kHzに設定
  // for (uint8_t addr = 1; addr < 127; addr++) {
  //   Wire1.beginTransmission(addr);
  //   if (Wire1.endTransmission() == 0) {
  //     Serial.printf("Found: 0x%02X\n", addr);
  //   }
  // }

  // IMU1の初期化
  if(lsm1.begin(IMU_1AG_ADDRESS, IMU_1M_ADDRESS)){
    display_println("IMU1 initialized");
  }
  // IMU2の初期化
  if(lsm2.begin(IMU_2AG_ADDRESS, IMU_2M_ADDRESS)){
    display_println("IMU2 initialized");
  }
  setupSensor(); // センサーの設定

  CAN_setup();  // CANの設定
  display_println("CAN setup complete"); // CAN設定完了メッセージを表示

  delay(1000); // 1秒待機
  display_menu();
  display_println("1");
}

void loop() {
  M5.update();
  if(Serial.available()){
    uart2CAN();
  }
  if(twai_receive(&rx_message, pdMS_TO_TICKS(0)) == ESP_OK){
    if(rx_message.data_length_code == 8){
      Serial.printf("%f, %f\n",*((float*)&rx_message.data[0]), *((float*)&rx_message.data[4]));
    }else{
      Serial.printf("%c%f\n",rx_message.data[0],*((float*)&rx_message.data[1]));
    }
  }
  if(M5.BtnC.wasPressed()){
    mode++; // モードを切り替え
    display_menu();
    if(mode > 8) {
      mode = 1; // モードが3を超えたら1に戻す
    }
    M5.Display.startWrite();
    M5.Display.println(String(mode));
    M5.Display.endWrite();
  }
  if(M5.BtnB.wasPressed()){
    switch(mode) {
      case 1:
        sineWave();
        break;
      case 2:
        imuMode();
        break;
      case 3:
        imuMode2();
        break;
      case 4:
        pace_FixCycle2();
        break;
      case 5:
        trot_FixCycle();
        break;
      case 6:
        pace_VirtualPassiveBase2();
        break;
      case 7:
        trot_VirtualPassiveBase();
        break;
      case 8:
        pace_VirtualPassiveBase_rev();
        break;
      default:
        break;
    }
    display_menu();
  }
  if(M5.BtnA.wasPressed()){
    mode--;
    display_menu();
    if(mode < 1) {
      mode = 8; // モードが1未満になったら6に戻す
    }
    M5.Display.startWrite();
    M5.Display.println(String(mode));
    M5.Display.endWrite();
  }
}