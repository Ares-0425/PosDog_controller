#ifndef CAN_HPP
#define CAN_HPP

#include <Arduino.h>
#include <driver/twai.h>

#ifdef M5STACK_CORE2
  #define CRX GPIO_NUM_33
  #define CTX GPIO_NUM_32
#elif M5STACK_CORES3
  #define CRX GPIO_NUM_1
  #define CTX GPIO_NUM_2
#elif M5ATOM_S3
  #define CRX GPIO_NUM_1
  #define CTX GPIO_NUM_2
  #define Serial USBSerial
#endif

// グローバル変数宣言
extern twai_message_t rx_message;
extern twai_message_t tx_message;

typedef struct { int id; uint8_t len; uint8_t data[8]; } CANFrame;

// グローバル関数宣言
void CAN_setup(void);

#endif