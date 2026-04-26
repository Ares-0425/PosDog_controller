#ifndef CONTROL_HPP
#define CONTROL_HPP

#include <Arduino.h>
#include <M5Unified.h>

// グローバル変数宣言
extern volatile uint8_t Loop_flag;

// グローバル関数宣言
void canTest(void);
void sendStop(void);
void sendStart(void);
void torqueMode(void);
void speedMode(void);
void positionMode(void);
void resetMotor(void);
void calibration(int);
void uart2CAN(void);
void sineWave(void);
void pace_FixCycle(void);
void pace_SimpleVariationCycle(void);
void pace_VariationCycle(void);
void pace_VirtualPassiveBase(void);
void pace_VirtualPassiveBase_rev(void);
void pace_VirtualPassiveBase2(void);
void pace_VirtualPassiveBase2_rev(void);
void pace_FixCycle2(void);
void trot_FixCycle(void);
void trot_VirtualPassiveBase(void);
void walk_VirtualPassiveBase(void);
void test2_fix(void);
void test2_VPB(void);
void test3_fix(void);
void test3_VPB(void);
void imuMode(void);
void imuMode2(void);
void torqueCalibration(void);

#endif