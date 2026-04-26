#include <Arduino.h>
#include <M5Unified.h>
#include "imu.hpp"

Adafruit_LSM9DS1 lsm1(&Wire1);
Adafruit_LSM9DS1 lsm2(&Wire1);

void setupSensor() {
  // 1.) Set the accelerometer range
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G, lsm.LSM9DS1_ACCELDATARATE_10HZ);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G, lsm.LSM9DS1_ACCELDATARATE_119HZ);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G, lsm.LSM9DS1_ACCELDATARATE_476HZ);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G, lsm.LSM9DS1_ACCELDATARATE_952HZ);
  lsm1.setupAccel(lsm1.LSM9DS1_ACCELRANGE_2G, lsm1.LSM9DS1_ACCELDATARATE_476HZ);
  lsm2.setupAccel(lsm2.LSM9DS1_ACCELRANGE_2G, lsm2.LSM9DS1_ACCELDATARATE_476HZ);
  
  // 2.) Set the magnetometer sensitivity
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_12GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);
  lsm1.setupMag(lsm1.LSM9DS1_MAGGAIN_4GAUSS);
  lsm2.setupMag(lsm2.LSM9DS1_MAGGAIN_4GAUSS);

  // 3.) Setup the gyroscope
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);
  lsm1.setupGyro(lsm1.LSM9DS1_GYROSCALE_500DPS);
  lsm2.setupGyro(lsm2.LSM9DS1_GYROSCALE_500DPS);
}

// 定数
const float g = 9.80665;  // 重力加速度[m/s^2]

// 入力パラメータ
const float R = 0.40;       // 円弧半径[m]
const float d = R - 0.226;  // 中心から重心までの距離[m]

// カルマンフィルタクラス
SimpleKalmanFilter::SimpleKalmanFilter() { // コンストラクタ
  Q_angle = 0.001f; // プロセスノイズの分散（角度）
  Q_bias = 0.003f; // プロセスノイズの分散（バイアス）
  R_measure = 0.03f; // 観測ノイズの分散
  angle = 0.0f;
  bias = 0.0f;
  P[0][0] = P[0][1] = P[1][0] = P[1][1] = 0.0f;
}

float SimpleKalmanFilter::update(float newAngle, float newRate, float dt) {
  // 予測ステップ
  rate = newRate - bias;
  angle += dt * rate;

  P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_bias * dt;

  // 更新ステップ
  float S = P[0][0] + R_measure;
  float K[2] = {P[0][0] / S, P[1][0] / S};

  float y = newAngle - angle;
  angle += K[0] * y;
  bias  += K[1] * y;

  float P00_temp = P[0][0], P01_temp = P[0][1];
  P[0][0] -= K[0] * P00_temp;
  P[0][1] -= K[0] * P01_temp;
  P[1][0] -= K[1] * P00_temp;
  P[1][1] -= K[1] * P01_temp;

  return angle;
}

SimpleKalmanFilter kalmanPitch;
SimpleKalmanFilter kalmanRoll;
SimpleKalmanFilter kalmanYaw;

// メインループ用変数
float accX, accY, accZ;
float gyroX, gyroY, gyroZ;
float rollAcc,pitchAcc,yawAcc;
unsigned long lastTime;

void kalman_update(float *kalX, float *kalY, float *kalZ){
  // センサ取得
  M5.Imu.getAccelData(&accX, &accY, &accZ);
  M5.Imu.getGyroData(&gyroX, &gyroY, &gyroZ);

  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0f;
  lastTime = now;

  pitchAcc = atan2(-accY, sqrt(accX*accX + accZ*accZ)) * 180.0 / PI;
  rollAcc  = atan2(-accX, sqrt(accY*accY + accZ*accZ)) * 180.0 / PI;

  // カルマンフィルタで角度更新
  *kalX = kalmanPitch.update(pitchAcc, gyroX, dt);
  *kalY = kalmanRoll.update(rollAcc, gyroY, dt);
  *kalZ = kalmanYaw.update(0.0, gyroZ, dt);
}

SimpleKalmanFilter kalman_Pitch1;
SimpleKalmanFilter kalman_Pitch2;
SimpleKalmanFilter kalman_Roll1;
SimpleKalmanFilter kalman_Roll2;
SimpleKalmanFilter kalman_Yaw1;
SimpleKalmanFilter kalman_Yaw2;
void kalman_update_2(float *kal_X1, float *kal_Y1, float *kal_Z1,
                     float *kal_X2, float *kal_Y2, float *kal_Z2, float dt) {
  sensors_event_t a1, m1, g1, temp1;
  sensors_event_t a2, m2, g2, temp2; 
  float pitchAcc1, pitchAcc2, rollAcc1, rollAcc2;

  lsm1.getEvent(&a1, &m1, &g1, &temp1);
  lsm2.getEvent(&a2, &m2, &g2, &temp2);
  rollAcc1 = atan2(-a1.acceleration.y, sqrt(a1.acceleration.x * a1.acceleration.x + a1.acceleration.z * a1.acceleration.z)) * RAD_TO_DEG;
  rollAcc2 = atan2(-a2.acceleration.y, sqrt(a2.acceleration.x * a2.acceleration.x + a2.acceleration.z * a2.acceleration.z)) * RAD_TO_DEG;
  pitchAcc1 = atan2(-a1.acceleration.x, sqrt(a1.acceleration.y * a1.acceleration.y + a1.acceleration.z * a1.acceleration.z)) * RAD_TO_DEG;
  pitchAcc2 = atan2(-a2.acceleration.x, sqrt(a2.acceleration.y * a2.acceleration.y + a2.acceleration.z * a2.acceleration.z)) * RAD_TO_DEG;

  *kal_X1 = kalman_Roll1.update(rollAcc1, g1.gyro.x * RAD_TO_DEG, dt);
  *kal_X2 = kalman_Roll2.update(rollAcc2, g2.gyro.x * RAD_TO_DEG, dt);
  *kal_Y1 = kalman_Pitch1.update(pitchAcc1, -g1.gyro.y * RAD_TO_DEG, dt);
  *kal_Y2 = kalman_Pitch2.update(pitchAcc2, -g2.gyro.y * RAD_TO_DEG, dt);
  *kal_Z1 = kalman_Yaw1.update(0.0, g1.gyro.z * RAD_TO_DEG, dt);
  *kal_Z2 = kalman_Yaw2.update(0.0, g2.gyro.z * RAD_TO_DEG, dt);
}

/* --- コサインフィット --- */
float estimatePeriod(float theta, float omega) {
    // 有効長
    float rho = (R * R) / d;

    // エネルギー
    float E = 0.5f * rho * rho * omega * omega + g * rho * (1.0f - cos(theta));

    // cos(theta_max)を計算
    float cos_theta_max = 1.0f - (E / (g * rho));
    if (cos_theta_max < -1.0f) cos_theta_max = -1.0f;
    if (cos_theta_max > 1.0f) cos_theta_max = 1.0f;

    // 最大振幅
    float theta_max = acos(cos_theta_max);

    // 周期（補正式）
    float T = 2.0f * PI * sqrt(rho / g) * (1.0f + (theta_max * theta_max) / 16.0f);

    // 周波数
    return 1.0f / T; // [Hz]
}

/* --- 位相同期ループ --- */
// 位相同期ループ用変数
// float phi_hat = 0.0f;          // 推定位相
// float omega_hat = 0.0f;        // 推定周波数
const float Kp_PLL = 19.2f;       // 位相検出器比例
const float Ki_PLL = 64.0f;     // 位相検出器積分

float PLL(float theta, float *omega_hat, float *phi_hat, int time_ms) {
    // 位相検出器
    float theta_rad = radians(theta);
    float phase_error = theta_rad * cos(*phi_hat);   // 位相誤差

    // 位相更新
    *omega_hat += Ki_PLL * phase_error * (time_ms / 1000.0f);
    *phi_hat += (*omega_hat + Kp_PLL * phase_error) * (time_ms / 1000.0f);

    // 位相ラップ
    while(*phi_hat > PI) *phi_hat -= 2.0f*PI;
    while(*phi_hat < -PI) *phi_hat += 2.0f*PI;

    float fleq_hat = *omega_hat / (2.0f * PI);

    return abs(fleq_hat);
}

int stand_judge(float front, float leg_R, float leg_L){ // 支持脚判定
  // Rのとき1,Lのとき2,わからないとき0を返す
  // frontは時計回りを正、legは進行方向を正とする
  // 単位はどちらもrad

  float legR_length,legL_length; // 脚の実質長さ
  float judge_rad; // 接地するとされる角度
  float delta_rad = radians(3.0); // 接地誤差
  
  if(leg_R <= radians(7.8) && leg_R >= radians(-7.8)){
    if(leg_R > 0.0){
      legR_length = -19.83*leg_R + 331.2;
    }else{
      legR_length = 45.54*leg_R + 331.2;
    }
  }else{
    if(leg_R > 0.0){
      legR_length = 328.5*cos(leg_R - radians(7.8));
    }else{
      legR_length = 325.0*cos(leg_R + radians(7.8));
    }
  }
  if(leg_L <= radians(7.8) && leg_L >= radians(-7.8)){
    if(leg_L > 0.0){
      legL_length = -19.83*leg_L + 331.2;
    }else{
      legL_length = 45.54*leg_L + 331.2;
    }
  }else{
    if(leg_L > 0.0){
      legL_length = 328.5*cos(leg_L - radians(7.8));
    }else{
      legL_length = 325.0*cos(leg_L + radians(7.8));
    }
  }

  judge_rad = atan((legR_length-legR_length)/66.8);
  if(isfinite(judge_rad) == false){
    return 0;
  }

  if(front >= judge_rad + delta_rad){
    return 1;
  }else if(front <= judge_rad - delta_rad){
    return 2;
  }else{
    return 0;
  }

  return 0;
}