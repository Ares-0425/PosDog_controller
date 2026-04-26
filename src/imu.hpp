#ifndef IMU_HPP
#define IMU_HPP

#include "esp_dsp.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS1.h>

extern Adafruit_LSM9DS1 lsm1;
extern Adafruit_LSM9DS1 lsm2;

#define IMU_1AG_ADDRESS 0x6A
#define IMU_1M_ADDRESS 0x1C
#define IMU_2AG_ADDRESS 0x6B
#define IMU_2M_ADDRESS 0x1E

class SimpleKalmanFilter {
private:
    float Q_angle, Q_bias, R_measure;
    float angle, bias, rate;
    float P[2][2];
public:
    SimpleKalmanFilter();
    float update(float newAngle, float newRate, float dt);
};

void setupSensor(void);
void imu_init(void);
void imu_update(void);
float imu_get_acc_x(void);
float imu_get_acc_y(void);
float imu_get_acc_z(void);
float imu_get_gyro_x(void);
float imu_get_gyro_y(void);
float imu_get_gyro_z(void);
void kalman_update(float *, float *, float *);
void kalman_update_2(float *kal_X1, float *kal_Y1, float *kal_Z1,
                     float *kal_X2, float *kal_Y2, float *kal_Z2, float dt);
float estimatePeriod(float theta, float omega); //コサインフィット
float PLL(float theta, float *omega_hat, float *phi_hat, int time_ms); //位相同期ループ
int stand_judge(float front, float leg_R, float leg_L);

#endif