#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

#include <Arduino.h>
#include <Wire.h>
#include "mpu9250.h"
#include <ArduinoEigenDense.h>

using namespace Eigen;

class IMUSensor {
public:
    IMUSensor();
    void begin();
    void IRAM_ATTR update();
    float getRollCF() const;
    float getPitchCF() const;
    float getYawCF() const;
    float getRollKF() const;
    float getPitchKF() const;
    float getYawKF() const;

private:
    bfs::Mpu9250 imu;
    unsigned long prevTime;
    float dt;
    float t;
    float roll_cf, pitch_cf, yaw_cf;
    float roll_kf, pitch_kf, yaw_kf;
    float comp_filter_gain;

    // Dinamik olarak heap'te bellek ayırma işlemi
    Matrix2f *A;
    Vector2f *x_roll, *x_pitch, *x_yaw;
    Matrix2f *P;
    Vector2f *K;
    float R, Q;

    void IRAM_ATTR kalmanFilter(Vector2f &x, float z);
    float IRAM_ATTR fastAtan2(float y, float x);
    bool pingIMU();

    unsigned long loopCounter;
    unsigned long lastPrintTime;

};

#endif
