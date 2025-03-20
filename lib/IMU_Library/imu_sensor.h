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
    void update();
    float getRollCF();
    float getPitchCF();
    float getRollKF();
    float getPitchKF();
    float getYawCF();
    float getYawKF();

private:
    bfs::Mpu9250 imu;
    unsigned long prevTime;
    float dt;
    float t;
    float roll_cf, pitch_cf;
    float roll_kf, pitch_kf;
    float comp_filter_gain;
    float yaw_cf, yaw_kf;


    Matrix2f A;
    Vector2f x_roll, x_pitch, x_yaw;
    Matrix2f P;
    Vector2f K;
    float R, Q;

    void kalmanFilter(Vector2f &x, float z);
};

#endif
