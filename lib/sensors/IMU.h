#ifndef IMU_H
#define IMU_H

#include <ArduinoEigenDense.h>
#include <MPU9250.h>
#include <Wire.h>
#include "MadgwickAHRS.h"

using namespace Eigen;
class IMU{
    public:
        IMU();
        void IMUInit();
        void update();
        Quaternionf getQuaternion();
        Quaternionf getOrientation();
        Vector3f getAcceleration();
        Vector3f getGyroscope();
        Vector3f getMagnetometer();
        float getRoll();

    private:
        MPU9250 imu;
        Madgwick filter;
        Quaternionf orientation;
        Vector3f acc;
        Vector3f gyro;
        Vector3f mag;
};
#endif
