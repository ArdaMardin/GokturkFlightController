#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#include "imu.h"
#include "pid.h"
#include "motor_controller.h"

class FlightController {
private:
    Eigen::Vector3f targetAngles;  // Hedef Roll, Pitch, Yaw
    Eigen::Vector3f measuredAngles; // Gerçek zamanlı IMU verisi
    float throttle;  // Yükseklik için throttle (motor gücü)
    PIDController3D pid;
    MotorController motorController;
    unsigned long lastPIDUpdate;
    unsigned long lastESCUpdate;
    unsigned long lastValidSignal;
    bool failsafeActive;

public:
    FlightController();
    void initialize();
    void update();
    void setTargetAngles(float roll, float pitch, float yaw);
    void setThrottle(float value);
    void checkFailsafe();
};

#endif // FLIGHT_CONTROLLER_H
