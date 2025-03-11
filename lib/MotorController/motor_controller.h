#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <ESP32Servo.h>
#include <ArduinoEigen.h>

using Eigen::Vector3f;

class MotorController {
private:
    Servo motor1, motor2, motor3, motor4;
    float minPWM, maxPWM;
    int currentThrottle;
    int targetThrottle;

public:
    MotorController();
    void initialize();
    void setMotorSpeeds(Vector3f controlSignals, float throttle);
    void updateESC();
    void stopMotors();
    void MotorsArm();
    void MotorsAllStop();
};

#endif // MOTOR_CONTROLLER_H
