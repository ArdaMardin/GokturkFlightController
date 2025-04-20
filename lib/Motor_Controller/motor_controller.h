#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "motor.h"

class MotorController {
public:
    int motor1PWM, motor2PWM, motor3PWM, motor4PWM;
    MotorController(int motor1Pin, int motor2Pin, int motor3Pin, int motor4Pin);
    void begin(); 
    void arm();  // Motorları arm et
    void updateMotors(int throttle, int rollPID, int pitchPID);
    int lastM1 = 1000, lastM2 = 1000, lastM3 = 1000, lastM4 = 1000;
    const int SLEW_RATE = 10;     // µs döngü başına izin verilen max değişim
    const int DEADBAND  = 10;     // µs altındaki değişimleri ihmal et
    void stopAllMotors();


private:
    Motor motor1, motor2, motor3, motor4;
};

#endif
