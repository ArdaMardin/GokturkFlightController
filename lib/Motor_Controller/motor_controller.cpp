#include "motor_controller.h"
#include <Arduino.h>

MotorController::MotorController(int motor1Pin, int motor2Pin, int motor3Pin, int motor4Pin)
    : motor1(motor1Pin), motor2(motor2Pin), motor3(motor3Pin), motor4(motor4Pin) {}

void MotorController::begin() {
    // Motorları başlat
    motor1.begin();
    motor2.begin();
    motor3.begin();
    motor4.begin();
}

void MotorController::arm() {
    // Motorları arm et (ESC'yi arm etmek için min throttle)
    motor1.arm();
    motor2.arm();
    motor3.arm();
    motor4.arm();
}

void MotorController::updateMotors(int throttle, int rollPID, int pitchPID) {
    int baseSpeed = constrain(throttle, 1000, 2000);
    motor1PWM = baseSpeed + rollPID + pitchPID;
    motor2PWM = baseSpeed - rollPID + pitchPID;
    motor3PWM = baseSpeed - rollPID - pitchPID;
    motor4PWM = baseSpeed + rollPID - pitchPID;

    motor1.setSpeed(motor1PWM);
    motor2.setSpeed(motor2PWM);
    motor3.setSpeed(motor3PWM);
    motor4.setSpeed(motor4PWM);
}
