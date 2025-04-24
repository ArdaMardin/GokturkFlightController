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
    delay(1000);
    Serial.println("motor attach tamamlandı");
}

void MotorController::arm() {
    // Motorları arm et (ESC'yi arm etmek için min throttle)
    motor1.arm();
    motor2.arm();
    motor3.arm();
    motor4.arm();
    delay(5000);  
    Serial.println("Motor Arm tamamlandı!");
}

void MotorController::updateMotors(int throttle, int rollPID, int pitchPID) {
    const int IDLE_THR = 1000;
    const int MAX_THR  = 2000;

    // 1) Temel throttle
    int base = constrain(throttle, IDLE_THR, MAX_THR);

    // 2) Ham mix
    float mix1 = base + rollPID + pitchPID;
    float mix2 = base - rollPID + pitchPID;
    float mix3 = base - rollPID - pitchPID;
    float mix4 = base + rollPID - pitchPID;

    // 3) Scale (Betaflight/PX4 tarzı)
    float mx = fmaxf(fmaxf(mix1,mix2), fmaxf(mix3,mix4));
    float mn = fminf(fminf(mix1,mix2), fminf(mix3,mix4));

    float upO   = mx - MAX_THR;
    float downO = IDLE_THR - mn;
    float scale = 1.0f;
    if (upO   > 0) scale = (MAX_THR - base)/(mx - base);
    if (downO > 0) scale = fminf(scale, (base - IDLE_THR)/(base - mn));

    mix1 = base + (mix1 - base)*scale;
    mix2 = base + (mix2 - base)*scale;
    mix3 = base + (mix3 - base)*scale;
    mix4 = base + (mix4 - base)*scale;

    // 4) Deadband: küçük dalgalanmaları ortadan kaldır
    auto applyDeadband = [&](int cmd){
        if (abs(cmd - base) < DEADBAND) return base;
        return cmd;
    };
    int c1 = applyDeadband(int(mix1));
    int c2 = applyDeadband(int(mix2));
    int c3 = applyDeadband(int(mix3));
    int c4 = applyDeadband(int(mix4));

    // 5) Slew‑rate limiter: ani sıçramaları yumuşat
    auto slew = [&](int cmd, int last){
        return constrain(cmd, last - SLEW_RATE, last + SLEW_RATE);
    };
    c1 = slew(c1, lastM1);
    c2 = slew(c2, lastM2);
    c3 = slew(c3, lastM3);
    c4 = slew(c4, lastM4);

    lastM1 = c1;  lastM2 = c2;  lastM3 = c3;  lastM4 = c4;

    motor1PWM = c1;
    motor2PWM = c2;
    motor3PWM = c3;
    motor4PWM = c4;
    // 6) Son olarak ESC aralığında clamp ve gönder
    motor1.setSpeed(constrain(c1, IDLE_THR, MAX_THR));
    motor2.setSpeed(constrain(c2, IDLE_THR, MAX_THR));
    motor3.setSpeed(constrain(c3, IDLE_THR, MAX_THR));
    motor4.setSpeed(constrain(c4, IDLE_THR, MAX_THR));
    

    
    // int baseSpeed = constrain(throttle, 1000, 2000);
    // motor1PWM = baseSpeed+ rollPID + pitchPID;
    // motor2PWM = baseSpeed- rollPID + pitchPID;
    // motor3PWM = baseSpeed- rollPID - pitchPID;
    // motor4PWM = baseSpeed+ rollPID - pitchPID;

    // motor1.setSpeed(motor1PWM);
    // motor2.setSpeed(motor2PWM);
    // motor3.setSpeed(motor3PWM);
    // motor4.setSpeed(motor4PWM);
}

void MotorController::stopAllMotors() {
    motor1.setSpeed(1000);
    motor2.setSpeed(1000);
    motor3.setSpeed(1000);
    motor4.setSpeed(1000);
}