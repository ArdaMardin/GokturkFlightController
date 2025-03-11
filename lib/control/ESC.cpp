#include "ESC.h"
#include <ESP32_Servo.h>

ESC::ESC(int motor1Pin, int motor2Pin, int motor3Pin, int motor4Pin) {
    motorPins[0] = motor1Pin;
    motorPins[1] = motor2Pin;
    motorPins[2] = motor3Pin;
    motorPins[3] = motor4Pin;
}

void ESC::initialize() {
    for (int i = 0; i < 4; i++) {
        motors[i].attach(motorPins[i]); // ESP32 Servo kütüphanesi ile motorları bağla
        motors[i].writeMicroseconds(minThrottle); // Motorları minimum hızda başlat
    }
    Serial.println("ESC Başlatıldı");
}

void ESC::setThrottle(float motor1, float motor2, float motor3, float motor4) {
    motor1 = constrain(motor1, minThrottle, maxThrottle);
    motor2 = constrain(motor2, minThrottle, maxThrottle);
    motor3 = constrain(motor3, minThrottle, maxThrottle);
    motor4 = constrain(motor4, minThrottle, maxThrottle);
    
    motors[0].writeMicroseconds(motor1);
    motors[1].writeMicroseconds(motor2);
    motors[2].writeMicroseconds(motor3);
    motors[3].writeMicroseconds(motor4);
}

void ESC::failsafe() {
    for (int i = 0; i < 4; i++) {
        motors[i].writeMicroseconds(minThrottle);
    }
    Serial.println("Failsafe aktif! Motorlar kapatıldı.");
}

void ESC::calibrate() {
    Serial.println("ESC Kalibrasyonu Başlatılıyor...");
    for (int i = 0; i < 4; i++) {
        motors[i].writeMicroseconds(maxThrottle);
    }
    delay(2000);
    for (int i = 0; i < 4; i++) {
        motors[i].writeMicroseconds(minThrottle);
    }
    Serial.println("ESC Kalibrasyonu Tamamlandı.");
}

void ESC::softStart(float targetThrottle, int duration) {
    float stepSize = (targetThrottle - minThrottle) / (duration / 10);
    float throttle = minThrottle;
    while (throttle < targetThrottle) {
        setThrottle(throttle, throttle, throttle, throttle);
        throttle += stepSize;
        delay(10);
    }
    setThrottle(targetThrottle, targetThrottle, targetThrottle, targetThrottle);
}
