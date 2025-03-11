#ifndef ESC_H
#define ESC_H

#include <Arduino.h>

Servo motors[4];

class ESC {
    public:
        ESC(int motor1Pin, int motor2Pin, int motor3Pin, int motor4Pin);
        void initialize();
        void setThrottle(float motor1, float motor2, float motor3, float motor4);
        void failsafe();   // Acil durum modu (Tüm motorları kapatır)
        void calibrate();  // ESC Kalibrasyonu
        void softStart(float targetThrottle, int duration);  // Yavaşça throttle artır

    private:
        int motorPins[4];
        const float minThrottle = 1000;  // Minimum ESC sinyali (us cinsinden)
        const float maxThrottle = 2000;  // Maksimum ESC sinyali (us cinsinden)
};

#endif // ESC_H
