#ifndef MOTOR_H
#define MOTOR_H

#include <ESP32Servo.h>  // Servo kütüphanesini ekle

class Motor {
public:
    Motor(int pin);  // Motor pini tanımla
    void begin(); 
    void arm();  // ESC'yi arm et
    void setSpeed(int newSpeed);
    int getSpeed();

private:
    int motorPin;
    Servo esc;  // Servo nesnesi (ESC'yi kontrol etmek için)
    int speed;
};

#endif
