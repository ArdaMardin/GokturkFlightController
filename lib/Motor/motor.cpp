#include "motor.h"
#include <Arduino.h>

Motor::Motor(int pin) : motorPin(pin), speed(1000) {}

void Motor::begin() {
    esc.attach(motorPin, 1000, 2000);  // ESC'yi bağla ve sinyal aralığını 1000-2000 µs olarak ayarla
    esc.writeMicroseconds(1000);  // ESC'yi arm etmek için minimum sinyal gönder (1000 µs)
    delay(5000);  // ESC'nin arm olmasını bekle (5 saniye)
    Serial.println("begin kısmındaki motor arm edildi");
}

void Motor::arm() {
    // ESC'yi arm etmek için minimum sinyal gönder
    esc.writeMicroseconds(1000);  // Min throttle (1000 µs) gönder
    delay(5000);  // ESC'nin arm olmasını bekle (5 saniye)
    Serial.println("Motor Arm tamamlandı!");
}

void Motor::setSpeed(int newSpeed) {
    speed = constrain(newSpeed, 1000, 2000);  // Hız sınırları
    esc.writeMicroseconds(speed);  // ESC'ye PWM sinyali gönder
}

int Motor::getSpeed() {
    return speed;
}
