#include "RCReceiver.h"

volatile unsigned long RCReceiver::pulseStart = 0;
volatile int RCReceiver::pulseWidth = 1000;

RCReceiver::RCReceiver(int pin) : pin(pin) {}

void RCReceiver::begin() {
    pinMode(pin, INPUT);
    attachInterrupt(digitalPinToInterrupt(pin), RCReceiver::handleInterrupt, CHANGE);
}

void IRAM_ATTR RCReceiver::handleInterrupt() {
    bool state = digitalRead(23); // Kanal pinin sabit olduğunu varsayıyoruz (pin 23)
    if (state) {
        pulseStart = micros();
    } else {
        unsigned long now = micros();
        int duration = now - pulseStart;
        // Anormal değerleri filtrele
        if (duration >= 800 && duration <= 2200) {
            pulseWidth = duration;
        }
    }
}

int RCReceiver::getRawPWM() const {
    return pulseWidth;
}

int RCReceiver::getNormalizedPWM() const {
    return constrain(pulseWidth, 1000, 2000);
}
