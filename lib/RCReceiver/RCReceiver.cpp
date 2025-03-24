#include "RCReceiver.h"

RCReceiver::RCReceiver(int pin) : channelPin(pin) {}

void RCReceiver::begin() {
    pinMode(channelPin, INPUT);
}

int RCReceiver::readRawPWM() {
    return pulseIn(channelPin, HIGH, 30000); // 30ms timeout
}

int RCReceiver::getNormalizedPWM() {
    int raw = readRawPWM();
    if (raw < 100) return 1000; // sinyal yoksa default
    return constrain(raw, 1000, 2000); // ESC mantığında normalize
}
