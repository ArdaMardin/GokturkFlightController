#ifndef RCRECEIVER_H
#define RCRECEIVER_H

#include <Arduino.h>

class RCReceiver {
public:
    RCReceiver(int pin);
    void begin();
    int getRawPWM() const;         // Ölçülen gerçek PWM genişliği (us)
    int getNormalizedPWM() const;  // 1000–2000 arası normalize edilmiş değer

private:
    static void IRAM_ATTR handleInterrupt(); // Statik ISR
    static volatile unsigned long pulseStart;
    static volatile int pulseWidth;
    int pin;
};

#endif
