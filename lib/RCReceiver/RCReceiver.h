#ifndef RCRECEIVER_H
#define RCRECEIVER_H

#include <Arduino.h>

class RCReceiver {
private:
    int channelPin;

public:
    RCReceiver(int pin);
    void begin();
    int readRawPWM();         // Gerçek gelen pulse (örneğin: 1472us)
    int getNormalizedPWM();   // ESC benzeri normalize edilmiş çıkış (1000-2000 arası)
};

#endif
