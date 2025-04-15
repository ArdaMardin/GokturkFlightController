#ifndef FLOGGER_H
#define FLOGGER_H

#include <Arduino.h>

class FlightLogger {
public:
    FlightLogger();
    void begin();  // Seri haberleşme başlat
    void log(unsigned long timeMs,
             int throttle,
             float setpointRoll, float setpointPitch,
             float roll, float pitch,
             float rollPID, float pitchPID,
             int m1, int m2, int m3, int m4);

private:
    unsigned long lastLogTime;
    unsigned int logIntervalMs = 100;
};

#endif
