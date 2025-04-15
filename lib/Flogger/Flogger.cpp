#include "flogger.h"

FlightLogger::FlightLogger() : lastLogTime(0) {}

void FlightLogger::begin() {
    Serial.begin(115200);  // veya SD.begin() yapılabilir
}

void FlightLogger::log(unsigned long timeMs,
                       int throttle,
                       float setpointRoll, float setpointPitch,
                       float roll, float pitch,
                       float rollPID, float pitchPID,
                       int m1, int m2, int m3, int m4) {
    if (timeMs - lastLogTime > logIntervalMs) {
        lastLogTime = timeMs;
        Serial.print("T:"); Serial.print(timeMs);
        Serial.print(", Thr:"); Serial.print(throttle);
        Serial.print(", SetRoll:"); Serial.print(setpointRoll);
        Serial.print(", Roll:"); Serial.print(roll);
        Serial.print(", PidR:"); Serial.print(rollPID);
        Serial.print(", SetPitch:"); Serial.print(setpointPitch);
        Serial.print(", Pitch:"); Serial.print(pitch);
        Serial.print(", PidP:"); Serial.print(pitchPID);
        Serial.print(", M1:"); Serial.print(m1);
        Serial.print(", M2:"); Serial.print(m2);
        Serial.print(", M3:"); Serial.print(m3);
        Serial.print(", M4:"); Serial.println(m4);
    }
}
