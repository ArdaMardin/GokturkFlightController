#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ArduinoEigenDense.h>

using namespace Eigen;

class IMUSensor {
public:
    IMUSensor();
    void begin();
    void IRAM_ATTR update();
    float getRollCF() const;
    float getPitchCF() const;
    float getYawCF() const;
    float getRollKF() const;
    float getPitchKF() const;
    float getYawKF() const;
    float getDt() const;
    float getRollFiltered()  const { return roll_filtered; }
    float getPitchFiltered() const { return pitch_filtered; }
    //kalibre
    void calibrateIMU();

private:
    Adafruit_MPU6050 imu; 
    unsigned long prevTime;
    float dt;
    float t;
    float roll_cf, pitch_cf, yaw_cf;
    float roll_kf, pitch_kf, yaw_kf;
    float comp_filter_gain;
    //kalibre etmek için gerekli değişkiler

    float rollOffset = 0;
    float pitchOffset = 0;
    float rawRoll = 0;
    float rawPitch = 0;
    
    float roll_filtered  = 0;
    float pitch_filtered = 0;
    float vib_alpha;                      // artık constexpr değil
    static constexpr float vib_fc = 1.0f; // EMA kesim frekansı [Hz]
    static constexpr float vib_fs = 250.0f; // örnekleme hızı [Hz]

    // Notch filtresi için
    float z1_roll = 0, z2_roll = 0;
    float z1_pitch = 0, z2_pitch = 0;
    float b0, b1, b2, a1, a2;
    void initNotch();
    float applyNotchRoll(float x);
    float applyNotchPitch(float x);
    // Dinamik olarak heap'te bellek ayırma işlemi
    Matrix2f *A;
    Vector2f *x_roll, *x_pitch, *x_yaw;
    Matrix2f *P;
    Vector2f *K;
    float R, Q;

    void IRAM_ATTR kalmanFilter(Vector2f &x, float z);
    float IRAM_ATTR fastAtan2(float y, float x);
    //bool pingIMU();

    unsigned long loopCounter;
    unsigned long lastPrintTime;

};

#endif
