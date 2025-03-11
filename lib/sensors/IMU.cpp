#include "IMU.h"
#include <cmath>

IMU::IMU() : imu(Wire, 0x68) {}

void IMU::IMUInit() {
    Wire.begin();
    if (imu.begin() != true) {
        Serial.println("MPU9250 bağlantı hatası!");
        while (1);
    }
    filter.begin(200);// Madgwick filtresini 200 Hz güncelleme ile başlat
    Serial.println("MPU9250 başlatıldı.");
}

void IMU::update() {
    imu.readSensor();
    
    acc = Vector3f(imu.getAccelX_mss(), imu.getAccelY_mss(), imu.getAccelZ_mss());
    gyro = Vector3f(imu.getGyroX_rads(), imu.getGyroY_rads(), imu.getGyroZ_rads());
    mag = Vector3f(imu.getMagX_uT(), imu.getMagY_uT(), imu.getMagZ_uT());
    
    // Madgwick filtresi ile Euler açılarını hesapla
    filter.updateIMU(gyro.x(), gyro.y(), gyro.z(), acc.x(), acc.y(), acc.z());

    float roll = filter.getRoll() * M_PI / 180.0;
    float pitch = filter.getPitch() * M_PI / 180.0;
    float yaw = filter.getYaw() * M_PI / 180.0;

    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);
    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);

    orientation = Quaternionf(
        cr * cp * cy + sr * sp * sy, // w
        sr * cp * cy - cr * sp * sy, // x
        cr * sp * cy + sr * cp * sy, // y
        cr * cp * sy - sr * sp * cy  // z
    );
}

Quaternionf IMU::getOrientation() {
    return orientation;
}

Vector3f IMU::getAcceleration() {
    return acc;
}

Vector3f IMU::getGyroscope() {
    return gyro;
}

Vector3f IMU::getMagnetometer() {
    return mag;
}


