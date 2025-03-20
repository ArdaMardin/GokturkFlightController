#include "imu_sensor.h"

IMUSensor::IMUSensor() : prevTime(0), t(0.004), comp_filter_gain(0.990), R(0.001), Q(0.01) {
    roll_cf = pitch_cf = 0;
    roll_kf = pitch_kf = 0;
    A << 1, -0.01,
         0,  1;
    x_roll << 0, 0;
    x_pitch << 0, 0;
    x_yaw << 0, 0;

    P = Matrix2f::Identity();
}

void IMUSensor::begin() {
    Wire.begin();
    Wire.setClock(400000);

    imu.Config(&Wire, bfs::Mpu9250::I2C_ADDR_PRIM);
    if (!imu.Begin()) {
        Serial.println("IMU başlatılamadı!");
        while (1);
    }
    imu.ConfigSrd(19);
}

void IMUSensor::update() {
    if (imu.Read()) {
        unsigned long currentTime = millis();
        dt = (currentTime - prevTime) / 1000.0;
        prevTime = currentTime;

        float ax = imu.accel_x_mps2();
        float ay = imu.accel_y_mps2();
        float az = imu.accel_z_mps2();
        float gx = imu.gyro_x_radps();
        float gy = imu.gyro_y_radps();
        float gz = imu.gyro_z_radps();  // Yaw için gyro Z verisi alınıyor.

        float roll_acc = atan2(-ay, sqrt(ax * ax + az * az)) * 180.0 / M_PI;
        float pitch_acc = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / M_PI;

        roll_cf = comp_filter_gain * (roll_cf + gx * t) + (1 - comp_filter_gain) * roll_acc;
        pitch_cf = comp_filter_gain * (pitch_cf + gy * t) + (1 - comp_filter_gain) * pitch_acc;
        yaw_cf += gz * dt;  // Yaw için Complementary Filter (Manyetometre kullanılmadığı için)



        kalmanFilter(x_roll, roll_acc);
        kalmanFilter(x_pitch, pitch_acc);
        kalmanFilter(x_yaw, gz * dt);

        roll_kf = x_roll(0);
        pitch_kf = x_pitch(0);
        yaw_kf = x_yaw(0);

    }
}

void IMUSensor::kalmanFilter(Vector2f &x, float z) {
    x = A * x;
    P = A * P * A.transpose() + Q * Matrix2f::Identity();
    K = P * Vector2f(1, 0) / (P(0, 0) + R);
    x += K * (z - x(0));
    P = (Matrix2f::Identity() - K * Vector2f(1, 0).transpose()) * P;
}

float IMUSensor::getRollCF() { return roll_cf; }
float IMUSensor::getPitchCF() { return pitch_cf; }
float IMUSensor::getRollKF() { return roll_kf; }
float IMUSensor::getPitchKF() { return pitch_kf; }
float IMUSensor::getYawKF() { return yaw_kf; }
float IMUSensor::getYawCF() { return yaw_cf; }


