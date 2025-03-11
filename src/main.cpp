#include <Arduino.h>
#include "IMU.h"
#include "PIDVector.h"

// **IMU Sensörü Tanımla**
IMU imuSensor;

// **PID Kazançları (Roll, Pitch, Yaw)**
Eigen::Vector3f Kp(1.5, 1.5, 1.5);
Eigen::Vector3f Ki(0.01, 0.01, 0.01);
Eigen::Vector3f Kd(0.5, 0.5, 0.5);

// **PID Çıkış Limitleri**
Eigen::Vector3f outputMin(-200, -200, -200);
Eigen::Vector3f outputMax(200, 200, 200);

// **PID Kontrolcüsü**
PIDController3D pid(Kp, Ki, Kd, outputMin, outputMax);

// **İstenen Açı (Drone’un dengede olması için)**
Eigen::Vector3f setpoint(0.0, 0.0, 0.0);

// **Gerçek zamanlı zamanlama için değişkenler**
unsigned long previousTime = 0;
const int loopTime = 10; // 10ms zamanlama

// **Quaternion'dan Euler Açılara Dönüştürme Fonksiyonu**
void quaternionToEuler(Quaternionf q, float &roll, float &pitch, float &yaw) {
    float sinr = 2.0f * (q.w() * q.x() + q.y() * q.z());
    float cosr = 1.0f - 2.0f * (q.x() * q.x() + q.y() * q.y());
    roll = atan2(sinr, cosr) * 180.0f / M_PI;

    float sinp = 2.0f * (q.w() * q.y() - q.z() * q.x());
    pitch = (fabs(sinp) >= 1) ? copysign(90.0f, sinp) : asin(sinp) * 180.0f / M_PI;

    float siny = 2.0f * (q.w() * q.z() + q.x() * q.y());
    float cosy = 1.0f - 2.0f * (q.y() * q.y() + q.z() * q.z());
    yaw = atan2(siny, cosy) * 180.0f / M_PI;
}

void setup() {
    Serial.begin(115200);
    Serial.println("IMU & PID Test Başlıyor...");

    imuSensor.IMUInit();  // IMU başlat
}

void loop() {
    unsigned long currentTime = millis();
    double dt = (currentTime - previousTime) / 1000.0; // Milisaniyeyi saniyeye çevir

    if (currentTime - previousTime >= loopTime) {
        previousTime = currentTime;

        // **IMU verilerini al**
        imuSensor.update();
        Quaternionf q = imuSensor.getOrientation(); // IMU'dan yönelim bilgisini al

        // **Euler açılara dönüştür**
        float roll, pitch, yaw;
        quaternionToEuler(q, roll, pitch, yaw);
        Eigen::Vector3f measuredAngles(roll, pitch, yaw);

        // **PID hesaplamasını yap**
        Eigen::Vector3f pidOutput = pid.compute(setpoint, measuredAngles, dt);

        // **Çıkışları yazdır**
        Serial.print("IMU Açılar (Roll, Pitch, Yaw): ");
        Serial.print(measuredAngles.x()); Serial.print(", ");
        Serial.print(measuredAngles.y()); Serial.print(", ");
        Serial.println(measuredAngles.z());

        Serial.print("PID Çıkışı (Roll, Pitch, Yaw): ");
        Serial.print(pidOutput.x()); Serial.print(", ");
        Serial.print(pidOutput.y()); Serial.print(", ");
        Serial.println(pidOutput.z());

        Serial.println("-----------------------------");
    }
}
