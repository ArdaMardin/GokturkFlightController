#include "flight_controller.h"

FlightController::FlightController()
    : pid(Eigen::Vector3f(1.0, 1.2, 1.1), 
          Eigen::Vector3f(0.1, 0.15, 0.12), 
          Eigen::Vector3f(0.05, 0.07, 0.06), 
          Eigen::Vector3f(-100, -100, -100), 
          Eigen::Vector3f(100, 100, 100)),
      throttle(50),  // Başlangıç throttle %50
      lastPIDUpdate(0),
      lastESCUpdate(0),
      lastValidSignal(0),
      failsafeActive(false)
{
    targetAngles = Eigen::Vector3f(0, 0, 0);
}

void FlightController::initialize() {
    initIMU();
    motorController.initialize();
}

void FlightController::update() {
    unsigned long now = millis();

    // 🚀 **Failsafe Kontrolü**
    checkFailsafe();

    // 🚀 **PID Güncelleme - 5ms (200Hz)**
    if (now - lastPIDUpdate >= 5) {
        lastPIDUpdate = now;
        measuredAngles = getIMUData();
        Eigen::Vector3f controlSignal = pid.compute(targetAngles, measuredAngles, 0.005);  // 5ms = 0.005s
        motorController.setMotorSpeeds(controlSignal, throttle);
    }

    // 🚀 **ESC Güncelleme - 5ms (200Hz)**
    if (now - lastESCUpdate >= 5) {
        lastESCUpdate = now;
        motorController.updateESC();  // Motorları belirli frekansta güncelle
    }
}

void FlightController::setTargetAngles(float roll, float pitch, float yaw) {
    targetAngles = Eigen::Vector3f(roll, pitch, yaw);
    lastValidSignal = millis();  // En son veri alındığını kaydet
}

void FlightController::setThrottle(float value) {
    throttle = constrain(value, 0, 100);  // %0 - %100 arasında throttle ayarla
    lastValidSignal = millis();
}

void FlightController::checkFailsafe() {
    if (millis() - lastValidSignal > 1000) {  // 1 saniye boyunca veri gelmezse
        failsafeActive = true;
        motorController.stopMotors();  // Motorları durdur
    } else {
        failsafeActive = false;
    }
}
