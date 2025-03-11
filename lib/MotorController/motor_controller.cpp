#include "motor_controller.h"

MotorController::MotorController()
    : minPWM(1000), maxPWM(2000), currentThrottle(1000), targetThrottle(1000) {}

/// @brief Motorları başlatır
void MotorController::initialize() {
    motor1.attach(15); // Motor pinleri değiştirilecek
    motor2.attach(2);  // Motor pinleri değiştirilecek
    motor3.attach(4);  // Motor pinleri değiştirilecek
    motor4.attach(5);  // Motor pinleri değiştirilecek
}

/// @brief Motorları arm eder (ESC'leri başlatır)
void MotorController::MotorsArm(){
    motor1.writeMicroseconds(1000);
    motor2.writeMicroseconds(1000);
    motor3.writeMicroseconds(1000);
    motor4.writeMicroseconds(1000);
}

/// @brief Motorları durdurur
void MotorController::MotorsAllStop(){
    motor1.writeMicroseconds(minPWM);
    motor2.writeMicroseconds(minPWM);
    motor3.writeMicroseconds(minPWM);
    motor4.writeMicroseconds(minPWM);
}

/// @brief Motor hızlarını belirler
/// @param controlSignals PID çıkışı (x, y, z eksenlerindeki düzeltme sinyalleri)
/// @param throttle Genel throttle değeri (motor gücü)
void MotorController::setMotorSpeeds(Vector3f controlSignals, float throttle) {
    // **Throttle güncellenirken smooth geçiş yapılıyor**
    targetThrottle = map(throttle, 0, 100, minPWM, maxPWM);

    // Motor hızlarını belirle (Dönme yönlerine göre düzeltme yapılabilir)
    float speed1 = constrain(targetThrottle + controlSignals.x(), minPWM, maxPWM);
    float speed2 = constrain(targetThrottle + controlSignals.y(), minPWM, maxPWM);
    float speed3 = constrain(targetThrottle + controlSignals.z(), minPWM, maxPWM);
    float speed4 = constrain(targetThrottle + controlSignals.x(), minPWM, maxPWM);

    // Hızları güncelle (Ramping ile)
    motor1.writeMicroseconds(map(speed1, 0, 100, minPWM, maxPWM));
    motor2.writeMicroseconds(map(speed2, 0, 100, minPWM, maxPWM));
    motor3.writeMicroseconds(map(speed3, 0, 100, minPWM, maxPWM));
    motor4.writeMicroseconds(map(speed4, 0, 100, minPWM, maxPWM));
}

/// @brief Motor hızlarını yavaş yavaş değiştirerek ani değişiklikleri önler
void MotorController::updateESC() {
    // **Motor hızlarını smooth ramping ile güncelle**
    if (currentThrottle < targetThrottle) currentThrottle += 5;
    if (currentThrottle > targetThrottle) currentThrottle -= 5;

    motor1.writeMicroseconds(currentThrottle);
    motor2.writeMicroseconds(currentThrottle);
    motor3.writeMicroseconds(currentThrottle);
    motor4.writeMicroseconds(currentThrottle);
}

/// @brief Motorları acil durumda durdurur (Failsafe)
void MotorController::stopMotors() {
    motor1.writeMicroseconds(minPWM);
    motor2.writeMicroseconds(minPWM);
    motor3.writeMicroseconds(minPWM);
    motor4.writeMicroseconds(minPWM);
}
