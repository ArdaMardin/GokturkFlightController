#include <Arduino.h>
#include "imu_sensor.h"
#include "motor_controller.h"
#include "pid_controller.h"

// ESC pinleri
#define ESC_PIN_1 18       // ESC1 için PWM çıkış pini
#define ESC_PIN_2 19       // ESC2 için PWM çıkış pini
#define ESC_PIN_3 21       // ESC3 için PWM çıkış pini
#define ESC_PIN_4 22       // ESC4 için PWM çıkış pini

// IMU ve Motor Kontrolcü nesneleri
IMUSensor imuSensor;
MotorController motorController(ESC_PIN_1, ESC_PIN_2, ESC_PIN_3, ESC_PIN_4);

// PID kontrolcülerini başlat
PIDController pidRoll(2.0, 0.5, 0.007, -400, 400);
PIDController pidPitch(2.0, 0.5, 0.007, -400, 400);


void setup() {
  Serial.begin(115200);  // Seri monitör başlatma
  Serial.println("Başlatılıyor...");

  // Motorları başlat
  motorController.begin();

  // IMU'yu başlat
  imuSensor.begin();
  Serial.println("Motorlar ve IMU başlatıldı!");
}

void loop() {
  // IMU verilerini güncelle
  Serial.println("IMU update çalıştı");
  unsigned long start = micros();

  imuSensor.update();
  unsigned long duration = micros() - start;





  // IMU'dan roll ve pitch açılarını al
  float roll = imuSensor.getRollKF();
  float pitch = imuSensor.getPitchKF();
  
  // Hedef açıları belirle (dengede durması için)
  float targetRoll = 0.0;
  float targetPitch = 0.0;

  // PID hesaplamalarını yap
  float rollPID = pidRoll.compute(targetRoll, roll, 0.004);  // 0.004, döngü süresi
  float pitchPID = pidPitch.compute(targetPitch, pitch, 0.004);

  // Motorlara PWM sinyali gönder
  motorController.updateMotors(1200, rollPID, pitchPID);

  // Seri monitöre motor hızlarını yazdır
  // Serial.print("Roll: "); Serial.print(roll);
  // Serial.print(" | Pitch: "); Serial.print(pitch);
  // Serial.print(" | RollPID: "); Serial.print(rollPID);
  // Serial.print(" | PitchPID: "); Serial.println(pitchPID);
  //Serial.print("motor hızları"); Serial.println();

  Serial.print("update() süresi: ");
  Serial.print(duration);
  Serial.println(" µs");

  // Serial.print("Motor 1 PWM: "); Serial.println(motorController.motor1PWM);
  // Serial.print("Motor 2 PWM: "); Serial.println(motorController.motor2PWM);
  // Serial.print("Motor 3 PWM: "); Serial.println(motorController.motor3PWM);
  // Serial.print("Motor 4 PWM: "); Serial.println(motorController.motor4PWM);
  

  delay(1000);  // 50ms aralıkla güncelle 
}
