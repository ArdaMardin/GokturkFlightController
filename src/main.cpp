#include <Arduino.h>
#include "imu_sensor.h"
#include "motor_controller.h"
#include "pid_controller.h"
#include <WiFi.h>
#include <SPIFFS.h>
//#include <ESPAsyncWebServer.h> //webserver kütüphaneleri
//#include <ArduinoJson.h>  //webserver kütüphaneleri
#include "RCReceiver.h"

// const char* ssid = "Girme";      // Kendi WiFi SSID'n
// const char* password = "arda2001"; // Şifren

// AsyncWebServer server(80); //webserver

// ESC pinleri
#define ESC_PIN_1 18       // ESC1 için PWM çıkış pini
#define ESC_PIN_2 26       // ESC2 için PWM çıkış pini
#define ESC_PIN_3 19       // ESC3 için PWM çıkış pini
#define ESC_PIN_4 27       // ESC4 için PWM çıkış pini
#define RC_CHANNEL_PIN 23 //KUMANDA CHANNEL PİNİ


// IMU ve Motor Kontrolcü nesneleri
IMUSensor imuSensor;
MotorController motorController(ESC_PIN_1, ESC_PIN_2, ESC_PIN_3, ESC_PIN_4);

//kumanda nesnesi
RCReceiver rcInput(RC_CHANNEL_PIN);

// PID kontrolcülerini başlat
PIDController pidRoll(2.0, 0.5, 0.007, -400, 400);
PIDController pidPitch(2.0, 0.5, 0.007, -400, 400);


void setup() {
  Serial.begin(115200);  // Seri monitör başlatma
  Serial.println("Başlatılıyor...");
  rcInput.begin();
  delay(1000);


    // // WiFi başlat
    // WiFi.begin(ssid, password);
    // Serial.print("WiFi bağlanıyor");
    // while (WiFi.status() != WL_CONNECTED) {
    //   delay(500);
    //   Serial.print(".");
    // }
    // Serial.println("\nWiFi bağlı:");
    // Serial.println(WiFi.localIP());
  
    // // SPIFFS başlat
    // if (!SPIFFS.begin(true)) {
    //   Serial.println("SPIFFS başlatılamadı!");
    //   return;
    // }
  
    // // Web sunucusu: index.html
    // server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    //   request->send(SPIFFS, "/index.html", "text/html");
    // });
  
    // // IMU verisi endpoint
    // server.on("/data", HTTP_GET, [](AsyncWebServerRequest *request){
    //   StaticJsonDocument<200> doc;
    //   doc["roll"] = imuSensor.getRollKF();
    //   doc["pitch"] = imuSensor.getPitchKF();
    //   doc["yaw"] = imuSensor.getYawKF();
      
    //   String json;
    //   serializeJson(doc, json);
    //   request->send(200, "application/json", json);
    // });
  
    // server.begin();
  




 
  // Motorları başlat
  motorController.begin();
  motorController.arm();

  // IMU'yu başlat
  imuSensor.begin();
  Serial.println("Motorlar ve IMU başlatıldı!");
}

void loop() {
  //Serial.println("IMU update çalıştı");
  // unsigned long start = micros();   //imu süresi tutmak için aç 

  imuSensor.update();
  // unsigned long duration = micros() - start;   //imu süresi tutmak için aç 
  // Serial.print("update() süresi: ");   //imu süresi tutmak için aç 
  // Serial.print(duration);   //imu süresi tutmak için aç 
  // Serial.println(" µs");   //imu süresi tutmak için aç 

 //kumanda okunan pwm kodları 

  int kumandapwm = rcInput.getNormalizedPWM(); // bu değeri istediğin gibi kullanabilirsin

  Serial.print("RC PWM Çıkışı: ");
  Serial.println(kumandapwm);






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
  motorController.updateMotors(kumandapwm, rollPID, pitchPID);

  // Seri monitöre motor hızlarını yazdır
  // Serial.print("Roll: "); Serial.print(roll);
  // Serial.print(" | Pitch: "); Serial.print(pitch);
  // Serial.print(" | RollPID: "); Serial.print(rollPID);
  // Serial.print(" | PitchPID: "); Serial.println(pitchPID);
  Serial.print("motor hızları"); Serial.println();

  Serial.print("Motor 1 PWM: "); Serial.println(motorController.motor1PWM);
  Serial.print("Motor 2 PWM: "); Serial.println(motorController.motor2PWM);
  Serial.print("Motor 3 PWM: "); Serial.println(motorController.motor3PWM);
  Serial.print("Motor 4 PWM: "); Serial.println(motorController.motor4PWM);
  

    // 50ms aralıkla güncelle 
}
