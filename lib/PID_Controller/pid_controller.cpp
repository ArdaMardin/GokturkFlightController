#include "pid_controller.h"

// Dış değişkenler - bunlar sınıf dışında tanımlanmış olabilir veya başka bir dosyaya taşınabilir
float PAngleRoll = 2.0, IAngleRoll = 0.5, DAngleRoll = 0.007;
float PAnglePitch = PAngleRoll, IAnglePitch = IAngleRoll, DAnglePitch = DAngleRoll;

PIDController::PIDController(float kp, float ki, float kd, float outputMin, float outputMax) 
    : Kp(kp), Ki(ki), Kd(kd), 
      prevError(0), integral(0), 
      outputMin(outputMin), outputMax(outputMax), 
      lastMeasurement(0), filterAlpha(0.1) {}

float PIDController::compute(float setpoint, float measured, float dt) {
    // Ölçüm gürültüsünü azaltmak için düşük geçişli filtre uygula
    measured = (1 - filterAlpha) * lastMeasurement + filterAlpha * measured;
    lastMeasurement = measured;
    
    // Hata hesaplama
    float error = setpoint - measured;

    // Anti-windup koruması - integral değerinin sınırları aşmasını önler
    if ((integral > 0 && error > 0) || (integral < 0 && error < 0)) {
        // Integral bileşeni (trapezoid kuralı kullanarak daha doğru integral)
        integral += (error + prevError) * (dt / 2);
        
        // Integral sınırlaması
        if (integral > integralMax) integral = integralMax;
        if (integral < integralMin) integral = integralMin;
    }

    // Türev hesaplaması için gürültü filtreleme
    float derivative;
    if (dt > 0) {
        derivative = (error - prevError) / dt;
        // Türev gürültü filtreleme
        derivative = derivative * 0.8 + prevDerivative * 0.2;
        prevDerivative = derivative;
    } else {
        derivative = 0;
    }

    // PID Çıkışı hesaplama
    float pTerm = Kp * error;
    float iTerm = Ki * integral;
    float dTerm = Kd * derivative;
    
    // Toplam çıkış
    float output = pTerm + iTerm + dTerm;

    // Çıkış sınırlaması
    if (output > outputMax) output = outputMax;
    if (output < outputMin) output = outputMin;

    // Önceki hatayı kaydet
    prevError = error;

    // Debugging için bileşenleri sakla
    lastPTerm = pTerm;
    lastITerm = iTerm;
    lastDTerm = dTerm;

    return output;
}

// PID parametrelerini güncellemek için metot
void PIDController::setTunings(float kp, float ki, float kd) {
    // Negatif değerleri engelle
    Kp = kp >= 0 ? kp : 0;
    Ki = ki >= 0 ? ki : 0;
    Kd = kd >= 0 ? kd : 0;
}

// Çıkış sınırlarını ayarlamak için metot
void PIDController::setOutputLimits(float min, float max) {
    if (min < max) {
        outputMin = min;
        outputMax = max;
    }
}

// Integral sınırlarını ayarlamak için metot
void PIDController::setIntegralLimits(float min, float max) {
    if (min < max) {
        integralMin = min;
        integralMax = max;
        
        // Mevcut integral değerini yeni sınırlara göre ayarla
        if (integral > integralMax) integral = integralMax;
        if (integral < integralMin) integral = integralMin;
    }
}

// Ölçüm filtre katsayısını ayarlamak için metot
void PIDController::setFilterAlpha(float alpha) {
    if (alpha >= 0 && alpha <= 1) {
        filterAlpha = alpha;
    }
}

// Integral bileşenini sıfırlamak için metot
void PIDController::resetIntegral() {
    integral = 0;
}

// Tüm PID durumunu sıfırlamak için metot
void PIDController::reset() {
    integral = 0;
    prevError = 0;
    lastMeasurement = 0;
    prevDerivative = 0;
    lastPTerm = 0;
    lastITerm = 0;
    lastDTerm = 0;
}