#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PIDController {
private:
    // PID katsayıları
    float Kp, Ki, Kd;
    
    // Hata ve entegral değerleri
    float prevError;
    float integral;
    
    // Çıkış sınırları
    float outputMin, outputMax;
    
    // Entegral sınırları
    float integralMin = -100.0;
    float integralMax = 100.0;
    
    // Ölçüm filtreleme için değişkenler
    float lastMeasurement;
    float filterAlpha;
    
    // Türev filtreleme için
    float prevDerivative = 0;
    
    // Hata ayıklama için bileşen değerleri
    float lastPTerm = 0;
    float lastITerm = 0;
    float lastDTerm = 0;

public:
    // Yapıcı fonksiyon
    PIDController(float kp, float ki, float kd, float outputMin, float outputMax);
    
    // Ana PID hesaplama fonksiyonu
    float compute(float setpoint, float measured, float dt);
    
    // PID parametrelerini ayarlama fonksiyonları
    void setTunings(float kp, float ki, float kd);
    void setOutputLimits(float min, float max);
    void setIntegralLimits(float min, float max);
    void setFilterAlpha(float alpha);
    
    // İntegral ve hata durumunu sıfırlama
    void resetIntegral();
    void reset();
    
    // Hata ayıklama ve durum bilgisi alma fonksiyonları
    float getP() const { return lastPTerm; }
    float getI() const { return lastITerm; }
    float getD() const { return lastDTerm; }
    float getIntegral() const { return integral; }
    float getLastError() const { return prevError; }
};

#endif // PID_CONTROLLER_H