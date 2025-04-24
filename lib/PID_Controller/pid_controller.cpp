#include "pid_controller.h"
#include <cmath>
// PID katsayıları
float PAngleRoll, IAngleRoll , DAngleRoll;
float PAnglePitch = PAngleRoll, IAnglePitch = IAngleRoll, DAnglePitch = DAngleRoll;

PIDController::PIDController(float kp, float ki, float kd, float outputMin, float outputMax)
    : Kp(kp), Ki(ki), Kd(kd), prevError(0), integral(0), outputMin(outputMin), outputMax(outputMax) {}

      float PIDController::compute(float setpoint, float measured, double dt) {
        float error = setpoint - measured;
        if(abs(error) < 0.15) {
            integral *= 0.75; // Hata çok küçükse sıfırla
        }
        // Anti-windup için integral bileşenini sınırlayın
    
        // Anti-windup için önceki çıktıyı saklayın
        float previousOutput = Kp * error + Ki * integral + Kd * prevError;
        
        // Integral bileşeni güncellendi
        integral += (error + prevError) * (dt / 2);
    
        // Differential bileşeni güncellendi
        float rawDerivative = (error - prevError) / dt;
        float derivative = 0.9 * prevDerivative + 0.1 * rawDerivative;
        prevDerivative = derivative;
        
    
        // Toplam PID Çıkışı
        float output = Kp * error + Ki * integral - Kd * derivative;
    
        // Çıkış limitleme
        if (output > outputMax) {
            output = outputMax;
            // Anti-windup: çıkış limitlere ulaştığında integral değerini ayarla
            integral -= (output - outputMax) / Ki;
        }
        if (output < outputMin) {
            output = outputMin;
            // Anti-windup: çıkış limitlere ulaştığında integral değerini ayarla
            integral -= (output - outputMin) / Ki;
        }
        // Önceki hatayı kaydet
        prevError = error;
    
    
        return output;
    }
