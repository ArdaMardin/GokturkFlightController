#include "pid_controller.h"

// PID katsayıları
float PAngleRoll = 2.0, IAngleRoll = 0.5, DAngleRoll = 0.007;
float PAnglePitch = PAngleRoll, IAnglePitch = IAngleRoll, DAnglePitch = DAngleRoll;

PIDController::PIDController(float kp, float ki, float kd, float outputMin, float outputMax) 
    : Kp(kp), Ki(ki), Kd(kd), prevError(0), integral(0), outputMin(outputMin), outputMax(outputMax) {}

float PIDController::compute(float setpoint, float measured, float dt) {
    float error = setpoint - measured;

    // **Integral bileşeni (Iterm) güncellendi**
    integral += (error + prevError) * (dt / 2);

    // **Differential bileşeni (Dterm) güncellendi**
    float derivative = (error - prevError) / dt;

    // **Toplam PID Çıkışı**
    float output = Kp * error + Ki * integral + Kd * derivative;

    // **Çıkış Limitleme**
    if (output > outputMax) output = outputMax;
    if (output < outputMin) output = outputMin;

    // Önceki hatayı kaydet
    prevError = error;

    return output;
}
