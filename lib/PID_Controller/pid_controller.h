#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PIDController {
public:
    PIDController(float kp, float ki, float kd, float outputMin, float outputMax);
    float compute(float setpoint, float measured, float dt);

private:
    float Kp, Ki, Kd;
    float prevError, integral;
    float outputMin, outputMax;
    float prevDerivative;
};

// PID katsayıları burada tanımlanıyor
extern float PAngleRoll, IAngleRoll, DAngleRoll;
extern float PAnglePitch, IAnglePitch, DAnglePitch;

#endif
