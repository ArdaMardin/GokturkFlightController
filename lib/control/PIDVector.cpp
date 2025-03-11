#include "PIDVector.h"

PIDController3D::PIDController3D(Eigen::Vector3f Kp, Eigen::Vector3f Ki, Eigen::Vector3f Kd, 
                                 Eigen::Vector3f outputMin, Eigen::Vector3f outputMax)
    : Kp(Kp), Ki(Ki), Kd(Kd), outputMin(outputMin), outputMax(outputMax) {
    prevError = Eigen::Vector3f::Zero();
    integral = Eigen::Vector3f::Zero();
}

Eigen::Vector3f PIDController3D::compute(Eigen::Vector3f setpoint, Eigen::Vector3f measured, double dt) {
    Eigen::Vector3f error = setpoint - measured;
    integral += error * dt;
    Eigen::Vector3f derivative = (error - prevError) / dt;
    Eigen::Vector3f output = (Kp.array() * error.array()).matrix() +
                             (Ki.array() * integral.array()).matrix() +
                             (Kd.array() * derivative.array()).matrix();

    // Çıkışları sınırla
    output = output.cwiseMax(outputMin).cwiseMin(outputMax);

    prevError = error;
    return output;
}

void PIDController3D::reset() {
    prevError = Eigen::Vector3f::Zero();
    integral = Eigen::Vector3f::Zero();
}