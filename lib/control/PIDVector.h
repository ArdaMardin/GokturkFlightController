#ifndef PIDVECTOR_H
#define PIDVECTOR_H

#include <ArduinoEigenDense.h>

class PIDController3D {
    private:
        Eigen::Vector3f Kp, Ki, Kd;
        Eigen::Vector3f prevError, integral;
        Eigen::Vector3f outputMin, outputMax;

    public:
        PIDController3D(Eigen::Vector3f Kp, Eigen::Vector3f Ki, Eigen::Vector3f Kd, 
                        Eigen::Vector3f outputMin, Eigen::Vector3f outputMax);
        Eigen::Vector3f compute(Eigen::Vector3f setpoint, Eigen::Vector3f measured, double dt);
        void reset();
};

#endif // PIDVECTOR_H
