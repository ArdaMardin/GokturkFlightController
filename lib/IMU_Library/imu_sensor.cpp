// #include "imu_sensor.h"

// IMUSensor::IMUSensor() : prevTime(0), t(0.004), comp_filter_gain(0.990), R(0.001), Q(0.01) {
//     roll_cf = pitch_cf = yaw_cf = 0;
//     roll_kf = pitch_kf = yaw_kf = 0;

//     // Dinamik bellek tahsisi
//     A = new Matrix2f();
//     x_roll = new Vector2f();
//     x_pitch = new Vector2f();
//     x_yaw = new Vector2f();
//     P = new Matrix2f();
//     K = new Vector2f();

//     *A << 1, -0.01,
//          0,  1;

//     *x_roll << 0, 0;
//     *x_pitch << 0, 0;
//     *x_yaw << 0, 0;

//     *P = Matrix2f::Identity();
// }

// bool IMUSensor::pingIMU() {
//     Wire.beginTransmission(bfs::Mpu9250::I2C_ADDR_PRIM);
//     return (Wire.endTransmission() == 0);
// }

// void IMUSensor::begin() {
//     Wire.begin();
//     Wire.setClock(400000);
//     Wire.setBufferSize(128);

//     if (!pingIMU()) {
//         Serial.println("IMU bulunamadı!");
//         while (1);
//     }

//     imu.Config(&Wire, bfs::Mpu9250::I2C_ADDR_PRIM);
//     if (!imu.Begin()) {
//         Serial.println("IMU başlatılamadı!");
//         while (1);
//     }

//     imu.ConfigSrd(9); // 100Hz
// }

// void IMUSensor::update() {
//     if (imu.Read()) {
//         unsigned long currentTime = micros();
//         dt = (currentTime - prevTime) / 1000000.0f;
//         prevTime = currentTime;

//         float ax = imu.accel_x_mps2();
//         float ay = imu.accel_y_mps2();
//         float az = imu.accel_z_mps2();
//         float gx = imu.gyro_x_radps();
//         float gy = imu.gyro_y_radps();
//         float gz = imu.gyro_z_radps();
//         //kalibre etme işlemi
//         //rawRoll = roll_cf;
//         //rawPitch = pitch_cf;


//         float ax_sq = ax * ax;
//         float ay_sq = ay * ay;
//         float az_sq = az * az;
        
//         float roll_denominator = sqrtf(ax_sq + az_sq);
//         float pitch_denominator = sqrtf(ay_sq + az_sq);
        
//         float roll_acc = fastAtan2(-ay, roll_denominator) * RAD_TO_DEG;
//         float pitch_acc = fastAtan2(-ax, pitch_denominator) * RAD_TO_DEG;

//         float gyro_roll_delta = gx * dt * RAD_TO_DEG;
//         float gyro_pitch_delta = gy * dt * RAD_TO_DEG;
//         float gyro_yaw_delta = gz * dt * RAD_TO_DEG;

//         float one_minus_gain = 1.0f - comp_filter_gain;
        
//         roll_cf = comp_filter_gain * (roll_cf + gyro_roll_delta) + one_minus_gain * roll_acc;
//         pitch_cf = comp_filter_gain * (pitch_cf + gyro_pitch_delta) + one_minus_gain * pitch_acc;
//         yaw_cf += gyro_yaw_delta;

//         kalmanFilter(*x_roll, roll_acc);
//         kalmanFilter(*x_pitch, pitch_acc);
//         kalmanFilter(*x_yaw, gyro_yaw_delta);
//         //roll ve pitch offset değerleri kalibre etmek için çıkarıldı
//         roll_kf = x_roll->coeff(0) - rollOffset;
//         pitch_kf = x_pitch->coeff(0) - pitchOffset;
//         yaw_kf = x_yaw->coeff(0);

//         rawRoll = x_roll->coeff(0);
//         rawPitch = x_pitch->coeff(0);
//     }


// }

// void IMUSensor::calibrateIMU() {
//     Serial.println("IMU Kalibrasyonu başlıyor...");
    
//     const int numSamples = 2000;
//     float rollSum = 0, pitchSum = 0;

//     // Zaman uyumlu örnekleme
//     for (int i = 0; i < numSamples; i++) {
//         update();  // IMU verilerini oku ve filtrele
//         rollSum += rawRoll;
//         pitchSum += rawPitch;
//         delay(2);
//     }

//     rollOffset = rollSum / numSamples;
//     pitchOffset = pitchSum / numSamples;

//     Serial.print("Kalibrasyon tamamlandı. Offset değerleri: Roll=");
//     Serial.print(rollOffset);
//     Serial.print(", Pitch=");
//     Serial.println(pitchOffset);
// }




// void IMUSensor::kalmanFilter(Vector2f &x, float z) {
//     static Matrix2f I = Matrix2f::Identity();
//     static Vector2f H(1, 0);
    
//     float temp_x0 = A->coeff(0, 0) * x.coeff(0) + A->coeff(0, 1) * x.coeff(1);
//     float temp_x1 = A->coeff(1, 0) * x.coeff(0) + A->coeff(1, 1) * x.coeff(1);
//     x.coeffRef(0) = temp_x0;
//     x.coeffRef(1) = temp_x1;
    
//     float temp_p00 = A->coeff(0, 0) * P->coeff(0, 0) + A->coeff(0, 1) * P->coeff(1, 0);
//     float temp_p01 = A->coeff(0, 0) * P->coeff(0, 1) + A->coeff(0, 1) * P->coeff(1, 1);
//     float temp_p10 = A->coeff(1, 0) * P->coeff(0, 0) + A->coeff(1, 1) * P->coeff(1, 0);
//     float temp_p11 = A->coeff(1, 0) * P->coeff(0, 1) + A->coeff(1, 1) * P->coeff(1, 1);
    
//     P->coeffRef(0, 0) = temp_p00 * A->coeff(0, 0) + temp_p01 * A->coeff(0, 1) + Q;
//     P->coeffRef(0, 1) = temp_p00 * A->coeff(1, 0) + temp_p01 * A->coeff(1, 1);
//     P->coeffRef(1, 0) = temp_p10 * A->coeff(0, 0) + temp_p11 * A->coeff(0, 1);
//     P->coeffRef(1, 1) = temp_p10 * A->coeff(1, 0) + temp_p11 * A->coeff(1, 1) + Q;
    
//     float denominator = P->coeff(0, 0) + R;
//     K->coeffRef(0) = P->coeff(0, 0) / denominator;
//     K->coeffRef(1) = P->coeff(1, 0) / denominator;
    
//     float innovation = z - x.coeff(0);
//     x.coeffRef(0) += K->coeff(0) * innovation;
//     x.coeffRef(1) += K->coeff(1) * innovation;
    
//     P->coeffRef(0, 0) = (1 - K->coeff(0)) * P->coeff(0, 0);
//     P->coeffRef(0, 1) = (1 - K->coeff(0)) * P->coeff(0, 1);
//     P->coeffRef(1, 0) = -K->coeff(1) * P->coeff(0, 0) + P->coeff(1, 0);
//     P->coeffRef(1, 1) = -K->coeff(1) * P->coeff(0, 1) + P->coeff(1, 1);
// }

// float IMUSensor::fastAtan2(float y, float x) {
//     return atan2f(y, x);
// }

// float IMUSensor::getRollKF() const {
//     return roll_kf;
// }

// float IMUSensor::getPitchKF() const {
//     return pitch_kf;
// }

// float IMUSensor::getYawKF() const {
//     return yaw_kf;
// }

// float IMUSensor::getRollCF() const {
//     return roll_cf;
// }

// float IMUSensor::getPitchCF() const {
//     return pitch_cf;
// }

// float IMUSensor::getYawCF() const {
//     return yaw_cf;
// }

// float IMUSensor::getDt() const {
//     return dt;
//   }