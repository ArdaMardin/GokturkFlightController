#include "imu_sensor6050.h"

IMUSensor::IMUSensor() : prevTime(0), t(0.004), comp_filter_gain(0.990), R(0.005), Q(0.01) {
    roll_cf = pitch_cf = yaw_cf = 0;
    roll_kf = pitch_kf = yaw_kf = 0;

    // Dinamik bellek tahsisi
    A = new Matrix2f();
    x_roll = new Vector2f();
    x_pitch = new Vector2f();
    x_yaw = new Vector2f();
    P = new Matrix2f();
    K = new Vector2f();

    *A << 1, -0.01,
         0,  1;

    *x_roll << 0, 0;
    *x_pitch << 0, 0;
    *x_yaw << 0, 0;

    *P = Matrix2f::Identity();
}

 //bool IMUSensor::pingIMU() {
//     Wire.beginTransmission(bfs::Mpu9250::I2C_ADDR_PRIM);
//     return (Wire.endTransmission() == 0);
// }
#define MPU6050_ADDRESS 0x68
void IMUSensor::begin() {
    // I2C başlat ve hızını 400 kHz yap
    Wire.begin();
    Wire.setClock(400000);

    // MPU6050 başlat
    if (!imu.begin()) {
      Serial.println("MPU6050 bulunamadı!");
      while (1) delay(10);
    }

    // Ölçeklendirme
    imu.setAccelerometerRange(MPU6050_RANGE_4_G);
    imu.setGyroRange(MPU6050_RANGE_500_DEG);

    // DLPF band genişliğini 260 Hz yap (250 Hz örnekleme için uygundur)
    imu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    // SMPLRT_DIV = 3 → 1000 Hz / (1 + 3) = 250 Hz örnekleme
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(0x19);    // SMPLRT_DIV register adresi
    Wire.write(3);       // divider değeri
    Wire.endTransmission();

    Serial.println("MPU6050 başlatıldı: I2C=400 kHz, 250 Hz örnekleme, DLPF=260 Hz");
}



void IMUSensor::update() {
    // sensör olaylarını oku
    sensors_event_t accelEvent, gyroEvent, tempEvent;
    imu.getEvent(&accelEvent, &gyroEvent, &tempEvent);
  
    unsigned long currentTime = micros();
    dt = (currentTime - prevTime) / 1e6f;
    prevTime = currentTime;
  
    // m/s² ve rad/s cinsinden değerler
    float ax = accelEvent.acceleration.x;
    float ay = accelEvent.acceleration.y;
    float az = accelEvent.acceleration.z;
    // Adafruit kütüphane gyro değeri deg/s cinsinden; rad/s’e çevir
    float gx = gyroEvent.gyro.x * DEG_TO_RAD;
    float gy = gyroEvent.gyro.y * DEG_TO_RAD;
    float gz = gyroEvent.gyro.z * DEG_TO_RAD;
  
    // (geri kalan hesaplamalar aynı kalabilir)
    float ax_sq = ax * ax, ay_sq = ay * ay, az_sq = az * az;
    float roll_den = sqrtf(ax_sq + az_sq);
    float pitch_den = sqrtf(ay_sq + az_sq);
    float roll_acc = fastAtan2(-ay, roll_den) * RAD_TO_DEG;
    float pitch_acc = fastAtan2(-ax, pitch_den) * RAD_TO_DEG;
  
    float gyro_roll_delta  = gx * dt * RAD_TO_DEG;
    float gyro_pitch_delta = gy * dt * RAD_TO_DEG;
    float gyro_yaw_delta   = gz * dt * RAD_TO_DEG;
  
    float one_minus = 1.0f - comp_filter_gain;
    roll_cf  = comp_filter_gain * (roll_cf  + gyro_roll_delta)  + one_minus * roll_acc;
    pitch_cf = comp_filter_gain * (pitch_cf + gyro_pitch_delta) + one_minus * pitch_acc;
    yaw_cf  += gyro_yaw_delta;
  
    // Kalman filtresi
    kalmanFilter(*x_roll,  roll_acc);
    kalmanFilter(*x_pitch, pitch_acc);
    kalmanFilter(*x_yaw,   gyro_yaw_delta);
  
    roll_kf  = x_roll->coeff(0)  - rollOffset;
    pitch_kf = x_pitch->coeff(0) - pitchOffset;
    yaw_kf   = x_yaw->coeff(0);
  
    rawRoll  = x_roll->coeff(0);
    rawPitch = x_pitch->coeff(0);
  }
  

void IMUSensor::calibrateIMU() {
    Serial.println("IMU Kalibrasyonu başlıyor...");
    
    const int numSamples = 2000;
    float rollSum = 0, pitchSum = 0;

    // Zaman uyumlu örnekleme
    for (int i = 0; i < numSamples; i++) {
        update();  // IMU verilerini oku ve filtrele
        rollSum += rawRoll;
        pitchSum += rawPitch;
        delay(2);
    }

    rollOffset = rollSum / numSamples;
    pitchOffset = pitchSum / numSamples;

    Serial.print("Kalibrasyon tamamlandı. Offset değerleri: Roll=");
    Serial.print(rollOffset);
    Serial.print(", Pitch=");
    Serial.println(pitchOffset);
}




void IMUSensor::kalmanFilter(Vector2f &x, float z) {
    static Matrix2f I = Matrix2f::Identity();
    static Vector2f H(1, 0);
    
    float temp_x0 = A->coeff(0, 0) * x.coeff(0) + A->coeff(0, 1) * x.coeff(1);
    float temp_x1 = A->coeff(1, 0) * x.coeff(0) + A->coeff(1, 1) * x.coeff(1);
    x.coeffRef(0) = temp_x0;
    x.coeffRef(1) = temp_x1;
    
    float temp_p00 = A->coeff(0, 0) * P->coeff(0, 0) + A->coeff(0, 1) * P->coeff(1, 0);
    float temp_p01 = A->coeff(0, 0) * P->coeff(0, 1) + A->coeff(0, 1) * P->coeff(1, 1);
    float temp_p10 = A->coeff(1, 0) * P->coeff(0, 0) + A->coeff(1, 1) * P->coeff(1, 0);
    float temp_p11 = A->coeff(1, 0) * P->coeff(0, 1) + A->coeff(1, 1) * P->coeff(1, 1);
    
    P->coeffRef(0, 0) = temp_p00 * A->coeff(0, 0) + temp_p01 * A->coeff(0, 1) + Q;
    P->coeffRef(0, 1) = temp_p00 * A->coeff(1, 0) + temp_p01 * A->coeff(1, 1);
    P->coeffRef(1, 0) = temp_p10 * A->coeff(0, 0) + temp_p11 * A->coeff(0, 1);
    P->coeffRef(1, 1) = temp_p10 * A->coeff(1, 0) + temp_p11 * A->coeff(1, 1) + Q;
    
    float denominator = P->coeff(0, 0) + R;
    K->coeffRef(0) = P->coeff(0, 0) / denominator;
    K->coeffRef(1) = P->coeff(1, 0) / denominator;
    
    float innovation = z - x.coeff(0);
    x.coeffRef(0) += K->coeff(0) * innovation;
    x.coeffRef(1) += K->coeff(1) * innovation;
    
    P->coeffRef(0, 0) = (1 - K->coeff(0)) * P->coeff(0, 0);
    P->coeffRef(0, 1) = (1 - K->coeff(0)) * P->coeff(0, 1);
    P->coeffRef(1, 0) = -K->coeff(1) * P->coeff(0, 0) + P->coeff(1, 0);
    P->coeffRef(1, 1) = -K->coeff(1) * P->coeff(0, 1) + P->coeff(1, 1);
}

float IMUSensor::fastAtan2(float y, float x) {
    return atan2f(y, x);
}

float IMUSensor::getRollKF() const {
    return roll_kf;
}

float IMUSensor::getPitchKF() const {
    return pitch_kf;
}

float IMUSensor::getYawKF() const {
    return yaw_kf;
}

float IMUSensor::getRollCF() const {
    return roll_cf;
}

float IMUSensor::getPitchCF() const {
    return pitch_cf;
}

float IMUSensor::getYawCF() const {
    return yaw_cf;
}

float IMUSensor::getDt() const {
    return dt;
  }