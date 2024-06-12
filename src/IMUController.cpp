#include "IMUController.h"

IMUController::IMUController() {
  // Constructor implementation
}

IMUController::~IMUController() {
  // Destructor implementation
}

void IMUController::init() {
  // Initialize the IMU
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0);    // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
}

void IMUController::read(Matrix3x1 &linear_acceleration,
                         Matrix3x1 &angular_velocity) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);            // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false); // keep the connection active
  Wire.requestFrom(
      MPU_ADDR, 6 * 2,
      true); // request a total of 12 registers (6 for accel, 6 for gyro)

  accelerometer_x = Wire.read() << 8 | Wire.read();
  accelerometer_y = Wire.read() << 8 | Wire.read();
  accelerometer_z = Wire.read() << 8 | Wire.read();
  gyro_x = Wire.read() << 8 | Wire.read();
  gyro_y = Wire.read() << 8 | Wire.read();
  gyro_z = Wire.read() << 8 | Wire.read();

  // Scale factors for MPU-6050 (assuming default settings, adjust if necessary)
  float accel_scale = 16384.0; // 2g range, LSB sensitivity = 16384 LSB/g
  float gyro_scale =
      131.0; // 250 degrees/s range, LSB sensitivity = 131 LSB/(degrees/s)

  // Convert raw values to appropriate units and store in matrices
  linear_acceleration.data[0] = accelerometer_x / accel_scale;
  linear_acceleration.data[1] = accelerometer_y / accel_scale;
  linear_acceleration.data[2] = accelerometer_z / accel_scale;

  angular_velocity.data[0] = gyro_x / gyro_scale;
  angular_velocity.data[1] = gyro_y / gyro_scale;
  angular_velocity.data[2] = gyro_z / gyro_scale;
}