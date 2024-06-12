/**
 * @file IMUController.cpp
 * @brief Implementation file for the IMUController class.
 */

#include "IMUController.h"

/**
 * @brief Constructs a new IMUController object.
 *
 * This constructor is responsible for initializing the IMUController object.
 * Any necessary setup or initialization code can be placed here.
 */
IMUController::IMUController() {
  // Constructor implementation
}

/**
 * @brief Destructor for the IMUController class.
 *
 * This destructor is responsible for cleaning up any resources
 * allocated by the IMUController object.
 */
IMUController::~IMUController() {
  // Destructor implementation
}

/**
 * Initializes the IMU.
 * This function initializes the IMU by calling Wire.begin().
 */
void IMUController::init() {
  // Initialize the IMU
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0);    // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
}

/**
 * Reads the sensor data from the IMU (Inertial Measurement Unit).
 *
 * This function reads the accelerometer and gyroscope data from the IMU
 * and stores them in the provided matrices.
 *
 * @param linear_acceleration A matrix to store the accelerometer data.
 * @param angular_velocity A matrix to store the gyroscope data.
 */
void IMUController::read(Matrix<3> &linear_acceleration,
                         Matrix<3> &angular_velocity) {
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
  constexpr float accel_scale =
      16384.0f; // 2g range, LSB sensitivity = 16384 LSB/g
  constexpr float gyro_scale =
      131.0f; // 250 degrees/s range, LSB sensitivity = 131 LSB/(degrees/s)

  // Convert raw values to appropriate units and store in matrices
  linear_acceleration(0) = accelerometer_x / accel_scale;
  linear_acceleration(1) = accelerometer_y / accel_scale;
  linear_acceleration(2) = accelerometer_z / accel_scale;

  angular_velocity(0) = gyro_x / gyro_scale;
  angular_velocity(1) = gyro_y / gyro_scale;
  angular_velocity(2) = gyro_z / gyro_scale;
}