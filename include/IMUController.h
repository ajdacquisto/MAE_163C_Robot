#ifndef IMUCONTROLLER_H
#define IMUCONTROLLER_H

#include "Matrix.h"
#include <Wire.h>

class IMUController {
private:
  const int MPU_ADDR = 0x68; // I2C address of the MPU-6050
  int16_t accelerometer_x, accelerometer_y, accelerometer_z;
  int16_t gyro_x, gyro_y, gyro_z;

public:
  IMUController();
  ~IMUController();

  void init();
  void read(Matrix3x1 &linear_acceleration, Matrix3x1 &angular_velocity);
};

#endif // IMUCONTROLLER_H