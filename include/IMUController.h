#ifndef IMUCONTROLLER_H
#define IMUCONTROLLER_H

#include <BasicLinearAlgebra.h>
#include <Wire.h>

using namespace BLA;

class IMUController {
private:
  static const int MPU_ADDR = 0x68; // I2C address of the MPU-6050
  int16_t accelerometer_x, accelerometer_y, accelerometer_z;
  int16_t gyro_x, gyro_y, gyro_z;

public:
  IMUController();
  ~IMUController();

  void init();
  void read(Matrix<3> &linear_acceleration, Matrix<3> &angular_velocity);
};

#endif // IMUCONTROLLER_H