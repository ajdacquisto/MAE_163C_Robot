#ifndef IMUCONTROLLER_H
#define IMUCONTROLLER_H

#include <Wire.h>
class IMUController {
public:
  IMUController();  // Constructor
  ~IMUController(); // Destructor

  // Add your member functions here
  void init();
  String read();

  int16_t accelerometer_x, accelerometer_y,
      accelerometer_z;            // variables for accelerometer raw data
  int16_t gyro_x, gyro_y, gyro_z; // variables for gyro raw data

private:
  // Add your member variables here
  const int MPU_ADDR = 0x68; // I2C address of the MPU-6050. If AD0 pin is set
                             // to HIGH, the I2C address will be 0x69.
  int16_t temperature;       // variables for temperature data
  char tmp_str[7];           // temporary variable used in convert function

  // Private helper functions
  String convert_int16_to_str(int16_t i);
};

#endif // IMUCONTROLLER_H