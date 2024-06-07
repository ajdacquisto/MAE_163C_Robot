#ifndef IMUCONTROLLER_H
#define IMUCONTROLLER_H

#include <Wire.h>

/**
 * @class IMUController
 * @brief A class that represents an IMU (Inertial Measurement Unit) controller.
 * 
 * The IMUController class provides functionality to initialize the IMU, read data from it,
 * and store the raw data from the accelerometer, gyro, and temperature sensors.
 */
class IMUController {
public:
  /**
   * @brief Constructor for the IMUController class.
   */
  IMUController();

  /**
   * @brief Destructor for the IMUController class.
   */
  ~IMUController();

  // Add your member functions here
  /**
   * @brief Initializes the IMU.
   */
  void init();

  /**
   * @brief Reads data from the IMU.
   * @return A string containing the raw data from the IMU.
   */
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
  /**
   * @brief Converts an int16_t value to a string.
   * @param i The int16_t value to convert.
   * @return A string representation of the int16_t value.
   */
  String convert_int16_to_str(int16_t i);
};

#endif // IMUCONTROLLER_H
