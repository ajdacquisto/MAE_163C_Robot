/**
 * @file main.cpp
 * @brief This file contains the main code for the MAE_163C_Robot project.
 */

#include "DataProcessor.h"
#include "IMUController.h"
#include "SDController.h"
#include "ServoController.h"
#include "config.h"
#include <Arduino.h>

// Control objects
IMUController imu;
ServoController servoController;
SDController sdController(SD_CS_PIN);
DataProcessor dataProcessor;

// put function declarations here:

/**
 * @brief The setup function.
 * 
 * This function is called once when the Arduino board is powered on or reset.
 * It initializes the serial communication, IMU, servo controller, and SD card.
 */
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  imu.init();
  servoController.init();

  if (sdController.begin()) {
    Serial.println("SD card initialized.");
  } else {
    Serial.println("SD card initialization failed.");
    return;
  }
}

/**
 * @brief The loop function.
 * 
 * This function is called repeatedly after the setup function.
 * It reads IMU data, processes accelerometer and gyro data, and performs additional operations using the processed data.
 */
void loop() {
  String imuData =
      imu.read(); // Assuming `imu.read()` updates accelerometer and gyro data

  // Process accelerometer data
  dataProcessor.processAccelerometer(imu.accelerometer_x, imu.accelerometer_y,
                                     imu.accelerometer_z);

  // Process gyro data
  dataProcessor.processGyro(imu.gyro_x, imu.gyro_y, imu.gyro_z);

  // Add your code to use processed data
}