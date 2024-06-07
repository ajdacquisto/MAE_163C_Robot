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