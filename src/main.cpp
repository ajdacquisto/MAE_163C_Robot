#include "DataProcessor.h"
#include "IMUController.h"
#include "SDController.h"
#include "ServoController.h"
#include "config.h"
#include <Arduino.h>

// Control objects
IMUController imuController;
ServoController servoController;
SDController sdController(SD_CS_PIN);
DataProcessor dataProcessor;

// put function declarations here:

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  imuController.init();
  servoController.init();

  if (sdController.begin()) {
    Serial.println("SD card initialized.");
  } else {
    Serial.println("SD card initialization failed.");
    return;
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(imuController.read());
}