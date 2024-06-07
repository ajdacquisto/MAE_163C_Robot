#include "IMUController.h"
#include "ServoController.h"
#include <Arduino.h>

// Control objects
IMUController imuController;
ServoController servoController;

// put function declarations here:

void setup() {
  // put your setup code here, to run once:
  imuController.init();
  servoController.init();
}

void loop() {
  // put your main code here, to run repeatedly:
}