#ifndef SERVOCONTROLLER_H
#define SERVOCONTROLLER_H

#include "config.h"
#include <Servo.h> // Update the include path to use angle brackets instead of quotes

class ServoController {
public:
  ServoController();
  ~ServoController();

  // Add your member functions here
  void init();

private:
  // Add your member variables here
  Servo servo1;
  Servo servo2;
  Servo servo3;
  Servo servo4;
};

#endif // SERVOCONTROLLER_H