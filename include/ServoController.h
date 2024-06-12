#ifndef SERVOCONTROLLER_H
#define SERVOCONTROLLER_H

#include <Arduino.h>

class ServoController {
public:
  ServoController();
  void attach(uint8_t pin);
  void write(uint8_t angle);

private:
  uint8_t _pin;
};

#endif // SERVOCONTROLLER_H