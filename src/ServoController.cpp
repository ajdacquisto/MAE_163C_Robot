#include "ServoController.h"
#include <Arduino.h>

ServoController::ServoController() : _pin(255) {}

void ServoController::attach(uint8_t pin) {
  _pin = pin;
  pinMode(_pin, OUTPUT);
}

void ServoController::write(uint8_t angle) {
  if (_pin != 255) {
    // Convert angle to pulse width in microseconds (e.g., 0-180 degrees to
    // 544-2400 microseconds)
    int pulseWidth = map(angle, 0, 180, 544, 2400);
    analogWrite(
        _pin, pulseWidth /
                  4); // Arduino's analogWrite function takes a value from 0-255
  }
}