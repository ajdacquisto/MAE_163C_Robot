#include "SpeedController.h"
#include <Arduino.h>

SpeedController::SpeedController(int pwm_pin)
    : pwm_pin(pwm_pin), propeller_speed(0.0f) {}

void SpeedController::init() {
  pinMode(pwm_pin, OUTPUT);
  analogWrite(pwm_pin, 0);
}

void SpeedController::setSpeed(float speed) {
  propeller_speed = speed;
  // Convert speed to PWM value (adjust the mapping according to your hardware)
  int pwm_value = map(speed, 0.0f, 2000.0f, 0, 255);
  analogWrite(pwm_pin, pwm_value);
}

float SpeedController::getSpeed() const { return propeller_speed; }