#include "DroneState.h"
#include <Arduino.h>      // For Serial
#include <avr/pgmspace.h> // For PROGMEM

DroneState::DroneState()
    : position(0), velocity(0), orientation(0), angular_velocity(0) {}

void DroneState::printState() const {
  // Use PROGMEM to store constant format strings
  const char formatPosition[] PROGMEM = "Position: \nX: %f, Y: %f, Z: %f\n";
  const char formatVelocity[] PROGMEM = "Velocity: \ndX: %f, dY: %f, dZ: %f\n";
  const char formatOrientation[] PROGMEM =
      "Orientation: \nRoll: %f, Pitch: %f, Yaw: %f\n";
  const char formatAngularVelocity[] PROGMEM =
      "Angular Velocity: \ndRoll: %f, dPitch: %f, dYaw: %f\n";

  char buffer[128]; // Reduced buffer size

  snprintf_P(buffer, sizeof(buffer), formatPosition, position(0), position(1),
             position(2));
  Serial.print(buffer);

  snprintf_P(buffer, sizeof(buffer), formatVelocity, velocity(0), velocity(1),
             velocity(2));
  Serial.print(buffer);

  snprintf_P(buffer, sizeof(buffer), formatOrientation, orientation(0),
             orientation(1), orientation(2));
  Serial.print(buffer);

  snprintf_P(buffer, sizeof(buffer), formatAngularVelocity, angular_velocity(0),
             angular_velocity(1), angular_velocity(2));
  Serial.print(buffer);
}