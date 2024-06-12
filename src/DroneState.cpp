#include "DroneState.h"
#include <Arduino.h>      // For Serial
#include <avr/pgmspace.h> // For PROGMEM

DroneState::DroneState()
    : position(), velocity(), orientation(), angular_velocity() {}

void DroneState::printState() const {
  // Use PROGMEM to store constant format strings
  const char formatPosition[] PROGMEM = "Position: \nX: %f, Y: %f, Z: %f\n";
  const char formatVelocity[] PROGMEM = "Velocity: \ndX: %f, dY: %f, dZ: %f\n";
  const char formatOrientation[] PROGMEM =
      "Orientation: \nRoll: %f, Pitch: %f, Yaw: %f\n";
  const char formatAngularVelocity[] PROGMEM =
      "Angular Velocity: \ndRoll: %f, dPitch: %f, dYaw: %f\n";

  char buffer[128]; // Reduced buffer size

  snprintf_P(buffer, sizeof(buffer), formatPosition, position.data[0],
             position.data[1], position.data[2]);
  Serial.print(buffer);

  snprintf_P(buffer, sizeof(buffer), formatVelocity, velocity.data[0],
             velocity.data[1], velocity.data[2]);
  Serial.print(buffer);

  snprintf_P(buffer, sizeof(buffer), formatOrientation, orientation.data[0],
             orientation.data[1], orientation.data[2]);
  Serial.print(buffer);

  snprintf_P(buffer, sizeof(buffer), formatAngularVelocity,
             angular_velocity.data[0], angular_velocity.data[1],
             angular_velocity.data[2]);
  Serial.print(buffer);
}