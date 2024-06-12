#ifndef THRUSTVECTORCONTROL_H
#define THRUSTVECTORCONTROL_H

#include "Matrix.h"
#include <Arduino.h> // For uint8_t

class ThrustVectorControl {
private:
  uint8_t frontServoPin;
  uint8_t rightServoPin;
  uint8_t rearServoPin;
  uint8_t leftServoPin;

  float frontRearArmLength; // Distance from the center of gravity to the
                            // front/rear servos
  float leftRightArmLength; // Distance from the center of gravity to the
                            // left/right servos
  float maxServoAngle;      // Maximum angle the servo can achieve
  float maxThrust;          // Maximum thrust each thruster can provide
  float propSpeed;          // Current propeller speed
  float maxPropSpeed;       // Maximum propeller speed

public:
  ThrustVectorControl(uint8_t frontPin, uint8_t rightPin, uint8_t rearPin,
                      uint8_t leftPin, float frontRearArm, float leftRightArm,
                      float maxServoAngle, float maxThrust, float maxPropSpeed);

  void initialize();
  void computeControlActions(const Matrix3x1 &u_total);
  void applyControlActions();
  float getCurrentPropSpeed() const;
};

#endif // THRUSTVECTORCONTROL_H