#ifndef THRUSTVECTORCONTROL_H
#define THRUSTVECTORCONTROL_H

#include "ServoController.h"
#include <BasicLinearAlgebra.h>

using namespace BLA;

class ThrustVectorControl {
private:
  ServoController frontServo;
  ServoController rightServo;
  ServoController rearServo;
  ServoController leftServo;

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
  void computeControlActions(const Matrix<3> &u_total);
  void applyControlActions();
  float getCurrentPropSpeed() const;
};

#endif // THRUSTVECTORCONTROL_H