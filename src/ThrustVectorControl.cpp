#include "ThrustVectorControl.h"
#include <Arduino.h> // For constrain and map functions

ThrustVectorControl::ThrustVectorControl(uint8_t frontPin, uint8_t rightPin,
                                         uint8_t rearPin, uint8_t leftPin,
                                         float frontRearArm, float leftRightArm,
                                         float maxServoAngle, float maxThrust,
                                         float maxPropSpeed)
    : frontRearArmLength(frontRearArm), leftRightArmLength(leftRightArm),
      maxServoAngle(maxServoAngle), maxThrust(maxThrust), propSpeed(0.0f),
      maxPropSpeed(maxPropSpeed) {
  frontServo.attach(frontPin);
  rightServo.attach(rightPin);
  rearServo.attach(rearPin);
  leftServo.attach(leftPin);
}

void ThrustVectorControl::initialize() {
  // Initialize servos to their neutral positions
  frontServo.write(90);
  rightServo.write(90);
  rearServo.write(90);
  leftServo.write(90);
}

void ThrustVectorControl::computeControlActions(const Matrix<3> &u_total) {
  // Decompose u_total into thrust vector components
  float Fx = u_total(0);
  float Fy = u_total(1);
  float Fz = u_total(2);

  // Calculate the total required thrust to maintain the desired z-force
  float requiredThrust = sqrt(Fx * Fx + Fy * Fy + Fz * Fz);

  // Adjust propeller speed to provide the required thrust
  propSpeed = (requiredThrust / maxThrust) * maxPropSpeed;

  // Ensure the propeller speed does not exceed maximum limits
  propSpeed = constrain(propSpeed, 0.0f, maxPropSpeed);

  // Calculate required servo angles to achieve the desired forces and torques
  // Simplified calculation; adjust based on actual geometry
  float thetaFront = atan2(Fx, Fz) * 180.0f / PI;
  float thetaRear = atan2(-Fx, Fz) * 180.0f / PI;
  float thetaRight = atan2(Fy, Fz) * 180.0f / PI;
  float thetaLeft = atan2(-Fy, Fz) * 180.0f / PI;

  // Constrain the angles to the servo limits
  thetaFront = constrain(thetaFront, -maxServoAngle, maxServoAngle);
  thetaRear = constrain(thetaRear, -maxServoAngle, maxServoAngle);
  thetaRight = constrain(thetaRight, -maxServoAngle, maxServoAngle);
  thetaLeft = constrain(thetaLeft, -maxServoAngle, maxServoAngle);

  // Map the angles to servo positions (assuming 0-180 degrees range)
  int frontServoPos = map(thetaFront, -maxServoAngle, maxServoAngle, 0, 180);
  int rearServoPos = map(thetaRear, -maxServoAngle, maxServoAngle, 0, 180);
  int rightServoPos = map(thetaRight, -maxServoAngle, maxServoAngle, 0, 180);
  int leftServoPos = map(thetaLeft, -maxServoAngle, maxServoAngle, 0, 180);

  // Apply the servo angles
  frontServo.write(frontServoPos);
  rearServo.write(rearServoPos);
  rightServo.write(rightServoPos);
  leftServo.write(leftServoPos);
}

void ThrustVectorControl::applyControlActions() {
  // Apply propeller speed (e.g., send to ESC)
  // This part depends on how you're controlling the motor speed, e.g., using
  // PWM or another method
}

float ThrustVectorControl::getCurrentPropSpeed() const { return propSpeed; }