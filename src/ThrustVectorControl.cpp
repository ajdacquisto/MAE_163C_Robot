#include "ThrustVectorControl.h"
#include <Arduino.h> // For pinMode, analogWrite, etc.

ThrustVectorControl::ThrustVectorControl(uint8_t frontPin, uint8_t rightPin,
                                         uint8_t rearPin, uint8_t leftPin,
                                         float frontRearArm, float leftRightArm,
                                         float maxServoAngle, float maxThrust,
                                         float maxPropSpeed)
    : frontServoPin(frontPin), rightServoPin(rightPin), rearServoPin(rearPin),
      leftServoPin(leftPin), frontRearArmLength(frontRearArm),
      leftRightArmLength(leftRightArm), maxServoAngle(maxServoAngle),
      maxThrust(maxThrust), propSpeed(0.0f), maxPropSpeed(maxPropSpeed) {}

void ThrustVectorControl::initialize() {
  // Initialize pins
  pinMode(frontServoPin, OUTPUT);
  pinMode(rightServoPin, OUTPUT);
  pinMode(rearServoPin, OUTPUT);
  pinMode(leftServoPin, OUTPUT);

  // Initialize servos to their neutral positions (example for PWM control)
  analogWrite(frontServoPin, 90); // Assuming 90 is neutral for your setup
  analogWrite(rightServoPin, 90);
  analogWrite(rearServoPin, 90);
  analogWrite(leftServoPin, 90);
}

void ThrustVectorControl::computeControlActions(const Matrix3x1 &u_total) {
  // Decompose u_total into thrust vector components
  float Fx = u_total.data[0];
  float Fy = u_total.data[1];
  float Fz = u_total.data[2];

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

  // Apply the servo angles (example for PWM control)
  analogWrite(frontServoPin, frontServoPos);
  analogWrite(rearServoPin, rearServoPos);
  analogWrite(rightServoPin, rightServoPos);
  analogWrite(leftServoPin, leftServoPos);
}

void ThrustVectorControl::applyControlActions() {
  // Apply propeller speed (e.g., send to ESC)
  // This part depends on how you're controlling the motor speed, e.g., using
  // PWM or another method
}

float ThrustVectorControl::getCurrentPropSpeed() const { return propSpeed; }