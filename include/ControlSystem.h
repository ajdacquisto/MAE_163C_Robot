#ifndef CONTROLSYSTEM_H
#define CONTROLSYSTEM_H

#include "DroneState.h"
#include "FeedforwardController.h"
#include "FrameConverter.h"
#include "IMUController.h"
#include "PIDController.h"
#include "RungeKuttaStateEstimator.h" // Include the new estimator
#include "SDController.h"
#include "SpeedController.h"
#include "ThrustVectorControl.h"

class ControlSystem {
private:
  float delta_t;
  IMUController imu;
  PIDController pid;
  FeedforwardController ff;
  SDController sd;
  ThrustVectorControl tvc;
  SpeedController speedController;
  RungeKuttaStateEstimator stateEstimator; // Use the new estimator
  DroneState state;
  Matrix3x1 desired_position;
  Matrix3x1 desired_velocity;
  Matrix3x1 desired_acceleration;

public:
  ControlSystem(float dt, const Matrix3x3 &Kp, const Matrix3x3 &Kd,
                float inertia, float mass, uint8_t sdCsPin, uint8_t frontPin,
                uint8_t rightPin, uint8_t rearPin, uint8_t leftPin,
                float frontRearArm, float leftRightArm, float maxServoAngle,
                float maxThrust, float maxPropSpeed, uint8_t pwm_pin);

  void initialize();
  void controlLoop();
  void applyControlInputs(const Matrix3x1 &u_total);
  void setDesiredState(const Matrix3x1 &pos, const Matrix3x1 &vel,
                       const Matrix3x1 &acc);
  void logState();
};

#endif // CONTROLSYSTEM_H