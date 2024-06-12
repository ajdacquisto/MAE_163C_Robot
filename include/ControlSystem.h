#ifndef CONTROLSYSTEM_H
#define CONTROLSYSTEM_H

#include "DroneState.h"
#include "FeedforwardController.h"
#include "FrameConverter.h"
#include "IMUController.h"
#include "PIDController.h"
#include "SDController.h"
#include "SimpleStateEstimator.h"
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
  SimpleStateEstimator estimator;
  DroneState state;
  Matrix3x1 desired_position;
  Matrix3x1 desired_velocity;
  Matrix3x1 desired_acceleration;

public:
  ControlSystem(float dt, const Matrix3x3 &Kp, const Matrix3x3 &Kd,
                float inertia, float mass, int sdCsPin, int frontPin,
                int rightPin, int rearPin, int leftPin, float frontRearArm,
                float leftRightArm, float maxServoAngle, float maxThrust,
                float maxPropSpeed, int pwm_pin);

  void initialize();
  void controlLoop();
  void applyControlInputs(const Matrix3x1 &u_total);
  void setDesiredState(const Matrix3x1 &pos, const Matrix3x1 &vel,
                       const Matrix3x1 &acc);
  void logState();
};

#endif // CONTROLSYSTEM_H