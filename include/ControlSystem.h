#ifndef CONTROLSYSTEM_H
#define CONTROLSYSTEM_H

#include "DroneState.h"
#include "FeedforwardController.h"
#include "FrameConverter.h"
#include "IMUController.h"
#include "KalmanFilter.h"
#include "PIDController.h"
#include "SDController.h"
#include "SpeedController.h"
#include "ThrustVectorControl.h"
#include <BasicLinearAlgebra.h>

using namespace BLA;

class ControlSystem {
private:
  float delta_t;
  IMUController imu;
  PIDController pid;
  KalmanFilter kf;
  FeedforwardController ff;
  SDController sd;
  ThrustVectorControl tvc;
  SpeedController speedController;
  DroneState state;
  BLA::Matrix<3> desired_position;
  BLA::Matrix<3> desired_velocity;
  BLA::Matrix<3> desired_acceleration;

public:
  ControlSystem(float dt, const BLA::Matrix<3, 3> &Kp,
                const BLA::Matrix<3, 3> &Kd, float inertia, float mass,
                uint8_t sdCsPin, uint8_t frontPin, uint8_t rightPin,
                uint8_t rearPin, uint8_t leftPin, float frontRearArm,
                float leftRightArm, float maxServoAngle, float maxThrust,
                float maxPropSpeed, uint8_t pwm_pin);

  void initialize();
  void controlLoop();
  void applyControlInputs(const BLA::Matrix<3> &u_total);
  void setDesiredState(const BLA::Matrix<3> &pos, const BLA::Matrix<3> &vel,
                       const BLA::Matrix<3> &acc);
  void logState();
};

#endif // CONTROLSYSTEM_H