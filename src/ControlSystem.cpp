#include "ControlSystem.h"
#include <Arduino.h> // For delay function

ControlSystem::ControlSystem(float dt, const Matrix3x3 &Kp, const Matrix3x3 &Kd,
                             float inertia, float mass, int sdCsPin,
                             int frontPin, int rightPin, int rearPin,
                             int leftPin, float frontRearArm,
                             float leftRightArm, float maxServoAngle,
                             float maxThrust, float maxPropSpeed, int pwm_pin)
    : delta_t(dt), imu(), pid(Kp, Kd), ff(inertia, mass), sd(sdCsPin),
      tvc(frontPin, rightPin, rearPin, leftPin, frontRearArm, leftRightArm,
          maxServoAngle, maxThrust, maxPropSpeed),
      speedController(pwm_pin), estimator(imu, dt), state(), desired_position(),
      desired_velocity(), desired_acceleration() {}

void ControlSystem::initialize() {
  // Initialize components if needed
  sd.begin();
  tvc.initialize();
  imu.init();
  speedController.init();
  estimator.initialize();
}

void ControlSystem::controlLoop() {
  // Update state estimation
  estimator.update();
  state = estimator.getState();

  // Step 4: Calculate error
  pid.calculateError(state.position, desired_position);

  // Step 5: Compute PD control inputs
  Matrix3x1 u_P = pid.computeProportionalTerm();
  Matrix3x1 u_D = pid.computeDerivativeTerm(delta_t);

  // Step 6: Compute gravity compensation in world coordinates
  Matrix3x1 u_grav = ff.computeGravityCompensation();

  // Step 7: Combine PD control inputs with gravity compensation in world
  // coordinates
  Matrix3x1 u_PDG = u_P + u_D + u_grav;

  // Step 8: Transform combined control input to local coordinates
  Matrix3x1 u_local = FrameConverter::toLocalFrame(u_PDG, state.orientation);

  // Step 9: Compute gyroscopic compensation
  float propeller_speed = speedController.getSpeed();
  Matrix3x1 gyroscopic_torques;
  ff.computeGyroscopicTorque(propeller_speed, state.angular_velocity,
                             gyroscopic_torques);

  // Step 10: Combine local control input with gyroscopic compensation
  Matrix3x1 u_total = u_local + gyroscopic_torques;

  // Step 11: Compute and apply the servo angles and prop speed
  tvc.computeControlActions(u_total);
  tvc.applyControlActions();

  // Optionally, update propeller speed if needed
  // speedController.setSpeed(new_speed);

  // Log the current state to the SD card
  logState();
}

void ControlSystem::applyControlInputs(const Matrix3x1 &u_total) {
  // Apply control inputs using thrust vector control
  tvc.computeControlActions(u_total);
  tvc.applyControlActions();
}

void ControlSystem::setDesiredState(const Matrix3x1 &pos, const Matrix3x1 &vel,
                                    const Matrix3x1 &acc) {
  desired_position = pos;
  desired_velocity = vel;
  desired_acceleration = acc;
}

void ControlSystem::logState() {
  char data[256];
  char buf[20]; // Buffer to hold the float converted to string

  snprintf(data, sizeof(data),
           "Position: %s, %s, %s | Velocity: %s, %s, %s | Orientation: %s, %s, "
           "%s | Angular Velocity: %s, %s, %s",
           dtostrf(state.position.data[0], 6, 2, buf),
           dtostrf(state.position.data[1], 6, 2, buf),
           dtostrf(state.position.data[2], 6, 2, buf),
           dtostrf(state.velocity.data[0], 6, 2, buf),
           dtostrf(state.velocity.data[1], 6, 2, buf),
           dtostrf(state.velocity.data[2], 6, 2, buf),
           dtostrf(state.orientation.data[0], 6, 2, buf),
           dtostrf(state.orientation.data[1], 6, 2, buf),
           dtostrf(state.orientation.data[2], 6, 2, buf),
           dtostrf(state.angular_velocity.data[0], 6, 2, buf),
           dtostrf(state.angular_velocity.data[1], 6, 2, buf),
           dtostrf(state.angular_velocity.data[2], 6, 2, buf));

  sd.writeData(data);
}