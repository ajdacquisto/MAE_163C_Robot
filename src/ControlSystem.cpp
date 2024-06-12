#include "ControlSystem.h"
#include <Arduino.h> // For delay function

ControlSystem::ControlSystem(float dt, const Matrix3x3 &Kp, const Matrix3x3 &Kd,
                             float inertia, float mass, uint8_t sdCsPin,
                             uint8_t frontPin, uint8_t rightPin,
                             uint8_t rearPin, uint8_t leftPin,
                             float frontRearArm, float leftRightArm,
                             float maxServoAngle, float maxThrust,
                             float maxPropSpeed, uint8_t pwm_pin)
    : delta_t(dt), imu(), pid(Kp, Kd), ff(inertia, mass), sd(sdCsPin),
      tvc(frontPin, rightPin, rearPin, leftPin, frontRearArm, leftRightArm,
          maxServoAngle, maxThrust, maxPropSpeed),
      speedController(pwm_pin), stateEstimator(imu, dt), state() {}

void ControlSystem::initialize() {
  // Initialize components if needed
  sd.begin();
  tvc.initialize();
  imu.init();
  speedController.init();
  stateEstimator.initialize();
}

void ControlSystem::controlLoop() {
  // Step 1: Update state estimation with Runge-Kutta method
  stateEstimator.update();
  state = stateEstimator.getState();

  // Step 2: Calculate error
  pid.calculateError(state.position, desired_position);

  // Step 3: Compute PD control inputs
  Matrix3x1 u_P = pid.computeProportionalTerm();
  Matrix3x1 u_D = pid.computeDerivativeTerm(delta_t);

  // Step 4: Compute gravity compensation in world coordinates
  Matrix3x1 u_grav = ff.computeGravityCompensation();

  // Step 5: Combine PD control inputs with gravity compensation in world
  // coordinates
  Matrix3x1 u_PDG = u_P + u_D + u_grav;

  // Step 6: Transform combined control input to local coordinates
  Matrix3x1 u_local = FrameConverter::toLocalFrame(u_PDG, state.orientation);

  // Step 7: Compute gyroscopic compensation
  Matrix3x1 gyroscopic_torques;
  ff.computeGyroscopicTorque(0, state.angular_velocity, gyroscopic_torques);

  // Step 8: Combine local control input with gyroscopic compensation
  Matrix3x1 u_total = u_local + gyroscopic_torques;

  // Step 9: Compute and apply the servo angles and prop speed
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
  char buffer[256];
  snprintf(buffer, sizeof(buffer),
           "Position: %f, %f, %f | Velocity: %f, %f, %f | Orientation: %f, %f, "
           "%f | Angular Velocity: %f, %f, %f",
           static_cast<double>(state.position.data[0]),
           static_cast<double>(state.position.data[1]),
           static_cast<double>(state.position.data[2]),
           static_cast<double>(state.velocity.data[0]),
           static_cast<double>(state.velocity.data[1]),
           static_cast<double>(state.velocity.data[2]),
           static_cast<double>(state.orientation.data[0]),
           static_cast<double>(state.orientation.data[1]),
           static_cast<double>(state.orientation.data[2]),
           static_cast<double>(state.angular_velocity.data[0]),
           static_cast<double>(state.angular_velocity.data[1]),
           static_cast<double>(state.angular_velocity.data[2]));

  sd.writeData(buffer);
}