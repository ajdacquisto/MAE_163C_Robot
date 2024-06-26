#include "ControlSystem.h"
#include <Arduino.h>      // For delay function
#include <avr/pgmspace.h> // For PROGMEM

ControlSystem::ControlSystem(float dt, const Matrix3x3 &Kp, const Matrix3x3 &Kd,
                             float inertia, float mass, uint8_t sdCsPin,
                             uint8_t frontPin, uint8_t rightPin,
                             uint8_t rearPin, uint8_t leftPin,
                             float frontRearArm, float leftRightArm,
                             float maxServoAngle, float maxThrust,
                             float maxPropSpeed, uint8_t pwm_pin)
    : delta_t(dt), imu(), pid(Kp, Kd), kf(), ff(inertia, mass), sd(sdCsPin),
      tvc(frontPin, rightPin, rearPin, leftPin, frontRearArm, leftRightArm,
          maxServoAngle, maxThrust, maxPropSpeed),
      speedController(pwm_pin), state(), desired_position(), desired_velocity(),
      desired_acceleration() {}

void ControlSystem::initialize() {
  // Initialize components if needed
  sd.begin();
  tvc.initialize();
  imu.init();
  speedController.init();
}

void ControlSystem::controlLoop() {
  // Step 1: Read IMU data
  Matrix3x1 linear_acceleration, angular_velocity;
  imu.read(linear_acceleration, angular_velocity);

  // Step 2: Read propeller speed from SpeedController
  float propeller_speed = speedController.getSpeed();

  // Step 3: State estimation with Kalman filter
  kf.predict(linear_acceleration, angular_velocity);
  kf.update(linear_acceleration, angular_velocity);
  state = kf.getState();

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
  // Use PROGMEM to store constant format strings
  const char formatPosition[] PROGMEM = "Position : % f, % f, % f";
  const char formatVelocity[] PROGMEM = " | Velocity : % f, % f, % f";
  const char formatOrientation[] PROGMEM = " | Orientation : % f, % f, % f";
  const char formatAngularVelocity[] PROGMEM =
      " | Angular Velocity : % f, % f, % f ";

  char data[128]; // Reduced buffer size
  snprintf_P(data, sizeof(data), formatPosition, state.position.data[0],
             state.position.data[1], state.position.data[2]);
  snprintf_P(data + strlen(data), sizeof(data) - strlen(data), formatVelocity,
             state.velocity.data[0], state.velocity.data[1],
             state.velocity.data[2]);
  snprintf_P(data + strlen(data), sizeof(data) - strlen(data),
             formatOrientation, state.orientation.data[0],
             state.orientation.data[1], state.orientation.data[2]);
  snprintf_P(data + strlen(data), sizeof(data) - strlen(data),
             formatAngularVelocity, state.angular_velocity.data[0],
             state.angular_velocity.data[1], state.angular_velocity.data[2]);

  sd.writeData(data);
}