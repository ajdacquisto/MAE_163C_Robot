#include "RungeKuttaStateEstimator.h"

RungeKuttaStateEstimator::RungeKuttaStateEstimator(IMUController &imu, float dt)
    : imu(imu), delta_t(dt) {}

void RungeKuttaStateEstimator::initialize() { imu.init(); }

Matrix3x1 RungeKuttaStateEstimator::f(const Matrix3x1 &state,
                                      const Matrix3x1 &input) {
  Matrix3x1 derivative;
  for (int i = 0; i < 3; i++) {
    derivative.data[i] = input.data[i]; // Assume input is acceleration
  }
  return derivative;
}

void RungeKuttaStateEstimator::update() {
  Matrix3x1 linear_acceleration, angular_velocity;
  imu.read(linear_acceleration, angular_velocity);

  // Runge-Kutta integration
  Matrix3x1 k1 = f(state.velocity, linear_acceleration) * delta_t;
  Matrix3x1 k2 = f(state.velocity + k1 * 0.5, linear_acceleration) * delta_t;
  Matrix3x1 k3 = f(state.velocity + k2 * 0.5, linear_acceleration) * delta_t;
  Matrix3x1 k4 = f(state.velocity + k3, linear_acceleration) * delta_t;

  for (int i = 0; i < 3; i++) {
    state.velocity.data[i] +=
        (k1.data[i] + 2 * k2.data[i] + 2 * k3.data[i] + k4.data[i]) / 6.0;
  }

  // Integrate velocity to update position
  for (int i = 0; i < 3; i++) {
    state.position.data[i] += state.velocity.data[i] * delta_t;
  }

  // Update orientation directly from IMU (simplified for this example)
  state.orientation = angular_velocity; // Assuming direct angular velocity as
                                        // orientation for simplicity
}

DroneState RungeKuttaStateEstimator::getState() const { return state; }