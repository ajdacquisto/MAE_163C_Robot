#include "SimpleStateEstimator.h"
#include <Arduino.h> // For time functions

SimpleStateEstimator::SimpleStateEstimator(IMUController &imuController, float dt)
    : imu(imuController), delta_t(dt) {}

void SimpleStateEstimator::initialize() {
    imu.init();
    // Initialize the state variables if needed
}

void SimpleStateEstimator::update() {
    Matrix3x1 linear_acceleration, angular_velocity;
    imu.read(linear_acceleration, angular_velocity);

    // Integrate acceleration to update velocity
    for (int i = 0; i < 3; i++) {
        state.velocity.data[i] += linear_acceleration.data[i] * delta_t;
    }

    // Integrate velocity to update position
    for (int i = 0; i < 3; i++) {
        state.position.data[i] += state.velocity.data[i] * delta_t;
    }

    // Update orientation directly from IMU
    state.orientation = angular_velocity; // Assuming direct angular velocity as orientation for simplicity
}

DroneState SimpleStateEstimator::getState() const {
    return state;
}