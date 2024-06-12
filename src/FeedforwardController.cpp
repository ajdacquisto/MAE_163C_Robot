#include "FeedforwardController.h"
#include <math.h> // For trigonometric functions

FeedforwardController::FeedforwardController(float inertia, float mass)
    : I_p(inertia), m(mass) {}

void FeedforwardController::computeGyroscopicTorque(
    float omega_p, const Matrix<3> &angular_velocity,
    Matrix<3> &gyroscopic_torques) const {
  gyroscopic_torques(0) = I_p * omega_p * angular_velocity(1);
  gyroscopic_torques(1) = -I_p * omega_p * angular_velocity(0);
  gyroscopic_torques(2) = 0;
}

Matrix<3> FeedforwardController::computeGravityCompensation() const {
  Matrix<3> g_world = {0, 0, -9.81 * m};
  return g_world; // Gravity in world coordinates
}