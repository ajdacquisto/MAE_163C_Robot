#include "FeedforwardController.h"
#include <math.h> // For trigonometric functions

FeedforwardController::FeedforwardController(float inertia, float mass)
    : I_p(inertia), m(mass) {}

void FeedforwardController::computeGyroscopicTorque(
    float omega_p, const Matrix3x1 &angular_velocity,
    Matrix3x1 &gyroscopic_torques) const {
  gyroscopic_torques.data[0] = I_p * omega_p * angular_velocity.data[1];
  gyroscopic_torques.data[1] = -I_p * omega_p * angular_velocity.data[0];
  gyroscopic_torques.data[2] = 0;
}

Matrix3x1 FeedforwardController::computeGravityCompensation() const {
  Matrix3x1 g_world;
  g_world.data[0] = 0;
  g_world.data[1] = 0;
  g_world.data[2] = -9.81 * m;
  return g_world; // Gravity in world coordinates
}