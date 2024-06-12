#include "PIDController.h"

PIDController::PIDController(const Matrix3x3 &Kp, const Matrix3x3 &Kd)
    : K_p(Kp), K_d(Kd), error(), previous_error() {}

void PIDController::calculateError(const Matrix3x1 &current,
                                   const Matrix3x1 &desired) {
  for (int i = 0; i < 3; ++i)
    error.data[i] = desired.data[i] - current.data[i];
}

Matrix3x1 PIDController::computeProportionalTerm() { return K_p * error; }

Matrix3x1 PIDController::computeDerivativeTerm(float dt) {
  Matrix3x1 derivative;
  for (int i = 0; i < 3; ++i)
    derivative.data[i] = (error.data[i] - previous_error.data[i]) / dt;
  previous_error = error;
  return K_d * derivative;
}