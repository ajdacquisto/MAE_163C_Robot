#include "PIDController.h"

PIDController::PIDController(const Matrix<3, 3> &Kp, const Matrix<3, 3> &Kd)
    : K_p(Kp), K_d(Kd), error(), previous_error() {}

void PIDController::calculateError(const Matrix<3> &current,
                                   const Matrix<3> &desired) {
  error = desired - current;
}

Matrix<3> PIDController::computeProportionalTerm() const {
  return K_p * error;
}

Matrix<3> PIDController::computeDerivativeTerm(float dt) {
  Matrix<3> derivative = (error - previous_error) / dt;
  previous_error = error;
  return K_d * derivative;
}