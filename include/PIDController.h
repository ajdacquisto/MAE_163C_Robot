#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include "Matrix.h"

class PIDController {
private:
  Matrix3x3 K_p;
  Matrix3x3 K_d;
  Matrix3x1 error;
  Matrix3x1 previous_error;

public:
  PIDController(const Matrix3x3 &Kp, const Matrix3x3 &Kd);
  void calculateError(const Matrix3x1 &current, const Matrix3x1 &desired);
  Matrix3x1 computeProportionalTerm();
  Matrix3x1 computeDerivativeTerm(float dt);
};

#endif // PIDCONTROLLER_H