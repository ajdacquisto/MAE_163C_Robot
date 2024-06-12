#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include <BasicLinearAlgebra.h>

using namespace BLA;

class PIDController {
private:
    Matrix<3, 3> K_p;
    Matrix<3, 3> K_d;
    Matrix<3> error;
    Matrix<3> previous_error;

public:
    PIDController(const Matrix<3, 3> &Kp, const Matrix<3, 3> &Kd);
    void calculateError(const Matrix<3> &current, const Matrix<3> &desired);
    Matrix<3> computeProportionalTerm() const;
    Matrix<3> computeDerivativeTerm(float dt);
};

#endif // PIDCONTROLLER_H