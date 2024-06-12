#ifndef RUNGEKUTTASTATEESTIMATOR_H
#define RUNGEKUTTASTATEESTIMATOR_H

#include "DroneState.h"
#include "IMUController.h"
#include "Matrix.h"

class RungeKuttaStateEstimator {
private:
  IMUController &imu;
  float delta_t;
  DroneState state;

  Matrix3x1 f(const Matrix3x1 &state, const Matrix3x1 &input);

public:
  RungeKuttaStateEstimator(IMUController &imu, float dt);

  void initialize();
  void update();
  DroneState getState() const;
};

#endif // RUNGEKUTTASTATEESTIMATOR_H