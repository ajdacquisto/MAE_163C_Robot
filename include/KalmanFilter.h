#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include "DroneState.h"
#include <BasicLinearAlgebra.h>

using namespace BLA;

class KalmanFilter {
private:
  DroneState state;
  BLA::Matrix<12, 12> F, P, Q, B;
  BLA::Matrix<6, 12> H;
  BLA::Matrix<6, 6> R;
  BLA::Matrix<12, 6> OKAY;

public:
  KalmanFilter();
  void initializeMatrices();
  void predict(const BLA::Matrix<3> &linear_acceleration,
               const BLA::Matrix<3> &angular_velocity);
  void update(const BLA::Matrix<3> &linear_acceleration,
              const BLA::Matrix<3> &angular_velocity);
  DroneState getState() const;
};

#endif // KALMANFILTER_H