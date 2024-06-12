#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include "DroneState.h"
#include "Matrix.h"

class KalmanFilter {
private:
  DroneState state;
  Matrix3x3 F;
  Matrix3x3 P;
  Matrix3x3 Q;
  Matrix3x3 B;
  Matrix3x3 H;
  Matrix3x3 R;
  Matrix3x3 OKAY;

public:
  KalmanFilter();
  void initializeMatrices();
  void predict(const Matrix3x1 &linear_acceleration,
               const Matrix3x1 &angular_velocity);
  void update(const Matrix3x1 &linear_acceleration,
              const Matrix3x1 &angular_velocity);
  DroneState getState() const;
};

#endif // KALMANFILTER_H