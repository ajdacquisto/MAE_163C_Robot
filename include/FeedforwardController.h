#ifndef FEEDFORWARDCONTROLLER_H
#define FEEDFORWARDCONTROLLER_H

#include <BasicLinearAlgebra.h>

using namespace BLA;

class FeedforwardController {
private:
  float I_p; // Moment of inertia of the propeller
  float m;   // Mass of the drone

public:
  FeedforwardController(float inertia, float mass);

  void computeGyroscopicTorque(float omega_p, const Matrix<3> &angular_velocity,
                               Matrix<3> &gyroscopic_torques) const;
  Matrix<3>
  computeGravityCompensation() const; // Updated to return in world coordinates
};

#endif // FEEDFORWARDCONTROLLER_H