#ifndef FEEDFORWARDCONTROLLER_H
#define FEEDFORWARDCONTROLLER_H

#include "Matrix.h"

class FeedforwardController {
private:
  float I_p; // Moment of inertia of the propeller (you should set this in the
             // constructor or a setter method)
  float m;   // Mass of the drone (you should set this in the constructor or a
             // setter method)

public:
  FeedforwardController(float inertia, float mass);

  void computeGyroscopicTorque(float omega_p, const Matrix3x1 &angular_velocity,
                               Matrix3x1 &gyroscopic_torques) const;
  Matrix3x1
  computeGravityCompensation() const; // Updated to return in world coordinates
};

#endif // FEEDFORWARDCONTROLLER_H