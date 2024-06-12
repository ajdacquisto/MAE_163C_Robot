#ifndef DRONESTATE_H
#define DRONESTATE_H

#include "Matrix.h"

struct DroneState {
  Matrix3x1 position;         // X, Y, Z
  Matrix3x1 velocity;         // dX, dY, dZ
  Matrix3x1 orientation;      // Roll (phi), Pitch (θ), Yaw (psi)
  Matrix3x1 angular_velocity; // Roll rate (dPhi), Pitch rate (dTheta), Yaw rate
                              // (dPsi)

  DroneState();            // Constructor to initialize matrices
  void printState() const; // Method to print the current state (for debugging)
};

#endif // DRONESTATE_H