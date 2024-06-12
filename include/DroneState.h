#ifndef DRONESTATE_H
#define DRONESTATE_H

#include <BasicLinearAlgebra.h>

using namespace BLA;

struct DroneState {
    BLA::Matrix<3> position;          // X, Y, Z
    BLA::Matrix<3> velocity;          // dX, dY, dZ
    BLA::Matrix<3> orientation;       // Roll (phi), Pitch (θ), Yaw (psi)
    BLA::Matrix<3> angular_velocity;  // Roll rate (dPhi), Pitch rate (dTheta), Yaw rate (dPsi)

    DroneState(); // Constructor to initialize matrices
    void printState() const; // Method to print the current state (for debugging)
};

#endif // DRONESTATE_H