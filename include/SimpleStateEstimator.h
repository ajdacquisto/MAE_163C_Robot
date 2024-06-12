#ifndef SIMPLESTATEESTIMATOR_H
#define SIMPLESTATEESTIMATOR_H

#include "DroneState.h"
#include "Matrix.h"
#include "IMUController.h"

class SimpleStateEstimator {
private:
    IMUController &imu;
    float delta_t;
    DroneState state;

public:
    SimpleStateEstimator(IMUController &imuController, float dt);
    
    void initialize();
    void update();
    DroneState getState() const;
};

#endif // SIMPLESTATEESTIMATOR_H