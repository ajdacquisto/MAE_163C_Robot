#ifndef DATAPROCESSOR_H
#define DATAPROCESSOR_H

// Include any necessary headers
#include <Arduino.h>

// Define your class for data processing

class DataProcessor {
public:
  DataProcessor();  // Constructor
  ~DataProcessor(); // Destructor

  // Declare public member functions
  void processAccelerometer(int16_t ax, int16_t ay, int16_t az);
  void processGyro(int16_t gx, int16_t gy, int16_t gz);

private:
  // Declare private member variables and functions
  unsigned long last_time_;

  // Surge: This is the linear acceleration along the longitudinal axis
  // (X-axis), which runs from the nose to the tail of the aircraft. It
  // represents forward and backward motion. Positive surge means the aircraft
  // is accelerating forward, while negative surge means it is accelerating
  // backward.
  float surge_position_;     // Position along X-axis
  float surge_velocity_;     // Velocity along X-axis
  float surge_acceleration_; // Acceleration along X-axis

  // Sway: This is the linear acceleration along the lateral axis (Y-axis),
  // which runs from wingtip to wingtip of the aircraft. It represents
  // side-to-side motion. Positive sway means the aircraft is accelerating to
  // the right (starboard), while negative sway means it is accelerating to the
  // left (port).
  float sway_position_;     // Position along Y-axis
  float sway_velocity_;     // Velocity along Y-axis
  float sway_acceleration_; // Acceleration along Y-axis

  // Heave: This is the linear acceleration along the vertical axis (Z-axis),
  // which runs from the bottom to the top of the aircraft. It represents upward
  // and downward motion. Positive heave means the aircraft is accelerating
  // upward, while negative heave means it is accelerating downward.
  float heave_position_;     // Position along Z-axis
  float heave_velocity_;     // Velocity along Z-axis
  float heave_acceleration_; // Acceleration along Z-axis

  // Yaw: This is the rotation around the vertical axis (Z-axis). It controls
  // the left and right movement of the drone's nose, much like turning a car
  // to the left or right. Positive yaw means the nose turns to the right
  // (clockwise when viewed from above), and negative yaw means the nose turns
  // to the left (counterclockwise when viewed from above).
  float yaw_angle_;        // Psi
  float yaw_rate_;         // dPsi/dt
  float yaw_acceleration_; // d^2Psi/dt^2
  float yaw_integral_;

  // Pitch: This is the rotation around the lateral axis (Y-axis), which runs
  // from wing to wing. It controls the up and down movement of the drone's
  // nose. Positive pitch means the nose points upward (nose up), and negative
  // pitch means it points downward (nose down).
  float pitch_angle_;        // Theta
  float pitch_rate_;         // dTheta/dt
  float pitch_acceleration_; // d^2Theta/dt^2
  float pitch_integral_;

  // Roll: This is the rotation around the longitudinal axis (X-axis), which
  // runs from the nose to the tail of the drone. It controls the tilting of the
  // drone to the left or right. Positive roll means the right wing dips (right
  // roll, clockwise when viewed from the rear), and negative roll means the
  // left wing dips (left roll, counterclockwise when viewed from the rear).
  float roll_angle_;        // Phi
  float roll_rate_;         // dPhi/dt
  float roll_acceleration_; // d^2Phi/dt^2
  float roll_integral_;

  // Define any necessary constants
  const float ACCEL_SCALE = 16384.0; // Scale factor for accelerometer data
                                     // (depends on your IMU settings)
  const float GYRO_SCALE =
      131.0; // Scale factor for gyroscope data (depends on your IMU settings)
  const float DT = 0.01; // Time step for integration (adjust as necessary)
};

#endif // DATAPROCESSOR_H
