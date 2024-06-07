#include "DataProcessor.h"

DataProcessor::DataProcessor() : last_time_(0),
                                 surge_position_(0), surge_velocity_(0), surge_acceleration_(0),
                                 sway_position_(0), sway_velocity_(0), sway_acceleration_(0),
                                 heave_position_(0), heave_velocity_(0), heave_acceleration_(0),
                                 yaw_angle_(0), yaw_rate_(0), yaw_acceleration_(0), yaw_integral_(0),
                                 pitch_angle_(0), pitch_rate_(0), pitch_acceleration_(0), pitch_integral_(0),
                                 roll_angle_(0), roll_rate_(0), roll_acceleration_(0), roll_integral_(0) {}

DataProcessor::~DataProcessor() {}

void DataProcessor::processAccelerometer(int16_t ax, int16_t ay, int16_t az) {
  unsigned long current_time = millis();
  float dt = (current_time - last_time_) / 1000.0; // Convert milliseconds to seconds
  
  if (last_time_ == 0) {
    dt = 0; // Avoid division by zero on the first run
  }

  // Convert raw accelerometer data to acceleration in g's
  surge_acceleration_ = ax / ACCEL_SCALE;
  sway_acceleration_ = ay / ACCEL_SCALE;
  heave_acceleration_ = az / ACCEL_SCALE;

  // Integrate acceleration to get velocity
  surge_velocity_ += surge_acceleration_ * dt;
  sway_velocity_ += sway_acceleration_ * dt;
  heave_velocity_ += heave_acceleration_ * dt;

  // Integrate velocity to get position
  surge_position_ += surge_velocity_ * dt;
  sway_position_ += sway_velocity_ * dt;
  heave_position_ += heave_velocity_ * dt;

  last_time_ = current_time;
}

void DataProcessor::processGyro(int16_t gx, int16_t gy, int16_t gz) {
  unsigned long current_time = millis();
  float dt = (current_time - last_time_) / 1000.0; // Convert milliseconds to seconds
  
  if (last_time_ == 0) {
    dt = 0; // Avoid division by zero on the first run
  }

  // Convert raw gyroscope data to angular velocity in degrees/s
  roll_rate_ = gx / GYRO_SCALE;
  pitch_rate_ = gy / GYRO_SCALE;
  yaw_rate_ = gz / GYRO_SCALE;

  // Integrate angular velocity to get angle
  roll_angle_ += roll_rate_ * dt;
  pitch_angle_ += pitch_rate_ * dt;
  yaw_angle_ += yaw_rate_ * dt;

  // Calculate angular acceleration (rate of change of angular velocity)
  roll_acceleration_ = (roll_rate_ - roll_angle_) / dt;
  pitch_acceleration_ = (pitch_rate_ - pitch_angle_) / dt;
  yaw_acceleration_ = (yaw_rate_ - yaw_angle_) / dt;

  // Update integrals
  roll_integral_ += roll_angle_ * dt;
  pitch_integral_ += pitch_angle_ * dt;
  yaw_integral_ += yaw_angle_ * dt;

  last_time_ = current_time;
}
