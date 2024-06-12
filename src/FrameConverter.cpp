#include "FrameConverter.h"
#include <math.h> // For trigonometric functions

Matrix3x3 FrameConverter::computeRotationMatrix(const Matrix3x1 &orientation) {
  float phi = orientation.data[0];   // Roll
  float theta = orientation.data[1]; // Pitch
  float psi = orientation.data[2];   // Yaw

  float R_x[3][3] = {
      {1, 0, 0}, {0, cos(phi), -sin(phi)}, {0, sin(phi), cos(phi)}};

  float R_y[3][3] = {
      {cos(theta), 0, sin(theta)}, {0, 1, 0}, {-sin(theta), 0, cos(theta)}};

  float R_z[3][3] = {
      {cos(psi), -sin(psi), 0}, {sin(psi), cos(psi), 0}, {0, 0, 1}};

  Matrix3x3 Rx(R_x), Ry(R_y), Rz(R_z);

  return Rz * Ry * Rx;
}

Matrix3x1 FrameConverter::toWorldFrame(const Matrix3x1 &local_vector,
                                       const Matrix3x1 &orientation) {
  Matrix3x3 R = computeRotationMatrix(orientation);
  return R * local_vector;
}

Matrix3x1 FrameConverter::toLocalFrame(const Matrix3x1 &world_vector,
                                       const Matrix3x1 &orientation) {
  Matrix3x3 R = computeRotationMatrix(orientation);
  return R.transpose() * world_vector;
}