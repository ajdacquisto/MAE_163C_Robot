#include "FrameConverter.h"
#include <math.h> // For trigonometric functions

Matrix<3, 3>
FrameConverter::computeRotationMatrix(const Matrix<3> &orientation) {
  float phi = orientation(0);   // Roll
  float theta = orientation(1); // Pitch
  float psi = orientation(2);   // Yaw

  Matrix<3, 3> R_x = {1, 0, 0, 0, cos(phi), -sin(phi), 0, sin(phi), cos(phi)};

  Matrix<3, 3> R_y = {cos(theta), 0,           sin(theta), 0,         1,
                      0,          -sin(theta), 0,          cos(theta)};

  Matrix<3, 3> R_z = {cos(psi), -sin(psi), 0, sin(psi), cos(psi), 0, 0, 0, 1};

  return R_z * R_y * R_x;
}

Matrix<3> FrameConverter::toWorldFrame(const Matrix<3> &local_vector,
                                       const Matrix<3> &orientation) {
  Matrix<3, 3> R = computeRotationMatrix(orientation);
  return R * local_vector;
}

Matrix<3> FrameConverter::toLocalFrame(const Matrix<3> &world_vector,
                                       const Matrix<3> &orientation) {
  Matrix<3, 3> R = computeRotationMatrix(orientation);
  return ~R * world_vector;
}