#include "Matrix.h"

// Matrix3x3 operator overloads
Matrix3x3 Matrix3x3::operator+(const Matrix3x3 &other) const {
  Matrix3x3 result;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      result.data[i][j] = data[i][j] + other.data[i][j];
    }
  }
  return result;
}

Matrix3x3 Matrix3x3::operator-(const Matrix3x3 &other) const {
  Matrix3x3 result;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      result.data[i][j] = data[i][j] - other.data[i][j];
    }
  }
  return result;
}

Matrix3x3 Matrix3x3::operator*(const Matrix3x3 &other) const {
  Matrix3x3 result;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      result.data[i][j] = 0;
      for (int k = 0; k < 3; ++k) {
        result.data[i][j] += data[i][k] * other.data[k][j];
      }
    }
  }
  return result;
}

Matrix3x1 Matrix3x3::operator*(const Matrix3x1 &other) const {
  Matrix3x1 result;
  for (int i = 0; i < 3; ++i) {
    result.data[i] = 0;
    for (int j = 0; j < 3; ++j) {
      result.data[i] += data[i][j] * other.data[j];
    }
  }
  return result;
}

Matrix3x3 Matrix3x3::transpose() const {
  Matrix3x3 result;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      result.data[i][j] = data[j][i];
    }
  }
  return result;
}

// Matrix3x1 operator overloads
Matrix3x1 Matrix3x1::operator+(const Matrix3x1 &other) const {
  Matrix3x1 result;
  for (int i = 0; i < 3; ++i) {
    result.data[i] = data[i] + other.data[i];
  }
  return result;
}

Matrix3x1 Matrix3x1::operator-(const Matrix3x1 &other) const {
  Matrix3x1 result;
  for (int i = 0; i < 3; ++i) {
    result.data[i] = data[i] - other.data[i];
  }
  return result;
}

Matrix3x1 Matrix3x1::operator*(float scalar) const {
  Matrix3x1 result;
  for (int i = 0; i < 3; ++i) {
    result.data[i] = data[i] * scalar;
  }
  return result;
}