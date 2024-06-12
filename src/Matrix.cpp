#include "Matrix.h"

// Matrix3x1 Implementation

Matrix3x1::Matrix3x1() {
  for (int i = 0; i < 3; i++) {
    data[i] = 0.0f;
  }
}

Matrix3x1::Matrix3x1(const float arr[3]) {
  for (int i = 0; i < 3; i++) {
    data[i] = arr[i];
  }
}

Matrix3x1 Matrix3x1::operator+(const Matrix3x1 &other) const {
  Matrix3x1 result;
  for (int i = 0; i < 3; i++) {
    result.data[i] = data[i] + other.data[i];
  }
  return result;
}

Matrix3x1 Matrix3x1::operator-(const Matrix3x1 &other) const {
  Matrix3x1 result;
  for (int i = 0; i < 3; i++) {
    result.data[i] = data[i] - other.data[i];
  }
  return result;
}

Matrix3x1 Matrix3x1::operator*(float scalar) const {
  Matrix3x1 result;
  for (int i = 0; i < 3; i++) {
    result.data[i] = data[i] * scalar;
  }
  return result;
}

// Matrix3x3 Implementation

Matrix3x3::Matrix3x3() {
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      data[i][j] = 0.0f;
    }
  }
}

Matrix3x3::Matrix3x3(const float arr[3][3]) {
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      data[i][j] = arr[i][j];
    }
  }
}

Matrix3x3 Matrix3x3::operator+(const Matrix3x3 &other) const {
  Matrix3x3 result;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      result.data[i][j] = data[i][j] + other.data[i][j];
    }
  }
  return result;
}

Matrix3x3 Matrix3x3::operator-(const Matrix3x3 &other) const {
  Matrix3x3 result;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      result.data[i][j] = data[i][j] - other.data[i][j];
    }
  }
  return result;
}

Matrix3x3 Matrix3x3::operator*(const Matrix3x3 &other) const {
  Matrix3x3 result;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      result.data[i][j] = 0.0f;
      for (int k = 0; k < 3; k++) {
        result.data[i][j] += data[i][k] * other.data[k][j];
      }
    }
  }
  return result;
}

Matrix3x1 Matrix3x3::operator*(const Matrix3x1 &other) const {
  Matrix3x1 result;
  for (int i = 0; i < 3; i++) {
    result.data[i] = 0.0f;
    for (int j = 0; j < 3; j++) {
      result.data[i] += data[i][j] * other.data[j];
    }
  }
  return result;
}

Matrix3x3 Matrix3x3::transpose() const {
  Matrix3x3 result;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      result.data[i][j] = data[j][i];
    }
  }
  return result;
}