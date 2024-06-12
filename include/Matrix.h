#ifndef MATRIX_H
#define MATRIX_H

#include <string.h> // For memset and memcpy

class Matrix3x1; // Forward declaration of Matrix3x1

class Matrix3x3 {
public:
  float data[3][3];

  Matrix3x3() { memset(data, 0, sizeof(data)); }

  Matrix3x3(const float arr[3][3]) { memcpy(data, arr, sizeof(data)); }

  Matrix3x3 operator+(const Matrix3x3 &other) const;
  Matrix3x3 operator-(const Matrix3x3 &other) const;
  Matrix3x3 operator*(const Matrix3x3 &other) const;
  Matrix3x1 operator*(const Matrix3x1 &other) const;
  Matrix3x3 transpose() const;
};

class Matrix3x1 {
public:
  float data[3];

  Matrix3x1() { memset(data, 0, sizeof(data)); }

  Matrix3x1(const float arr[3]) { memcpy(data, arr, sizeof(data)); }

  Matrix3x1 operator+(const Matrix3x1 &other) const;
  Matrix3x1 operator-(const Matrix3x1 &other) const;
  Matrix3x1 operator*(float scalar) const;
};

#endif // MATRIX_H