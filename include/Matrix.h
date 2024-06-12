#ifndef MATRIX_H
#define MATRIX_H

class Matrix3x1 {
public:
  float data[3];

  Matrix3x1();
  Matrix3x1(const float arr[3]);

  Matrix3x1 operator+(const Matrix3x1 &other) const;
  Matrix3x1 operator-(const Matrix3x1 &other) const;
  Matrix3x1 operator*(float scalar) const;
};

class Matrix3x3 {
public:
  float data[3][3];

  Matrix3x3();
  Matrix3x3(const float arr[3][3]);

  Matrix3x3 operator+(const Matrix3x3 &other) const;
  Matrix3x3 operator-(const Matrix3x3 &other) const;
  Matrix3x3 operator*(const Matrix3x3 &other) const;
  Matrix3x1 operator*(const Matrix3x1 &other) const;
  Matrix3x3 transpose() const;
};

#endif // MATRIX_H