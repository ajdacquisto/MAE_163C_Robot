#ifndef MATRIX_H
#define MATRIX_H

class Matrix3x1;  // Forward declaration

class Matrix3x3 {
public:
    float data[3][3];

    Matrix3x3();
    Matrix3x3(float init_data[3][3]);
    Matrix3x3 operator+(const Matrix3x3 &other) const;
    Matrix3x3 operator-(const Matrix3x3 &other) const;
    Matrix3x3 operator*(const Matrix3x3 &other) const;
    Matrix3x1 operator*(const Matrix3x1 &other) const;
    Matrix3x3 transpose() const;
};

class Matrix3x1 {
public:
    float data[3];

    Matrix3x1();
    Matrix3x1(float init_data[3]);
    Matrix3x1 operator+(const Matrix3x1 &other) const;
    Matrix3x1 operator-(const Matrix3x1 &other) const;
    Matrix3x1 operator*(float scalar) const;
};

#endif // MATRIX_H