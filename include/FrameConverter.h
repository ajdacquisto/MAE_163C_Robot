#ifndef FRAMECONVERTER_H
#define FRAMECONVERTER_H

#include "Matrix.h"

class FrameConverter {
public:
  static Matrix3x3 computeRotationMatrix(const Matrix3x1 &orientation);
  static Matrix3x1 toWorldFrame(const Matrix3x1 &local_vector,
                                const Matrix3x1 &orientation);
  static Matrix3x1 toLocalFrame(const Matrix3x1 &world_vector,
                                const Matrix3x1 &orientation);
};

#endif // FRAMECONVERTER_H