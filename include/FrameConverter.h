#ifndef FRAMECONVERTER_H
#define FRAMECONVERTER_H

#include <BasicLinearAlgebra.h>

using namespace BLA;

class FrameConverter {
public:
  static Matrix<3, 3> computeRotationMatrix(const Matrix<3> &orientation);
  static Matrix<3> toWorldFrame(const Matrix<3> &local_vector,
                                const Matrix<3> &orientation);
  static Matrix<3> toLocalFrame(const Matrix<3> &world_vector,
                                const Matrix<3> &orientation);
};

#endif // FRAMECONVERTER_H