//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// mtimes.cpp
//
// Code generation for function 'mtimes'
//

// Include files
#include "mtimes.h"
#include "rt_nonfinite.h"
#include "coder_array.h"

// Function Definitions
namespace coder {
namespace internal {
namespace blas {
void mtimes(const double A[9], const double B[3], double C[3])
{
  double d;
  double d1;
  double d2;
  d = B[0];
  d1 = B[1];
  d2 = B[2];
  for (int i{0}; i < 3; i++) {
    int aoffset;
    aoffset = i * 3;
    C[i] = (A[aoffset] * d + A[aoffset + 1] * d1) + A[aoffset + 2] * d2;
  }
}

void mtimes(const ::coder::array<double, 2U> &A,
            const ::coder::array<double, 2U> &B, ::coder::array<double, 2U> &C)
{
  int inner;
  int mc;
  int nc;
  mc = A.size(0);
  inner = A.size(1);
  nc = B.size(0);
  C.set_size(A.size(0), B.size(0));
  for (int j{0}; j < nc; j++) {
    int coffset;
    coffset = j * mc;
    for (int i{0}; i < mc; i++) {
      C[coffset + i] = 0.0;
    }
    for (int k{0}; k < inner; k++) {
      double bkj;
      int aoffset;
      aoffset = k * A.size(0);
      bkj = B[k * B.size(0) + j];
      for (int i{0}; i < mc; i++) {
        int b_i;
        b_i = coffset + i;
        C[b_i] = C[b_i] + A[aoffset + i] * bkj;
      }
    }
  }
}

} // namespace blas
} // namespace internal
} // namespace coder

// End of code generation (mtimes.cpp)
