//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// xgemm.cpp
//
// Code generation for function 'xgemm'
//

// Include files
#include "xgemm.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <cstring>

// Function Definitions
namespace coder {
namespace internal {
namespace blas {
void xgemm(int b_m, int n, int k, const double A[12769], int lda,
           const ::coder::array<double, 2U> &B, int ib0, int ldb,
           ::coder::array<double, 2U> &C, int ldc)
{
  if ((b_m != 0) && (n != 0)) {
    int br;
    int i;
    int i1;
    int lastColC;
    br = ib0;
    lastColC = ldc * (n - 1);
    for (int cr{0}; ldc < 0 ? cr >= lastColC : cr <= lastColC; cr += ldc) {
      i = cr + 1;
      i1 = cr + b_m;
      for (int ic{i}; ic <= i1; ic++) {
        C[ic - 1] = 0.0;
      }
    }
    for (int cr{0}; ldc < 0 ? cr >= lastColC : cr <= lastColC; cr += ldc) {
      int ar;
      ar = -1;
      i = br + k;
      for (int ib{br}; ib < i; ib++) {
        int i2;
        i1 = cr + 1;
        i2 = cr + b_m;
        for (int ic{i1}; ic <= i2; ic++) {
          C[ic - 1] = C[ic - 1] + B[ib - 1] * A[(ar + ic) - cr];
        }
        ar += lda;
      }
      br += ldb;
    }
  }
}

void xgemm(int b_m, int n, int k, const ::coder::array<double, 2U> &A, int ia0,
           int lda, const ::coder::array<double, 2U> &B, int ldb,
           ::coder::array<double, 2U> &C, int ldc)
{
  if ((b_m != 0) && (n != 0)) {
    int br;
    int i;
    int i1;
    int lastColC;
    lastColC = ldc * (n - 1);
    for (int cr{0}; ldc < 0 ? cr >= lastColC : cr <= lastColC; cr += ldc) {
      i = cr + 1;
      i1 = cr + b_m;
      for (int ic{i}; ic <= i1; ic++) {
        C[ic - 1] = 0.0;
      }
    }
    br = -1;
    for (int cr{0}; ldc < 0 ? cr >= lastColC : cr <= lastColC; cr += ldc) {
      int ar;
      ar = ia0;
      i = cr + 1;
      i1 = cr + b_m;
      for (int ic{i}; ic <= i1; ic++) {
        double temp;
        temp = 0.0;
        for (int w{0}; w < k; w++) {
          temp += A[(w + ar) - 1] * B[(w + br) + 1];
        }
        C[ic - 1] = C[ic - 1] + temp;
        ar += lda;
      }
      br += ldb;
    }
  }
}

} // namespace blas
} // namespace internal
} // namespace coder

// End of code generation (xgemm.cpp)
