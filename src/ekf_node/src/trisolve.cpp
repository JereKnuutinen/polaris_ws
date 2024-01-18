//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// trisolve.cpp
//
// Code generation for function 'trisolve'
//

// Include files
#include "trisolve.h"
#include "rt_nonfinite.h"

// Function Definitions
namespace coder {
namespace internal {
void b_trisolve(const double A[196], double B[980])
{
  for (int j{0}; j < 70; j++) {
    int jBcol;
    jBcol = 14 * j;
    for (int k{13}; k >= 0; k--) {
      double d;
      int i;
      int kAcol;
      kAcol = 14 * k;
      i = k + jBcol;
      d = B[i];
      if (d != 0.0) {
        B[i] = d / A[k + kAcol];
        for (int b_i{0}; b_i < k; b_i++) {
          int i1;
          i1 = b_i + jBcol;
          B[i1] -= B[i] * A[b_i + kAcol];
        }
      }
    }
  }
}

void trisolve(const double A[196], double B[980])
{
  for (int j{0}; j < 70; j++) {
    int jBcol;
    jBcol = 14 * j - 1;
    for (int k{0}; k < 14; k++) {
      double d;
      int i;
      int kAcol;
      kAcol = 14 * k - 1;
      i = (k + jBcol) + 1;
      d = B[i];
      if (d != 0.0) {
        int i1;
        B[i] = d / A[(k + kAcol) + 1];
        i1 = k + 2;
        for (int b_i{i1}; b_i < 15; b_i++) {
          int i2;
          i2 = b_i + jBcol;
          B[i2] -= B[i] * A[b_i + kAcol];
        }
      }
    }
  }
}

} // namespace internal
} // namespace coder

// End of code generation (trisolve.cpp)
