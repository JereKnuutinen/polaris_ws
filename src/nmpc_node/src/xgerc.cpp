//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// xgerc.cpp
//
// Code generation for function 'xgerc'
//

// Include files
#include "xgerc.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <cstring>

// Function Definitions
namespace coder {
namespace internal {
namespace blas {
void xgerc(int b_m, int n, double alpha1, int ix0,
           const ::coder::array<double, 1U> &y, ::coder::array<double, 2U> &A,
           int ia0, int lda)
{
  if (!(alpha1 == 0.0)) {
    int jA;
    jA = ia0;
    for (int j{0}; j < n; j++) {
      if (y[j] != 0.0) {
        double temp;
        int i;
        temp = y[j] * alpha1;
        i = b_m + jA;
        for (int ijA{jA}; ijA < i; ijA++) {
          A[ijA - 1] = A[ijA - 1] + A[((ix0 + ijA) - jA) - 1] * temp;
        }
      }
      jA += lda;
    }
  }
}

} // namespace blas
} // namespace internal
} // namespace coder

// End of code generation (xgerc.cpp)
