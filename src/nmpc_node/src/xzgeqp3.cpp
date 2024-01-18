//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// xzgeqp3.cpp
//
// Code generation for function 'xzgeqp3'
//

// Include files
#include "xzgeqp3.h"
#include "rt_nonfinite.h"
#include "xzlarf.h"
#include "xzlarfg.h"
#include "coder_array.h"
#include <cstring>

// Function Definitions
namespace coder {
namespace internal {
namespace reflapack {
void qrf(::coder::array<double, 2U> &A, int b_m, int n, int nfxd,
         ::coder::array<double, 1U> &tau)
{
  array<double, 1U> work;
  double atmp;
  int ii;
  int lda;
  int mmi;
  lda = A.size(0);
  work.set_size(A.size(1));
  ii = A.size(1);
  for (mmi = 0; mmi < ii; mmi++) {
    work[mmi] = 0.0;
  }
  for (int i{0}; i < nfxd; i++) {
    double d;
    ii = i * lda + i;
    mmi = b_m - i;
    if (i + 1 < b_m) {
      atmp = A[ii];
      d = xzlarfg(mmi, atmp, A, ii + 2);
      tau[i] = d;
      A[ii] = atmp;
    } else {
      d = 0.0;
      tau[i] = 0.0;
    }
    if (i + 1 < n) {
      atmp = A[ii];
      A[ii] = 1.0;
      xzlarf(mmi, (n - i) - 1, ii + 1, d, A, (ii + lda) + 1, lda, work);
      A[ii] = atmp;
    }
  }
}

} // namespace reflapack
} // namespace internal
} // namespace coder

// End of code generation (xzgeqp3.cpp)
