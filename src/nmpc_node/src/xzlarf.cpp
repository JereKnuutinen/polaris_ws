//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// xzlarf.cpp
//
// Code generation for function 'xzlarf'
//

// Include files
#include "xzlarf.h"
#include "rt_nonfinite.h"
#include "xgemv.h"
#include "xgerc.h"
#include "coder_array.h"
#include <cstring>

// Function Definitions
namespace coder {
namespace internal {
namespace reflapack {
void xzlarf(int b_m, int n, int iv0, double tau, ::coder::array<double, 2U> &C,
            int ic0, int ldc, ::coder::array<double, 1U> &work)
{
  int lastc;
  int lastv;
  if (tau != 0.0) {
    bool exitg2;
    lastv = b_m;
    lastc = iv0 + b_m;
    while ((lastv > 0) && (C[lastc - 2] == 0.0)) {
      lastv--;
      lastc--;
    }
    lastc = n;
    exitg2 = false;
    while ((!exitg2) && (lastc > 0)) {
      int coltop;
      int exitg1;
      int ia;
      coltop = ic0 + (lastc - 1) * ldc;
      ia = coltop;
      do {
        exitg1 = 0;
        if (ia <= (coltop + lastv) - 1) {
          if (C[ia - 1] != 0.0) {
            exitg1 = 1;
          } else {
            ia++;
          }
        } else {
          lastc--;
          exitg1 = 2;
        }
      } while (exitg1 == 0);
      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
  } else {
    lastv = 0;
    lastc = 0;
  }
  if (lastv > 0) {
    blas::xgemv(lastv, lastc, C, ic0, ldc, C, iv0, work);
    blas::xgerc(lastv, lastc, -tau, iv0, work, C, ic0, ldc);
  }
}

} // namespace reflapack
} // namespace internal
} // namespace coder

// End of code generation (xzlarf.cpp)
