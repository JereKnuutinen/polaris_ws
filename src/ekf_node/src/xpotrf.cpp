//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// xpotrf.cpp
//
// Code generation for function 'xpotrf'
//

// Include files
#include "xpotrf.h"
#include "EKF_Node_rtwutil.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions
namespace coder {
namespace internal {
namespace lapack {
int b_xpotrf(double A[196])
{
  int info;
  int j;
  bool exitg1;
  info = 0;
  j = 0;
  exitg1 = false;
  while ((!exitg1) && (j < 14)) {
    double c;
    double ssq;
    int idxA1j;
    int idxAjj;
    int k;
    idxA1j = j * 14;
    idxAjj = idxA1j + j;
    ssq = 0.0;
    if (j >= 1) {
      for (k = 0; k < j; k++) {
        c = A[idxA1j + k];
        ssq += c * c;
      }
    }
    ssq = A[idxAjj] - ssq;
    if (ssq > 0.0) {
      ssq = std::sqrt(ssq);
      A[idxAjj] = ssq;
      if (j + 1 < 14) {
        int i;
        int idxAjjp1;
        k = idxA1j + 15;
        idxAjjp1 = idxAjj + 15;
        if (j != 0) {
          i = (idxA1j + 14 * (12 - j)) + 15;
          for (int iac{k}; iac <= i; iac += 14) {
            int i1;
            c = 0.0;
            i1 = (iac + j) - 1;
            for (int ia{iac}; ia <= i1; ia++) {
              c += A[ia - 1] * A[(idxA1j + ia) - iac];
            }
            i1 =
                (idxAjj + div_nde_s32_floor((iac - idxA1j) - 15, 14) * 14) + 14;
            A[i1] -= c;
          }
        }
        ssq = 1.0 / ssq;
        i = (idxAjj + 14 * (12 - j)) + 15;
        for (k = idxAjjp1; k <= i; k += 14) {
          A[k - 1] *= ssq;
        }
      }
      j++;
    } else {
      A[idxAjj] = ssq;
      info = j + 1;
      exitg1 = true;
    }
  }
  return info;
}

int xpotrf(double A[4900])
{
  int info;
  int j;
  bool exitg1;
  info = 0;
  j = 0;
  exitg1 = false;
  while ((!exitg1) && (j < 70)) {
    double c;
    double ssq;
    int idxA1j;
    int idxAjj;
    int k;
    idxA1j = j * 70;
    idxAjj = idxA1j + j;
    ssq = 0.0;
    if (j >= 1) {
      for (k = 0; k < j; k++) {
        c = A[idxA1j + k];
        ssq += c * c;
      }
    }
    ssq = A[idxAjj] - ssq;
    if (ssq > 0.0) {
      ssq = std::sqrt(ssq);
      A[idxAjj] = ssq;
      if (j + 1 < 70) {
        int i;
        int idxAjjp1;
        k = idxA1j + 71;
        idxAjjp1 = idxAjj + 71;
        if (j != 0) {
          i = (idxA1j + 70 * (68 - j)) + 71;
          for (int iac{k}; iac <= i; iac += 70) {
            int i1;
            c = 0.0;
            i1 = (iac + j) - 1;
            for (int ia{iac}; ia <= i1; ia++) {
              c += A[ia - 1] * A[(idxA1j + ia) - iac];
            }
            i1 =
                (idxAjj + div_nde_s32_floor((iac - idxA1j) - 71, 70) * 70) + 70;
            A[i1] -= c;
          }
        }
        ssq = 1.0 / ssq;
        i = (idxAjj + 70 * (68 - j)) + 71;
        for (k = idxAjjp1; k <= i; k += 70) {
          A[k - 1] *= ssq;
        }
      }
      j++;
    } else {
      A[idxAjj] = ssq;
      info = j + 1;
      exitg1 = true;
    }
  }
  return info;
}

} // namespace lapack
} // namespace internal
} // namespace coder

// End of code generation (xpotrf.cpp)
