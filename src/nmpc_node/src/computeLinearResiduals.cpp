//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// computeLinearResiduals.cpp
//
// Code generation for function 'computeLinearResiduals'
//

// Include files
#include "computeLinearResiduals.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <cstring>

// Function Definitions
namespace coder {
namespace optim {
namespace coder {
namespace fminconsqp {
namespace internal {
void computeLinearResiduals(const double x[113], int nVar,
                            ::coder::array<double, 1U> &workspaceIneq,
                            int mLinIneq,
                            const ::coder::array<double, 1U> &AineqT,
                            const ::coder::array<double, 1U> &bineq, int ldAi,
                            double workspaceEq[98], int mLinEq,
                            const ::coder::array<double, 1U> &AeqT, int ldAe)
{
  double c;
  int i;
  int iy;
  int k;
  if (mLinIneq > 0) {
    for (k = 0; k < mLinIneq; k++) {
      workspaceIneq[k] = bineq[k];
    }
    if (nVar != 0) {
      for (iy = 0; iy < mLinIneq; iy++) {
        workspaceIneq[iy] = -workspaceIneq[iy];
      }
      iy = 0;
      k = ldAi * (mLinIneq - 1) + 1;
      for (int iac{1}; ldAi < 0 ? iac >= k : iac <= k; iac += ldAi) {
        c = 0.0;
        i = (iac + nVar) - 1;
        for (int ia{iac}; ia <= i; ia++) {
          c += AineqT[ia - 1] * x[ia - iac];
        }
        workspaceIneq[iy] = workspaceIneq[iy] + c;
        iy++;
      }
    }
  }
  if ((mLinEq > 0) && (nVar != 0)) {
    for (iy = 0; iy < mLinEq; iy++) {
      workspaceEq[iy] = -workspaceEq[iy];
    }
    iy = 0;
    k = ldAe * (mLinEq - 1) + 1;
    for (int iac{1}; ldAe < 0 ? iac >= k : iac <= k; iac += ldAe) {
      c = 0.0;
      i = (iac + nVar) - 1;
      for (int ia{iac}; ia <= i; ia++) {
        c += AeqT[ia - 1] * x[ia - iac];
      }
      workspaceEq[iy] += c;
      iy++;
    }
  }
}

} // namespace internal
} // namespace fminconsqp
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (computeLinearResiduals.cpp)
