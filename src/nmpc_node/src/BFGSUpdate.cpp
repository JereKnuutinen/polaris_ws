//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// BFGSUpdate.cpp
//
// Code generation for function 'BFGSUpdate'
//

// Include files
#include "BFGSUpdate.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <cstring>

// Function Definitions
namespace coder {
namespace optim {
namespace coder {
namespace fminconsqp {
bool BFGSUpdate(int nvar, double Bk[12769],
                const ::coder::array<double, 1U> &sk,
                ::coder::array<double, 1U> &yk,
                ::coder::array<double, 2U> &workspace)
{
  double curvatureS;
  double dotSY;
  double theta;
  int i;
  int i1;
  int ix;
  int k;
  bool success;
  dotSY = 0.0;
  if (nvar >= 1) {
    for (k = 0; k < nvar; k++) {
      dotSY += sk[k] * yk[k];
    }
  }
  if (nvar != 0) {
    for (ix = 0; ix < nvar; ix++) {
      workspace[ix] = 0.0;
    }
    ix = 0;
    i = 113 * (nvar - 1) + 1;
    for (int iac{1}; iac <= i; iac += 113) {
      i1 = (iac + nvar) - 1;
      for (int ia{iac}; ia <= i1; ia++) {
        k = ia - iac;
        workspace[k] = workspace[k] + Bk[ia - 1] * sk[ix];
      }
      ix++;
    }
  }
  curvatureS = 0.0;
  if (nvar >= 1) {
    for (k = 0; k < nvar; k++) {
      curvatureS += sk[k] * workspace[k];
    }
  }
  if (dotSY < 0.2 * curvatureS) {
    theta = 0.8 * curvatureS / (curvatureS - dotSY);
    for (k = 0; k < nvar; k++) {
      yk[k] = theta * yk[k];
    }
    if ((nvar >= 1) && (!(1.0 - theta == 0.0))) {
      ix = nvar - 1;
      for (k = 0; k <= ix; k++) {
        yk[k] = yk[k] + (1.0 - theta) * workspace[k];
      }
    }
    dotSY = 0.0;
    if (nvar >= 1) {
      for (k = 0; k < nvar; k++) {
        dotSY += sk[k] * yk[k];
      }
    }
  }
  if ((curvatureS > 2.2204460492503131E-16) &&
      (dotSY > 2.2204460492503131E-16)) {
    success = true;
  } else {
    success = false;
  }
  if (success) {
    curvatureS = -1.0 / curvatureS;
    if (!(curvatureS == 0.0)) {
      ix = 0;
      for (k = 0; k < nvar; k++) {
        if (workspace[k] != 0.0) {
          theta = workspace[k] * curvatureS;
          i = ix + 1;
          i1 = nvar + ix;
          for (int iac{i}; iac <= i1; iac++) {
            Bk[iac - 1] += workspace[(iac - ix) - 1] * theta;
          }
        }
        ix += 113;
      }
    }
    curvatureS = 1.0 / dotSY;
    if (!(curvatureS == 0.0)) {
      ix = 0;
      for (k = 0; k < nvar; k++) {
        theta = yk[k];
        if (theta != 0.0) {
          theta *= curvatureS;
          i = ix + 1;
          i1 = nvar + ix;
          for (int iac{i}; iac <= i1; iac++) {
            Bk[iac - 1] += yk[(iac - ix) - 1] * theta;
          }
        }
        ix += 113;
      }
    }
  }
  return success;
}

} // namespace fminconsqp
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (BFGSUpdate.cpp)
