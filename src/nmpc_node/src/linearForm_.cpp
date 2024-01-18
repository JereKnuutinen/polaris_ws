//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// linearForm_.cpp
//
// Code generation for function 'linearForm_'
//

// Include files
#include "linearForm_.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <cstring>

// Function Definitions
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace Objective {
void linearForm_(bool obj_hasLinear, int obj_nvar,
                 ::coder::array<double, 2U> &workspace, const double H[12769],
                 const ::coder::array<double, 1U> &f,
                 const ::coder::array<double, 1U> &x)
{
  int ix;
  ix = 0;
  if (obj_hasLinear) {
    for (ix = 0; ix < obj_nvar; ix++) {
      workspace[ix] = f[ix];
    }
    ix = 1;
  }
  if (obj_nvar != 0) {
    int i;
    if (ix != 1) {
      for (ix = 0; ix < obj_nvar; ix++) {
        workspace[ix] = 0.0;
      }
    }
    ix = 0;
    i = obj_nvar * (obj_nvar - 1) + 1;
    for (int iac{1}; obj_nvar < 0 ? iac >= i : iac <= i; iac += obj_nvar) {
      double c;
      int i1;
      c = 0.5 * x[ix];
      i1 = (iac + obj_nvar) - 1;
      for (int ia{iac}; ia <= i1; ia++) {
        int i2;
        i2 = ia - iac;
        workspace[i2] = workspace[i2] + H[ia - 1] * c;
      }
      ix++;
    }
  }
}

} // namespace Objective
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (linearForm_.cpp)
