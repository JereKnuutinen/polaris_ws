//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// linearForm_.h
//
// Code generation for function 'linearForm_'
//

#ifndef LINEARFORM__H
#define LINEARFORM__H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace Objective {
void linearForm_(bool obj_hasLinear, int obj_nvar,
                 ::coder::array<double, 2U> &workspace, const double H[12769],
                 const ::coder::array<double, 1U> &f,
                 const ::coder::array<double, 1U> &x);

}
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

#endif
// End of code generation (linearForm_.h)
