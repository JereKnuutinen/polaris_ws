//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// computeFval.h
//
// Code generation for function 'computeFval'
//

#ifndef COMPUTEFVAL_H
#define COMPUTEFVAL_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
struct g_struct_T;

// Function Declarations
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace Objective {
double computeFval(const g_struct_T &obj, ::coder::array<double, 2U> &workspace,
                   const double H[12769], const ::coder::array<double, 1U> &f,
                   const ::coder::array<double, 1U> &x);

}
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

#endif
// End of code generation (computeFval.h)
