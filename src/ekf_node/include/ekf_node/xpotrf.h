//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// xpotrf.h
//
// Code generation for function 'xpotrf'
//

#ifndef XPOTRF_H
#define XPOTRF_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace internal {
namespace lapack {
int b_xpotrf(double A[196]);

int xpotrf(double A[4900]);

} // namespace lapack
} // namespace internal
} // namespace coder

#endif
// End of code generation (xpotrf.h)
