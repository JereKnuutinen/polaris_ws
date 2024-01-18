//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// xswap.h
//
// Code generation for function 'xswap'
//

#ifndef XSWAP_H
#define XSWAP_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace internal {
namespace blas {
void b_xswap(double x[196], int ix0, int iy0);

void xswap(double x[4900], int ix0, int iy0);

} // namespace blas
} // namespace internal
} // namespace coder

#endif
// End of code generation (xswap.h)
