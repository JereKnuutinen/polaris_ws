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
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace internal {
namespace lapack {
int xpotrf(int n, ::coder::array<double, 2U> &A, int lda);

}
} // namespace internal
} // namespace coder

#endif
// End of code generation (xpotrf.h)
