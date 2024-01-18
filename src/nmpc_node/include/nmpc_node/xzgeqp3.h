//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// xzgeqp3.h
//
// Code generation for function 'xzgeqp3'
//

#ifndef XZGEQP3_H
#define XZGEQP3_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace internal {
namespace reflapack {
void qrf(::coder::array<double, 2U> &A, int b_m, int n, int nfxd,
         ::coder::array<double, 1U> &tau);

}
} // namespace internal
} // namespace coder

#endif
// End of code generation (xzgeqp3.h)
