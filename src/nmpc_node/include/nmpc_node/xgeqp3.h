//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// xgeqp3.h
//
// Code generation for function 'xgeqp3'
//

#ifndef XGEQP3_H
#define XGEQP3_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace internal {
namespace lapack {
void xgeqp3(::coder::array<double, 2U> &A, int b_m, int n,
            ::coder::array<int, 1U> &jpvt, ::coder::array<double, 1U> &tau);

}
} // namespace internal
} // namespace coder

#endif
// End of code generation (xgeqp3.h)
