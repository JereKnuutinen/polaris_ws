//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// xgerc.h
//
// Code generation for function 'xgerc'
//

#ifndef XGERC_H
#define XGERC_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace internal {
namespace blas {
void xgerc(int b_m, int n, double alpha1, int ix0,
           const ::coder::array<double, 1U> &y, ::coder::array<double, 2U> &A,
           int ia0, int lda);

}
} // namespace internal
} // namespace coder

#endif
// End of code generation (xgerc.h)
