//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// xgemv.h
//
// Code generation for function 'xgemv'
//

#ifndef XGEMV_H
#define XGEMV_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace internal {
namespace blas {
void xgemv(int b_m, int n, const ::coder::array<double, 1U> &A, int lda,
           const ::coder::array<double, 1U> &x, ::coder::array<double, 2U> &y);

void xgemv(int b_m, int n, const ::coder::array<double, 1U> &A, int lda,
           const ::coder::array<double, 1U> &x, ::coder::array<double, 2U> &y,
           int iy0);

void xgemv(int b_m, int n, const ::coder::array<double, 1U> &A, int lda,
           const ::coder::array<double, 2U> &x, ::coder::array<double, 1U> &y);

void xgemv(int b_m, int n, const ::coder::array<double, 1U> &A, int lda,
           const ::coder::array<double, 2U> &x, int ix0,
           ::coder::array<double, 1U> &y);

void xgemv(int b_m, int n, const ::coder::array<double, 1U> &A, int lda,
           const ::coder::array<double, 1U> &x, ::coder::array<double, 1U> &y);

void xgemv(int b_m, int n, const ::coder::array<double, 2U> &A, int ia0,
           int lda, const ::coder::array<double, 2U> &x, int ix0,
           ::coder::array<double, 1U> &y);

void xgemv(int b_m, int n, const ::coder::array<double, 2U> &A, int lda,
           const ::coder::array<double, 1U> &x, ::coder::array<double, 2U> &y);

} // namespace blas
} // namespace internal
} // namespace coder

#endif
// End of code generation (xgemv.h)
