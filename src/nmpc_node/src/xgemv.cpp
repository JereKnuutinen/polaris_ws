//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// xgemv.cpp
//
// Code generation for function 'xgemv'
//

// Include files
#include "xgemv.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <cstring>

// Function Definitions
namespace coder {
namespace internal {
namespace blas {
void xgemv(int b_m, int n, const ::coder::array<double, 1U> &A, int lda,
           const ::coder::array<double, 1U> &x, ::coder::array<double, 2U> &y)
{
  if ((b_m != 0) && (n != 0)) {
    int i;
    int iy;
    for (iy = 0; iy < n; iy++) {
      y[iy] = -y[iy];
    }
    iy = 0;
    i = lda * (n - 1) + 1;
    for (int iac{1}; lda < 0 ? iac >= i : iac <= i; iac += lda) {
      double c;
      int i1;
      c = 0.0;
      i1 = (iac + b_m) - 1;
      for (int ia{iac}; ia <= i1; ia++) {
        c += A[ia - 1] * x[ia - iac];
      }
      y[iy] = y[iy] + c;
      iy++;
    }
  }
}

void xgemv(int b_m, int n, const ::coder::array<double, 1U> &A, int lda,
           const ::coder::array<double, 1U> &x, ::coder::array<double, 2U> &y,
           int iy0)
{
  if ((b_m != 0) && (n != 0)) {
    int iy;
    int iyend;
    iyend = (iy0 + n) - 1;
    for (iy = iy0; iy <= iyend; iy++) {
      y[iy - 1] = 0.0;
    }
    iy = iy0 - 1;
    iyend = lda * (n - 1) + 1;
    for (int iac{1}; lda < 0 ? iac >= iyend : iac <= iyend; iac += lda) {
      double c;
      int i;
      c = 0.0;
      i = (iac + b_m) - 1;
      for (int ia{iac}; ia <= i; ia++) {
        c += A[ia - 1] * x[ia - iac];
      }
      y[iy] = y[iy] + c;
      iy++;
    }
  }
}

void xgemv(int b_m, int n, const ::coder::array<double, 1U> &A, int lda,
           const ::coder::array<double, 2U> &x, ::coder::array<double, 1U> &y)
{
  if ((b_m != 0) && (n != 0)) {
    int i;
    int iy;
    for (iy = 0; iy < n; iy++) {
      y[iy] = -y[iy];
    }
    iy = 0;
    i = lda * (n - 1) + 1;
    for (int iac{1}; lda < 0 ? iac >= i : iac <= i; iac += lda) {
      double c;
      int i1;
      c = 0.0;
      i1 = (iac + b_m) - 1;
      for (int ia{iac}; ia <= i1; ia++) {
        c += A[ia - 1] * x[ia - iac];
      }
      y[iy] = y[iy] + c;
      iy++;
    }
  }
}

void xgemv(int b_m, int n, const ::coder::array<double, 1U> &A, int lda,
           const ::coder::array<double, 2U> &x, int ix0,
           ::coder::array<double, 1U> &y)
{
  if ((b_m != 0) && (n != 0)) {
    int i;
    int iy;
    for (iy = 0; iy < n; iy++) {
      y[iy] = -y[iy];
    }
    iy = 0;
    i = lda * (n - 1) + 1;
    for (int iac{1}; lda < 0 ? iac >= i : iac <= i; iac += lda) {
      double c;
      int i1;
      c = 0.0;
      i1 = (iac + b_m) - 1;
      for (int ia{iac}; ia <= i1; ia++) {
        c += A[ia - 1] * x[((ix0 + ia) - iac) - 1];
      }
      y[iy] = y[iy] + c;
      iy++;
    }
  }
}

void xgemv(int b_m, int n, const ::coder::array<double, 1U> &A, int lda,
           const ::coder::array<double, 1U> &x, ::coder::array<double, 1U> &y)
{
  if ((b_m != 0) && (n != 0)) {
    int i;
    int iy;
    for (iy = 0; iy < n; iy++) {
      y[iy] = -y[iy];
    }
    iy = 0;
    i = lda * (n - 1) + 1;
    for (int iac{1}; lda < 0 ? iac >= i : iac <= i; iac += lda) {
      double c;
      int i1;
      c = 0.0;
      i1 = (iac + b_m) - 1;
      for (int ia{iac}; ia <= i1; ia++) {
        c += A[ia - 1] * x[ia - iac];
      }
      y[iy] = y[iy] + c;
      iy++;
    }
  }
}

void xgemv(int b_m, int n, const ::coder::array<double, 2U> &A, int ia0,
           int lda, const ::coder::array<double, 2U> &x, int ix0,
           ::coder::array<double, 1U> &y)
{
  if ((b_m != 0) && (n != 0)) {
    int i;
    int iy;
    for (iy = 0; iy < n; iy++) {
      y[iy] = 0.0;
    }
    iy = 0;
    i = ia0 + lda * (n - 1);
    for (int iac{ia0}; lda < 0 ? iac >= i : iac <= i; iac += lda) {
      double c;
      int i1;
      c = 0.0;
      i1 = (iac + b_m) - 1;
      for (int ia{iac}; ia <= i1; ia++) {
        c += A[ia - 1] * x[((ix0 + ia) - iac) - 1];
      }
      y[iy] = y[iy] + c;
      iy++;
    }
  }
}

void xgemv(int b_m, int n, const ::coder::array<double, 2U> &A, int lda,
           const ::coder::array<double, 1U> &x, ::coder::array<double, 2U> &y)
{
  if ((b_m != 0) && (n != 0)) {
    int i;
    int iy;
    for (iy = 0; iy < n; iy++) {
      y[iy] = 0.0;
    }
    iy = 0;
    i = lda * (n - 1) + 1;
    for (int iac{1}; lda < 0 ? iac >= i : iac <= i; iac += lda) {
      double c;
      int i1;
      c = 0.0;
      i1 = (iac + b_m) - 1;
      for (int ia{iac}; ia <= i1; ia++) {
        c += A[ia - 1] * x[ia - iac];
      }
      y[iy] = y[iy] + c;
      iy++;
    }
  }
}

} // namespace blas
} // namespace internal
} // namespace coder

// End of code generation (xgemv.cpp)
