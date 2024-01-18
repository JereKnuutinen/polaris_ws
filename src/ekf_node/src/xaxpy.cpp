//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// xaxpy.cpp
//
// Code generation for function 'xaxpy'
//

// Include files
#include "xaxpy.h"
#include "rt_nonfinite.h"

// Function Definitions
namespace coder {
namespace internal {
namespace blas {
void b_xaxpy(int n, double a, int ix0, double y[196], int iy0)
{
  if ((n >= 1) && (!(a == 0.0))) {
    int i;
    i = n - 1;
    for (int k{0}; k <= i; k++) {
      int i1;
      i1 = (iy0 + k) - 1;
      y[i1] += a * y[(ix0 + k) - 1];
    }
  }
}

void b_xaxpy(int n, double a, const double x[70], int ix0, double y[4900],
             int iy0)
{
  if ((n >= 1) && (!(a == 0.0))) {
    int i;
    i = n - 1;
    for (int k{0}; k <= i; k++) {
      int i1;
      i1 = (iy0 + k) - 1;
      y[i1] += a * x[(ix0 + k) - 1];
    }
  }
}

void c_xaxpy(int n, double a, const double x[196], int ix0, double y[14],
             int iy0)
{
  if ((n >= 1) && (!(a == 0.0))) {
    int i;
    i = n - 1;
    for (int k{0}; k <= i; k++) {
      int i1;
      i1 = (iy0 + k) - 1;
      y[i1] += a * x[(ix0 + k) - 1];
    }
  }
}

void d_xaxpy(int n, double a, const double x[14], int ix0, double y[196],
             int iy0)
{
  if ((n >= 1) && (!(a == 0.0))) {
    int i;
    i = n - 1;
    for (int k{0}; k <= i; k++) {
      int i1;
      i1 = (iy0 + k) - 1;
      y[i1] += a * x[(ix0 + k) - 1];
    }
  }
}

void xaxpy(int n, double a, const double x[4900], int ix0, double y[70],
           int iy0)
{
  if ((n >= 1) && (!(a == 0.0))) {
    int i;
    i = n - 1;
    for (int k{0}; k <= i; k++) {
      int i1;
      i1 = (iy0 + k) - 1;
      y[i1] += a * x[(ix0 + k) - 1];
    }
  }
}

void xaxpy(int n, double a, int ix0, double y[4900], int iy0)
{
  if ((n >= 1) && (!(a == 0.0))) {
    int i;
    i = n - 1;
    for (int k{0}; k <= i; k++) {
      int i1;
      i1 = (iy0 + k) - 1;
      y[i1] += a * y[(ix0 + k) - 1];
    }
  }
}

} // namespace blas
} // namespace internal
} // namespace coder

// End of code generation (xaxpy.cpp)
