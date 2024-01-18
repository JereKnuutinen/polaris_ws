//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// xaxpy.h
//
// Code generation for function 'xaxpy'
//

#ifndef XAXPY_H
#define XAXPY_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace internal {
namespace blas {
void b_xaxpy(int n, double a, int ix0, double y[196], int iy0);

void b_xaxpy(int n, double a, const double x[70], int ix0, double y[4900],
             int iy0);

void c_xaxpy(int n, double a, const double x[196], int ix0, double y[14],
             int iy0);

void d_xaxpy(int n, double a, const double x[14], int ix0, double y[196],
             int iy0);

void xaxpy(int n, double a, const double x[4900], int ix0, double y[70],
           int iy0);

void xaxpy(int n, double a, int ix0, double y[4900], int iy0);

} // namespace blas
} // namespace internal
} // namespace coder

#endif
// End of code generation (xaxpy.h)
