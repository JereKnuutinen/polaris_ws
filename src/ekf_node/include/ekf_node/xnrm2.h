//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// xnrm2.h
//
// Code generation for function 'xnrm2'
//

#ifndef XNRM2_H
#define XNRM2_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace internal {
namespace blas {
double b_xnrm2(int n, const double x[70], int ix0);

double c_xnrm2(int n, const double x[196], int ix0);

double d_xnrm2(int n, const double x[14], int ix0);

double e_xnrm2(int n, const double x[1176], int ix0);

double f_xnrm2(int n, const double x[5880], int ix0);

double g_xnrm2(int n, const double x[9800], int ix0);

double xnrm2(int n, const double x[4900], int ix0);

} // namespace blas
} // namespace internal
} // namespace coder

#endif
// End of code generation (xnrm2.h)
