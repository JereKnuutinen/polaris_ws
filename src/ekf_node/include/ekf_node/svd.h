//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// svd.h
//
// Code generation for function 'svd'
//

#ifndef SVD_H
#define SVD_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace internal {
void b_svd(const double A[196], double U[196], double s[14], double V[196]);

void svd(const double A[4900], double U[4900], double s[70], double V[4900]);

} // namespace internal
} // namespace coder

#endif
// End of code generation (svd.h)
