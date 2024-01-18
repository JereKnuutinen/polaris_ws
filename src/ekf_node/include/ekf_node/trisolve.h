//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// trisolve.h
//
// Code generation for function 'trisolve'
//

#ifndef TRISOLVE_H
#define TRISOLVE_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace internal {
void b_trisolve(const double A[196], double B[980]);

void trisolve(const double A[196], double B[980]);

} // namespace internal
} // namespace coder

#endif
// End of code generation (trisolve.h)
