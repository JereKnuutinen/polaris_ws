//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// diag.h
//
// Code generation for function 'diag'
//

#ifndef DIAG_H
#define DIAG_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
void diag(const ::coder::array<double, 2U> &v, ::coder::array<double, 1U> &d);

void diag(double d[4900]);

void diag(const double v[13], double d[169]);

} // namespace coder

#endif
// End of code generation (diag.h)
