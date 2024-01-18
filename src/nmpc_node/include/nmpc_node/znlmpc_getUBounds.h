//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// znlmpc_getUBounds.h
//
// Code generation for function 'znlmpc_getUBounds'
//

#ifndef ZNLMPC_GETUBOUNDS_H
#define ZNLMPC_GETUBOUNDS_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
void znlmpc_getUBounds(const double runtimedata_lastMV[2],
                       const double runtimedata_MVMin[14],
                       const double runtimedata_MVMax[14],
                       const double runtimedata_MVRateMin[14],
                       const double runtimedata_MVRateMax[14],
                       ::coder::array<double, 2U> &A,
                       ::coder::array<double, 1U> &Bu);

}

#endif
// End of code generation (znlmpc_getUBounds.h)
