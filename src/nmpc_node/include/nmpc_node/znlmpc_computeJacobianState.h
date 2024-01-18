//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// znlmpc_computeJacobianState.h
//
// Code generation for function 'znlmpc_computeJacobianState'
//

#ifndef ZNLMPC_COMPUTEJACOBIANSTATE_H
#define ZNLMPC_COMPUTEJACOBIANSTATE_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
void znlmpc_computeJacobianState(const double f0[14], double x0[14],
                                 double u0[2], double Jx[196], double Jmv[28]);

}

#endif
// End of code generation (znlmpc_computeJacobianState.h)
