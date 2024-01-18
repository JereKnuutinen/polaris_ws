//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// updateWorkingSetForNewQP.h
//
// Code generation for function 'updateWorkingSetForNewQP'
//

#ifndef UPDATEWORKINGSETFORNEWQP_H
#define UPDATEWORKINGSETFORNEWQP_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
struct i_struct_T;

// Function Declarations
namespace coder {
namespace optim {
namespace coder {
namespace fminconsqp {
namespace internal {
void updateWorkingSetForNewQP(const double xk[113], i_struct_T &WorkingSet,
                              int mIneq, int mNonlinIneq,
                              const ::coder::array<double, 1U> &cIneq,
                              int mNonlinEq, const double cEq[98], int mLB,
                              const double lb[113], int mUB,
                              const double ub[113], int mFixed);

void updateWorkingSetForNewQP(const double xk[113], i_struct_T &WorkingSet,
                              int mIneq, int mNonlinIneq,
                              const ::coder::array<double, 1U> &cIneq, int mEq,
                              int mNonlinEq, const double cEq[98], int mLB,
                              const double lb[113], int mUB,
                              const double ub[113], int mFixed);

} // namespace internal
} // namespace fminconsqp
} // namespace coder
} // namespace optim
} // namespace coder

#endif
// End of code generation (updateWorkingSetForNewQP.h)
