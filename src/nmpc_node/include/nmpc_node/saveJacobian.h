//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// saveJacobian.h
//
// Code generation for function 'saveJacobian'
//

#ifndef SAVEJACOBIAN_H
#define SAVEJACOBIAN_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
struct d_struct_T;

// Function Declarations
namespace coder {
namespace optim {
namespace coder {
namespace fminconsqp {
namespace TrialState {
void saveJacobian(d_struct_T &obj, int nVar, int mIneq,
                  const ::coder::array<double, 1U> &JacCineqTrans, int ineqCol0,
                  int mEq, const ::coder::array<double, 1U> &JacCeqTrans,
                  int eqCol0, int ldJ);

}
} // namespace fminconsqp
} // namespace coder
} // namespace optim
} // namespace coder

#endif
// End of code generation (saveJacobian.h)
