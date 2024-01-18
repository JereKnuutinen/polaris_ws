//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// computeMeritFcn.h
//
// Code generation for function 'computeMeritFcn'
//

#ifndef COMPUTEMERITFCN_H
#define COMPUTEMERITFCN_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace optim {
namespace coder {
namespace fminconsqp {
namespace MeritFunction {
double computeMeritFcn(double obj_penaltyParam, double fval,
                       const ::coder::array<double, 1U> &Cineq_workspace,
                       int mIneq, const double Ceq_workspace[98], int mEq,
                       bool evalWellDefined);

}
} // namespace fminconsqp
} // namespace coder
} // namespace optim
} // namespace coder

#endif
// End of code generation (computeMeritFcn.h)
