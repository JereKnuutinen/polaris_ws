//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// evalObjAndConstrAndDerivatives.h
//
// Code generation for function 'evalObjAndConstrAndDerivatives'
//

#ifndef EVALOBJANDCONSTRANDDERIVATIVES_H
#define EVALOBJANDCONSTRANDDERIVATIVES_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace coder {
namespace internal {
class i_stickyStruct;

}
} // namespace coder

// Function Declarations
namespace coder {
namespace optim {
namespace coder {
namespace utils {
namespace ObjNonlinEvaluator {
double evalObjAndConstrAndDerivatives(
    const ::coder::internal::i_stickyStruct &obj, const double x[113],
    ::coder::array<double, 1U> &grad_workspace,
    ::coder::array<double, 1U> &Cineq_workspace, int ineq0,
    double Ceq_workspace[98], int eq0,
    ::coder::array<double, 1U> &JacIneqTrans_workspace, int iJI_col, int ldJI,
    ::coder::array<double, 1U> &JacEqTrans_workspace, int iJE_col, int ldJE,
    int &status);

}
} // namespace utils
} // namespace coder
} // namespace optim
} // namespace coder

#endif
// End of code generation (evalObjAndConstrAndDerivatives.h)
