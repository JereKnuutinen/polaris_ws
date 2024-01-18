//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// evalObjAndConstr.h
//
// Code generation for function 'evalObjAndConstr'
//

#ifndef EVALOBJANDCONSTR_H
#define EVALOBJANDCONSTR_H

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
double evalObjAndConstr(const ::coder::internal::i_stickyStruct &obj,
                        const double x[113],
                        ::coder::array<double, 1U> &Cineq_workspace, int ineq0,
                        double Ceq_workspace[98], int eq0, int &status);

}
} // namespace utils
} // namespace coder
} // namespace optim
} // namespace coder

#endif
// End of code generation (evalObjAndConstr.h)
