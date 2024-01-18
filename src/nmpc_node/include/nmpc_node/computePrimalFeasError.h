//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// computePrimalFeasError.h
//
// Code generation for function 'computePrimalFeasError'
//

#ifndef COMPUTEPRIMALFEASERROR_H
#define COMPUTEPRIMALFEASERROR_H

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
namespace stopping {
double computePrimalFeasError(const double x[113], int mLinIneq,
                              int mNonlinIneq,
                              const ::coder::array<double, 1U> &cIneq,
                              int mLinEq, int mNonlinEq, const double cEq[98],
                              const ::coder::array<int, 1U> &finiteLB, int mLB,
                              const double lb[113],
                              const ::coder::array<int, 1U> &finiteUB, int mUB,
                              const double ub[113]);

}
} // namespace fminconsqp
} // namespace coder
} // namespace optim
} // namespace coder

#endif
// End of code generation (computePrimalFeasError.h)
