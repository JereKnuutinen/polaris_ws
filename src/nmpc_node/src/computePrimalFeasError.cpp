//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// computePrimalFeasError.cpp
//
// Code generation for function 'computePrimalFeasError'
//

// Include files
#include "computePrimalFeasError.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <cmath>
#include <cstring>

// Function Definitions
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
                              const double ub[113])
{
  double feasError;
  int mEq;
  int mIneq;
  feasError = 0.0;
  mEq = mNonlinEq + mLinEq;
  mIneq = mNonlinIneq + mLinIneq;
  for (int idx{0}; idx < mEq; idx++) {
    feasError = std::fmax(feasError, std::abs(cEq[idx]));
  }
  for (int idx{0}; idx < mIneq; idx++) {
    feasError = std::fmax(feasError, cIneq[idx]);
  }
  for (int idx{0}; idx < mLB; idx++) {
    feasError =
        std::fmax(feasError, lb[finiteLB[idx] - 1] - x[finiteLB[idx] - 1]);
  }
  for (int idx{0}; idx < mUB; idx++) {
    feasError =
        std::fmax(feasError, x[finiteUB[idx] - 1] - ub[finiteUB[idx] - 1]);
  }
  return feasError;
}

} // namespace stopping
} // namespace fminconsqp
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (computePrimalFeasError.cpp)
