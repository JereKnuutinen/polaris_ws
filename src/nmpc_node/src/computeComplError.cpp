//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// computeComplError.cpp
//
// Code generation for function 'computeComplError'
//

// Include files
#include "computeComplError.h"
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
double
computeComplError(const ::coder::array<double, 1U> &fscales_lineq_constraint,
                  const ::coder::array<double, 1U> &fscales_cineq_constraint,
                  const double xCurrent[113], int mIneq,
                  const ::coder::array<double, 1U> &cIneq,
                  const ::coder::array<int, 1U> &finiteLB, int mLB,
                  const double lb[113], const ::coder::array<int, 1U> &finiteUB,
                  int mUB, const double ub[113],
                  const ::coder::array<double, 1U> &lambda, int iL0)
{
  double nlpComplError;
  int mLinIneq;
  int mNonlinIneq;
  nlpComplError = 0.0;
  mLinIneq = fscales_lineq_constraint.size(0) - 1;
  mNonlinIneq = fscales_cineq_constraint.size(0);
  if ((mIneq + mLB) + mUB > 0) {
    double lbDelta;
    double lbLambda;
    int iLineq0;
    for (int idx{0}; idx <= mLinIneq; idx++) {
      lbDelta = lambda[(iL0 + idx) - 1];
      nlpComplError = std::fmax(
          nlpComplError,
          std::fmin(
              std::abs(cIneq[idx] * lbDelta),
              std::fmin(std::abs(cIneq[idx]) / fscales_lineq_constraint[idx],
                        lbDelta * fscales_lineq_constraint[idx])));
    }
    iLineq0 = (iL0 + fscales_lineq_constraint.size(0)) - 2;
    for (int idx{0}; idx < mNonlinIneq; idx++) {
      lbLambda = cIneq[(mLinIneq + idx) + 1];
      lbDelta = lambda[(iLineq0 + idx) + 1];
      nlpComplError = std::fmax(
          nlpComplError,
          std::fmin(
              std::abs(lbLambda * lbDelta),
              std::fmin(std::abs(lbLambda) / fscales_cineq_constraint[idx],
                        lbDelta * fscales_cineq_constraint[idx])));
    }
    mLinIneq = (iL0 + mIneq) - 1;
    mNonlinIneq = mLinIneq + mLB;
    for (int idx{0}; idx < mLB; idx++) {
      lbDelta = xCurrent[finiteLB[idx] - 1] - lb[finiteLB[idx] - 1];
      lbLambda = lambda[mLinIneq + idx];
      nlpComplError = std::fmax(
          nlpComplError, std::fmin(std::abs(lbDelta * lbLambda),
                                   std::fmin(std::abs(lbDelta), lbLambda)));
    }
    for (int idx{0}; idx < mUB; idx++) {
      lbDelta = ub[finiteUB[idx] - 1] - xCurrent[finiteUB[idx] - 1];
      lbLambda = lambda[mNonlinIneq + idx];
      nlpComplError = std::fmax(
          nlpComplError, std::fmin(std::abs(lbDelta * lbLambda),
                                   std::fmin(std::abs(lbDelta), lbLambda)));
    }
  }
  return nlpComplError;
}

} // namespace stopping
} // namespace fminconsqp
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (computeComplError.cpp)
