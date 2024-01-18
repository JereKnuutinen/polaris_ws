//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// computeComplError.h
//
// Code generation for function 'computeComplError'
//

#ifndef COMPUTECOMPLERROR_H
#define COMPUTECOMPLERROR_H

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
double
computeComplError(const ::coder::array<double, 1U> &fscales_lineq_constraint,
                  const ::coder::array<double, 1U> &fscales_cineq_constraint,
                  const double xCurrent[113], int mIneq,
                  const ::coder::array<double, 1U> &cIneq,
                  const ::coder::array<int, 1U> &finiteLB, int mLB,
                  const double lb[113], const ::coder::array<int, 1U> &finiteUB,
                  int mUB, const double ub[113],
                  const ::coder::array<double, 1U> &lambda, int iL0);

}
} // namespace fminconsqp
} // namespace coder
} // namespace optim
} // namespace coder

#endif
// End of code generation (computeComplError.h)
