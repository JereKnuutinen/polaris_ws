//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// computeGradLag.h
//
// Code generation for function 'computeGradLag'
//

#ifndef COMPUTEGRADLAG_H
#define COMPUTEGRADLAG_H

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
void computeGradLag(::coder::array<double, 1U> &workspace, int ldA, int nVar,
                    const ::coder::array<double, 1U> &grad, int mIneq,
                    const ::coder::array<double, 1U> &AineqTrans, int mEq,
                    const ::coder::array<double, 1U> &AeqTrans,
                    const ::coder::array<int, 1U> &finiteFixed, int mFixed,
                    const ::coder::array<int, 1U> &finiteLB, int mLB,
                    const ::coder::array<int, 1U> &finiteUB, int mUB,
                    const ::coder::array<double, 1U> &lambda);

void computeGradLag(::coder::array<double, 2U> &workspace, int ldA, int nVar,
                    const ::coder::array<double, 1U> &grad, int mIneq,
                    const ::coder::array<double, 1U> &AineqTrans, int mEq,
                    const ::coder::array<double, 1U> &AeqTrans,
                    const ::coder::array<int, 1U> &finiteFixed, int mFixed,
                    const ::coder::array<int, 1U> &finiteLB, int mLB,
                    const ::coder::array<int, 1U> &finiteUB, int mUB,
                    const ::coder::array<double, 1U> &lambda);

} // namespace stopping
} // namespace fminconsqp
} // namespace coder
} // namespace optim
} // namespace coder

#endif
// End of code generation (computeGradLag.h)
