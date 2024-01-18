//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// computeLinearResiduals.h
//
// Code generation for function 'computeLinearResiduals'
//

#ifndef COMPUTELINEARRESIDUALS_H
#define COMPUTELINEARRESIDUALS_H

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
namespace internal {
void computeLinearResiduals(const double x[113], int nVar,
                            ::coder::array<double, 1U> &workspaceIneq,
                            int mLinIneq,
                            const ::coder::array<double, 1U> &AineqT,
                            const ::coder::array<double, 1U> &bineq, int ldAi,
                            double workspaceEq[98], int mLinEq,
                            const ::coder::array<double, 1U> &AeqT, int ldAe);

}
} // namespace fminconsqp
} // namespace coder
} // namespace optim
} // namespace coder

#endif
// End of code generation (computeLinearResiduals.h)
