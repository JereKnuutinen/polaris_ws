//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// computeLambdaLSQ.h
//
// Code generation for function 'computeLambdaLSQ'
//

#ifndef COMPUTELAMBDALSQ_H
#define COMPUTELAMBDALSQ_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
struct e_struct_T;

// Function Declarations
namespace coder {
namespace optim {
namespace coder {
namespace fminconsqp {
namespace stopping {
void computeLambdaLSQ(int nVar, int mConstr, e_struct_T &b_QRManager,
                      const ::coder::array<double, 1U> &ATwset, int ldA,
                      const ::coder::array<double, 1U> &grad,
                      ::coder::array<double, 1U> &lambdaLSQ,
                      ::coder::array<double, 2U> &workspace);

}
} // namespace fminconsqp
} // namespace coder
} // namespace optim
} // namespace coder

#endif
// End of code generation (computeLambdaLSQ.h)
