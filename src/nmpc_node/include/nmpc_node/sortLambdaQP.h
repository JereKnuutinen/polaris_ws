//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// sortLambdaQP.h
//
// Code generation for function 'sortLambdaQP'
//

#ifndef SORTLAMBDAQP_H
#define SORTLAMBDAQP_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace parseoutput {
void sortLambdaQP(::coder::array<double, 1U> &lambda,
                  int WorkingSet_nActiveConstr, const int WorkingSet_sizes[5],
                  const int WorkingSet_isActiveIdx[6],
                  const ::coder::array<int, 1U> &WorkingSet_Wid,
                  const ::coder::array<int, 1U> &WorkingSet_Wlocalidx,
                  ::coder::array<double, 2U> &workspace);

}
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

#endif
// End of code generation (sortLambdaQP.h)
