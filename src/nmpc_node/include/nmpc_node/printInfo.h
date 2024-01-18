//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// printInfo.h
//
// Code generation for function 'printInfo'
//

#ifndef PRINTINFO_H
#define PRINTINFO_H

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
namespace display {
void printInfo(bool newBlocking, int PROBLEM_TYPE, double alpha,
               double stepNorm, int activeConstrChangedType,
               int localActiveConstrIdx, int activeSetChangeID,
               double solution_fstar, double solution_firstorderopt,
               double solution_maxConstr, int solution_iterations,
               const ::coder::array<int, 1U> &workingset_indexLB,
               const ::coder::array<int, 1U> &workingset_indexUB,
               int workingset_nActiveConstr);

}
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

#endif
// End of code generation (printInfo.h)
