//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// checkVectorNonFinite.h
//
// Code generation for function 'checkVectorNonFinite'
//

#ifndef CHECKVECTORNONFINITE_H
#define CHECKVECTORNONFINITE_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace optim {
namespace coder {
namespace utils {
namespace ObjNonlinEvaluator {
namespace internal {
int checkVectorNonFinite(int N, const ::coder::array<double, 1U> &vec, int iv0);

int checkVectorNonFinite(const double vec[98], int iv0);

} // namespace internal
} // namespace ObjNonlinEvaluator
} // namespace utils
} // namespace coder
} // namespace optim
} // namespace coder

#endif
// End of code generation (checkVectorNonFinite.h)
