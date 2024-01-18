//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// countsort.h
//
// Code generation for function 'countsort'
//

#ifndef COUNTSORT_H
#define COUNTSORT_H

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
void countsort(::coder::array<int, 1U> &x, int xLen,
               ::coder::array<int, 1U> &workspace, int xMin, int xMax);

}
} // namespace coder
} // namespace optim
} // namespace coder

#endif
// End of code generation (countsort.h)
