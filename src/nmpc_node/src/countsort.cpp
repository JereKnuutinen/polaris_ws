//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// countsort.cpp
//
// Code generation for function 'countsort'
//

// Include files
#include "countsort.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <cstring>

// Function Definitions
namespace coder {
namespace optim {
namespace coder {
namespace utils {
void countsort(::coder::array<int, 1U> &x, int xLen,
               ::coder::array<int, 1U> &workspace, int xMin, int xMax)
{
  if ((xLen > 1) && (xMax > xMin)) {
    int idxEnd;
    int idxStart;
    int maxOffset;
    idxStart = xMax - xMin;
    for (int idx{0}; idx <= idxStart; idx++) {
      workspace[idx] = 0;
    }
    maxOffset = idxStart - 1;
    for (int idx{0}; idx < xLen; idx++) {
      idxStart = x[idx] - xMin;
      workspace[idxStart] = workspace[idxStart] + 1;
    }
    for (int idx{2}; idx <= maxOffset + 2; idx++) {
      workspace[idx - 1] = workspace[idx - 1] + workspace[idx - 2];
    }
    idxStart = 1;
    idxEnd = workspace[0];
    for (int idx{0}; idx <= maxOffset; idx++) {
      for (int idxFill{idxStart}; idxFill <= idxEnd; idxFill++) {
        x[idxFill - 1] = idx + xMin;
      }
      idxStart = workspace[idx] + 1;
      idxEnd = workspace[idx + 1];
    }
    for (int idx{idxStart}; idx <= idxEnd; idx++) {
      x[idx - 1] = xMax;
    }
  }
}

} // namespace utils
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (countsort.cpp)
