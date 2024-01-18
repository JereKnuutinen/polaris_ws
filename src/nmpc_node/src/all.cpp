//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// all.cpp
//
// Code generation for function 'all'
//

// Include files
#include "all.h"
#include "rt_nonfinite.h"
#include <cstring>

// Function Definitions
namespace coder {
void all(const bool x[21], bool y[3])
{
  int ix;
  bool exitg1;
  y[0] = true;
  y[1] = true;
  y[2] = true;
  ix = 1;
  exitg1 = false;
  while ((!exitg1) && (ix <= 7)) {
    if (!x[ix - 1]) {
      y[0] = false;
      exitg1 = true;
    } else {
      ix++;
    }
  }
  ix = 8;
  exitg1 = false;
  while ((!exitg1) && (ix <= 14)) {
    if (!x[ix - 1]) {
      y[1] = false;
      exitg1 = true;
    } else {
      ix++;
    }
  }
  ix = 15;
  exitg1 = false;
  while ((!exitg1) && (ix <= 21)) {
    if (!x[ix - 1]) {
      y[2] = false;
      exitg1 = true;
    } else {
      ix++;
    }
  }
}

} // namespace coder

// End of code generation (all.cpp)
