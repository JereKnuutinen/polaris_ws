//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// xswap.cpp
//
// Code generation for function 'xswap'
//

// Include files
#include "xswap.h"
#include "rt_nonfinite.h"

// Function Definitions
namespace coder {
namespace internal {
namespace blas {
void b_xswap(double x[196], int ix0, int iy0)
{
  for (int k{0}; k < 14; k++) {
    double temp;
    int i;
    int temp_tmp;
    temp_tmp = (ix0 + k) - 1;
    temp = x[temp_tmp];
    i = (iy0 + k) - 1;
    x[temp_tmp] = x[i];
    x[i] = temp;
  }
}

void xswap(double x[4900], int ix0, int iy0)
{
  for (int k{0}; k < 70; k++) {
    double temp;
    int i;
    int temp_tmp;
    temp_tmp = (ix0 + k) - 1;
    temp = x[temp_tmp];
    i = (iy0 + k) - 1;
    x[temp_tmp] = x[i];
    x[i] = temp;
  }
}

} // namespace blas
} // namespace internal
} // namespace coder

// End of code generation (xswap.cpp)
