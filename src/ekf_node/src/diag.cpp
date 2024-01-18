//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// diag.cpp
//
// Code generation for function 'diag'
//

// Include files
#include "diag.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <cstring>

// Function Definitions
namespace coder {
void diag(const ::coder::array<double, 2U> &v, ::coder::array<double, 1U> &d)
{
  if ((v.size(0) == 1) && (v.size(1) == 1)) {
    d.set_size(1);
    d[0] = v[0];
  } else {
    int dlen;
    int k;
    dlen = v.size(0);
    k = v.size(1);
    if (dlen <= k) {
      k = dlen;
    }
    if (v.size(1) > 0) {
      dlen = k;
    } else {
      dlen = 0;
    }
    d.set_size(dlen);
    dlen--;
    for (k = 0; k <= dlen; k++) {
      d[k] = v[k + v.size(0) * k];
    }
  }
}

void diag(double d[4900])
{
  std::memset(&d[0], 0, 4900U * sizeof(double));
  for (int j{0}; j < 70; j++) {
    d[j + 70 * j] = 1.0;
  }
}

void diag(const double v[13], double d[169])
{
  std::memset(&d[0], 0, 169U * sizeof(double));
  for (int j{0}; j < 13; j++) {
    d[j + 13 * j] = v[j];
  }
}

} // namespace coder

// End of code generation (diag.cpp)
