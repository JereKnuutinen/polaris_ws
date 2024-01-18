//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// cholPSD.cpp
//
// Code generation for function 'cholPSD'
//

// Include files
#include "cholPSD.h"
#include "rt_nonfinite.h"
#include "svd.h"
#include "xpotrf.h"
#include <algorithm>
#include <cmath>
#include <cstring>

// Function Definitions
namespace coder {
namespace matlabshared {
namespace tracking {
namespace internal {
void cholPSD(const double A[4900], double b_value[4900])
{
  double Ss[4900];
  int jmax;
  std::copy(&A[0], &A[4900], &Ss[0]);
  jmax = ::coder::internal::lapack::xpotrf(Ss);
  if (jmax == 0) {
    int i;
    std::copy(&A[0], &A[4900], &Ss[0]);
    jmax = ::coder::internal::lapack::xpotrf(Ss);
    if (jmax == 0) {
      jmax = 68;
    } else {
      jmax -= 3;
    }
    for (int j{0}; j <= jmax; j++) {
      i = j + 2;
      if (i <= jmax + 2) {
        std::memset(&Ss[(j * 70 + i) + -1], 0,
                    static_cast<unsigned int>((jmax - i) + 3) * sizeof(double));
      }
    }
    for (i = 0; i < 70; i++) {
      for (jmax = 0; jmax < 70; jmax++) {
        b_value[jmax + 70 * i] = Ss[i + 70 * jmax];
      }
    }
  } else {
    double V[4900];
    double s[70];
    double d;
    bool p;
    p = true;
    for (jmax = 0; jmax < 4900; jmax++) {
      if (p) {
        d = A[jmax];
        if (std::isinf(d) || std::isnan(d)) {
          p = false;
        }
      } else {
        p = false;
      }
    }
    if (p) {
      ::coder::internal::svd(A, Ss, s, V);
    } else {
      for (jmax = 0; jmax < 70; jmax++) {
        s[jmax] = rtNaN;
      }
      for (int i{0}; i < 4900; i++) {
        V[i] = rtNaN;
      }
    }
    std::memset(&Ss[0], 0, 4900U * sizeof(double));
    for (jmax = 0; jmax < 70; jmax++) {
      Ss[jmax + 70 * jmax] = s[jmax];
    }
    for (jmax = 0; jmax < 4900; jmax++) {
      Ss[jmax] = std::sqrt(Ss[jmax]);
    }
    for (int i{0}; i < 70; i++) {
      for (jmax = 0; jmax < 70; jmax++) {
        d = 0.0;
        for (int j{0}; j < 70; j++) {
          d += V[i + 70 * j] * Ss[j + 70 * jmax];
        }
        b_value[i + 70 * jmax] = d;
      }
    }
  }
}

} // namespace internal
} // namespace tracking
} // namespace matlabshared
} // namespace coder

// End of code generation (cholPSD.cpp)
