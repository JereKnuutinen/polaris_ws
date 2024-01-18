//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// checkVectorNonFinite.cpp
//
// Code generation for function 'checkVectorNonFinite'
//

// Include files
#include "checkVectorNonFinite.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <cmath>
#include <cstring>

// Function Definitions
namespace coder {
namespace optim {
namespace coder {
namespace utils {
namespace ObjNonlinEvaluator {
namespace internal {
int checkVectorNonFinite(int N, const ::coder::array<double, 1U> &vec, int iv0)
{
  int idx_current;
  int idx_end;
  int status;
  bool allFinite;
  status = 1;
  allFinite = true;
  idx_current = iv0;
  idx_end = (iv0 + N) - 1;
  while (allFinite && (idx_current <= idx_end)) {
    double allFinite_tmp;
    allFinite_tmp = vec[idx_current - 1];
    allFinite = ((!std::isinf(allFinite_tmp)) && (!std::isnan(allFinite_tmp)));
    idx_current++;
  }
  if (!allFinite) {
    idx_current -= 2;
    if (std::isnan(vec[idx_current])) {
      status = -3;
    } else if (vec[idx_current] < 0.0) {
      status = -1;
    } else {
      status = -2;
    }
  }
  return status;
}

int checkVectorNonFinite(const double vec[98], int iv0)
{
  int idx_current;
  int status;
  bool allFinite;
  status = 1;
  allFinite = true;
  idx_current = iv0;
  while (allFinite && (idx_current <= iv0 + 97)) {
    double allFinite_tmp;
    allFinite_tmp = vec[idx_current - 1];
    allFinite = ((!std::isinf(allFinite_tmp)) && (!std::isnan(allFinite_tmp)));
    idx_current++;
  }
  if (!allFinite) {
    idx_current -= 2;
    if (std::isnan(vec[idx_current])) {
      status = -3;
    } else if (vec[idx_current] < 0.0) {
      status = -1;
    } else {
      status = -2;
    }
  }
  return status;
}

} // namespace internal
} // namespace ObjNonlinEvaluator
} // namespace utils
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (checkVectorNonFinite.cpp)
