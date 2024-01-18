//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// cosd.cpp
//
// Code generation for function 'cosd'
//

// Include files
#include "cosd.h"
#include "EKF_Node_rtwutil.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions
namespace coder {
void b_cosd(double &x)
{
  if (std::isinf(x) || std::isnan(x)) {
    x = rtNaN;
  } else {
    double absx;
    signed char n;
    x = rt_remd_snf(x, 360.0);
    absx = std::abs(x);
    if (absx > 180.0) {
      if (x > 0.0) {
        x -= 360.0;
      } else {
        x += 360.0;
      }
      absx = std::abs(x);
    }
    if (absx <= 45.0) {
      x *= 0.017453292519943295;
      n = 0;
    } else if (absx <= 135.0) {
      if (x > 0.0) {
        x = 0.017453292519943295 * (x - 90.0);
        n = 1;
      } else {
        x = 0.017453292519943295 * (x + 90.0);
        n = -1;
      }
    } else if (x > 0.0) {
      x = 0.017453292519943295 * (x - 180.0);
      n = 2;
    } else {
      x = 0.017453292519943295 * (x + 180.0);
      n = -2;
    }
    if (n == 0) {
      x = std::cos(x);
    } else if (n == 1) {
      x = -std::sin(x);
    } else if (n == -1) {
      x = std::sin(x);
    } else {
      x = -std::cos(x);
    }
  }
}

} // namespace coder

// End of code generation (cosd.cpp)
