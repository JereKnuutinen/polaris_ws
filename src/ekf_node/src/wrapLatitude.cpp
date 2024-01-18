//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// wrapLatitude.cpp
//
// Code generation for function 'wrapLatitude'
//

// Include files
#include "wrapLatitude.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions
namespace coder {
namespace matlabshared {
namespace internal {
namespace latlon {
void wrapLatitude(double &lat, double &lon)
{
  double flat;
  double x;
  flat = std::abs(lat);
  if (flat > 180.0) {
    x = lat + 180.0;
    if (std::isnan(x) || std::isinf(x)) {
      flat = rtNaN;
    } else if (x == 0.0) {
      flat = 0.0;
    } else {
      flat = std::fmod(x, 360.0);
      if (flat == 0.0) {
        flat = 0.0;
      } else if (x < 0.0) {
        flat += 360.0;
      }
    }
    lat = lat * 0.0 + (flat - 180.0);
    flat = std::abs(lat);
  }
  if (flat > 90.0) {
    bool latp2;
    flat = std::abs(lat);
    latp2 = (flat > 90.0);
    lon += 180.0;
    x = lat * static_cast<double>(latp2);
    if (!std::isnan(x)) {
      if (x < 0.0) {
        x = -1.0;
      } else {
        x = (x > 0.0);
      }
    }
    lat = lat * static_cast<double>(!latp2) +
          x * (90.0 - (flat * static_cast<double>(latp2) - 90.0)) *
              static_cast<double>(latp2);
  }
}

} // namespace latlon
} // namespace internal
} // namespace matlabshared
} // namespace coder

// End of code generation (wrapLatitude.cpp)
