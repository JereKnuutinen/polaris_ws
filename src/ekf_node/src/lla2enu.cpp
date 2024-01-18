//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// lla2enu.cpp
//
// Code generation for function 'lla2enu'
//

// Include files
#include "lla2enu.h"
#include "EKF_Node_data.h"
#include "EKF_Node_rtwutil.h"
#include "cosd.h"
#include "rt_nonfinite.h"
#include "sind.h"
#include "wrapLatitude.h"
#include <cmath>

// Function Definitions
namespace coder {
void lla2enu(const double lla[3], const double b_lla0[3], double xyzENU[3])
{
  double Rn;
  double dLon;
  double xyzNED_idx_0;
  double xyzNED_idx_1;
  double xyzNED_idx_2;
  int k;
  bool x[3];
  bool exitg1;
  bool y;
  xyzNED_idx_2 = lla[0] - b_lla0[0];
  dLon = lla[1] - b_lla0[1];
  matlabshared::internal::latlon::wrapLatitude(xyzNED_idx_2, dLon);
  xyzNED_idx_1 = b_lla0[0];
  b_sind(xyzNED_idx_1);
  xyzNED_idx_1 = 1.0 - 0.0066943799901413165 * xyzNED_idx_1 * xyzNED_idx_1;
  Rn = 6.378137E+6 / std::sqrt(xyzNED_idx_1);
  xyzNED_idx_0 =
      xyzNED_idx_2 /
      (57.295779513082323 *
       rt_atan2d_snf(1.0, Rn * (0.99330562000985867 / xyzNED_idx_1)));
  if ((dLon > 180.0) || (dLon < -180.0)) {
    xyzNED_idx_1 = rt_remd_snf(dLon, 360.0);
    dLon =
        dLon * 0.0 + (xyzNED_idx_1 - 360.0 * std::trunc(xyzNED_idx_1 / 180.0));
  }
  xyzNED_idx_1 = b_lla0[0];
  b_cosd(xyzNED_idx_1);
  xyzNED_idx_1 =
      dLon / (57.295779513082323 * rt_atan2d_snf(1.0, Rn * xyzNED_idx_1));
  xyzNED_idx_2 = -lla[2] + b_lla0[2];
  x[0] = std::isnan(xyzNED_idx_0);
  x[1] = std::isnan(xyzNED_idx_1);
  x[2] = std::isnan(xyzNED_idx_2);
  y = false;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k <= 2)) {
    if (x[k]) {
      y = true;
      exitg1 = true;
    } else {
      k++;
    }
  }
  Rn = 0.0 / static_cast<double>(!y);
  xyzNED_idx_0 += Rn;
  xyzNED_idx_1 += Rn;
  xyzNED_idx_2 += Rn;
  for (k = 0; k < 3; k++) {
    xyzENU[k] = (static_cast<double>(iv[k]) * xyzNED_idx_0 +
                 static_cast<double>(iv[k + 3]) * xyzNED_idx_1) +
                static_cast<double>(iv[k + 6]) * xyzNED_idx_2;
  }
}

} // namespace coder

// End of code generation (lla2enu.cpp)
