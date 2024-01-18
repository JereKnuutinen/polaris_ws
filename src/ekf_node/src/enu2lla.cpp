//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// enu2lla.cpp
//
// Code generation for function 'enu2lla'
//

// Include files
#include "enu2lla.h"
#include "EKF_Node_data.h"
#include "EKF_Node_rtwutil.h"
#include "cosd.h"
#include "rt_nonfinite.h"
#include "sind.h"
#include "wrapLatitude.h"
#include <cmath>

// Function Definitions
namespace coder {
void b_enu2lla(const double xyzENU[6], const double b_lla0[3], double lla[6])
{
  double lla0tmp[6];
  double xyz[6];
  double Rm_idx_0;
  double Rm_idx_1;
  double Rn_idx_0;
  double Rn_idx_1;
  double absx;
  double d;
  double r_idx_0;
  double r_idx_1;
  int ibmat;
  int jcol;
  signed char n;
  bool b[6];
  bool nanIdx[2];
  bool b_b;
  bool exitg1;
  bool y;
  for (jcol = 0; jcol < 3; jcol++) {
    ibmat = jcol << 1;
    d = b_lla0[jcol];
    lla0tmp[ibmat] = d;
    lla0tmp[ibmat + 1] = d;
  }
  for (jcol = 0; jcol < 2; jcol++) {
    d = xyzENU[jcol];
    absx = xyzENU[jcol + 2];
    Rn_idx_1 = xyzENU[jcol + 4];
    for (ibmat = 0; ibmat < 3; ibmat++) {
      xyz[jcol + (ibmat << 1)] = (static_cast<double>(iv[ibmat]) * d +
                                  static_cast<double>(iv[ibmat + 3]) * absx) +
                                 static_cast<double>(iv[ibmat + 6]) * Rn_idx_1;
    }
  }
  lla[4] = -xyz[4] + lla0tmp[4];
  if (std::isinf(lla0tmp[0]) || std::isnan(lla0tmp[0])) {
    d = rtNaN;
    absx = rtNaN;
  } else {
    Rn_idx_0 = rt_remd_snf(lla0tmp[0], 360.0);
    Rn_idx_1 = Rn_idx_0;
    absx = std::abs(Rn_idx_0);
    if (absx > 180.0) {
      if (Rn_idx_0 > 0.0) {
        Rn_idx_1 = Rn_idx_0 - 360.0;
      } else {
        Rn_idx_1 = Rn_idx_0 + 360.0;
      }
      absx = std::abs(Rn_idx_1);
    }
    if (absx <= 45.0) {
      Rn_idx_1 *= 0.017453292519943295;
      n = 0;
    } else if (absx <= 135.0) {
      if (Rn_idx_1 > 0.0) {
        Rn_idx_1 = 0.017453292519943295 * (Rn_idx_1 - 90.0);
        n = 1;
      } else {
        Rn_idx_1 = 0.017453292519943295 * (Rn_idx_1 + 90.0);
        n = -1;
      }
    } else if (Rn_idx_1 > 0.0) {
      Rn_idx_1 = 0.017453292519943295 * (Rn_idx_1 - 180.0);
      n = 2;
    } else {
      Rn_idx_1 = 0.017453292519943295 * (Rn_idx_1 + 180.0);
      n = -2;
    }
    if (n == 0) {
      d = std::sin(Rn_idx_1);
    } else if (n == 1) {
      d = std::cos(Rn_idx_1);
    } else if (n == -1) {
      d = -std::cos(Rn_idx_1);
    } else {
      d = -std::sin(Rn_idx_1);
    }
    absx = std::abs(Rn_idx_0);
    if (absx > 180.0) {
      if (Rn_idx_0 > 0.0) {
        Rn_idx_0 -= 360.0;
      } else {
        Rn_idx_0 += 360.0;
      }
      absx = std::abs(Rn_idx_0);
    }
    if (absx <= 45.0) {
      Rn_idx_0 *= 0.017453292519943295;
      n = 0;
    } else if (absx <= 135.0) {
      if (Rn_idx_0 > 0.0) {
        Rn_idx_0 = 0.017453292519943295 * (Rn_idx_0 - 90.0);
        n = 1;
      } else {
        Rn_idx_0 = 0.017453292519943295 * (Rn_idx_0 + 90.0);
        n = -1;
      }
    } else if (Rn_idx_0 > 0.0) {
      Rn_idx_0 = 0.017453292519943295 * (Rn_idx_0 - 180.0);
      n = 2;
    } else {
      Rn_idx_0 = 0.017453292519943295 * (Rn_idx_0 + 180.0);
      n = -2;
    }
    if (n == 0) {
      absx = std::cos(Rn_idx_0);
    } else if (n == 1) {
      absx = -std::sin(Rn_idx_0);
    } else if (n == -1) {
      absx = std::sin(Rn_idx_0);
    } else {
      absx = -std::cos(Rn_idx_0);
    }
  }
  d = 1.0 - 0.0066943799901413165 * d * d;
  Rn_idx_1 = 6.378137E+6 / std::sqrt(d);
  d = Rn_idx_1 * (0.99330562000985867 / d);
  Rn_idx_1 *= absx;
  lla[0] = 57.295779513082323 * (xyz[0] * rt_atan2d_snf(1.0, d)) + lla0tmp[0];
  lla[2] =
      57.295779513082323 * (xyz[2] * rt_atan2d_snf(1.0, Rn_idx_1)) + lla0tmp[2];
  lla[5] = -xyz[5] + lla0tmp[5];
  if (std::isinf(lla0tmp[1]) || std::isnan(lla0tmp[1])) {
    d = rtNaN;
    absx = rtNaN;
  } else {
    Rn_idx_0 = rt_remd_snf(lla0tmp[1], 360.0);
    Rn_idx_1 = Rn_idx_0;
    absx = std::abs(Rn_idx_0);
    if (absx > 180.0) {
      if (Rn_idx_0 > 0.0) {
        Rn_idx_1 = Rn_idx_0 - 360.0;
      } else {
        Rn_idx_1 = Rn_idx_0 + 360.0;
      }
      absx = std::abs(Rn_idx_1);
    }
    if (absx <= 45.0) {
      Rn_idx_1 *= 0.017453292519943295;
      n = 0;
    } else if (absx <= 135.0) {
      if (Rn_idx_1 > 0.0) {
        Rn_idx_1 = 0.017453292519943295 * (Rn_idx_1 - 90.0);
        n = 1;
      } else {
        Rn_idx_1 = 0.017453292519943295 * (Rn_idx_1 + 90.0);
        n = -1;
      }
    } else if (Rn_idx_1 > 0.0) {
      Rn_idx_1 = 0.017453292519943295 * (Rn_idx_1 - 180.0);
      n = 2;
    } else {
      Rn_idx_1 = 0.017453292519943295 * (Rn_idx_1 + 180.0);
      n = -2;
    }
    if (n == 0) {
      d = std::sin(Rn_idx_1);
    } else if (n == 1) {
      d = std::cos(Rn_idx_1);
    } else if (n == -1) {
      d = -std::cos(Rn_idx_1);
    } else {
      d = -std::sin(Rn_idx_1);
    }
    absx = std::abs(Rn_idx_0);
    if (absx > 180.0) {
      if (Rn_idx_0 > 0.0) {
        Rn_idx_0 -= 360.0;
      } else {
        Rn_idx_0 += 360.0;
      }
      absx = std::abs(Rn_idx_0);
    }
    if (absx <= 45.0) {
      Rn_idx_0 *= 0.017453292519943295;
      n = 0;
    } else if (absx <= 135.0) {
      if (Rn_idx_0 > 0.0) {
        Rn_idx_0 = 0.017453292519943295 * (Rn_idx_0 - 90.0);
        n = 1;
      } else {
        Rn_idx_0 = 0.017453292519943295 * (Rn_idx_0 + 90.0);
        n = -1;
      }
    } else if (Rn_idx_0 > 0.0) {
      Rn_idx_0 = 0.017453292519943295 * (Rn_idx_0 - 180.0);
      n = 2;
    } else {
      Rn_idx_0 = 0.017453292519943295 * (Rn_idx_0 + 180.0);
      n = -2;
    }
    if (n == 0) {
      absx = std::cos(Rn_idx_0);
    } else if (n == 1) {
      absx = -std::sin(Rn_idx_0);
    } else if (n == -1) {
      absx = std::sin(Rn_idx_0);
    } else {
      absx = -std::cos(Rn_idx_0);
    }
  }
  d = 1.0 - 0.0066943799901413165 * d * d;
  Rn_idx_1 = 6.378137E+6 / std::sqrt(d);
  d = Rn_idx_1 * (0.99330562000985867 / d);
  Rn_idx_1 *= absx;
  lla[1] = 57.295779513082323 * (xyz[1] * rt_atan2d_snf(1.0, d)) + lla0tmp[1];
  lla[3] =
      57.295779513082323 * (xyz[3] * rt_atan2d_snf(1.0, Rn_idx_1)) + lla0tmp[3];
  for (jcol = 0; jcol < 6; jcol++) {
    b[jcol] = std::isnan(lla[jcol]);
  }
  nanIdx[0] = false;
  nanIdx[1] = false;
  jcol = 1;
  exitg1 = false;
  while ((!exitg1) && (jcol <= 5)) {
    if (b[jcol - 1]) {
      nanIdx[0] = true;
      exitg1 = true;
    } else {
      jcol += 2;
    }
  }
  jcol = 2;
  exitg1 = false;
  while ((!exitg1) && (jcol <= 6)) {
    if (b[jcol - 1]) {
      nanIdx[1] = true;
      exitg1 = true;
    } else {
      jcol += 2;
    }
  }
  nanIdx[0] = !nanIdx[0];
  nanIdx[1] = !nanIdx[1];
  y = nanIdx[0];
  b_b = nanIdx[1];
  for (jcol = 0; jcol < 3; jcol++) {
    ibmat = jcol << 1;
    b[ibmat] = y;
    b[ibmat + 1] = b_b;
  }
  for (jcol = 0; jcol < 6; jcol++) {
    lla[jcol] += 0.0 / static_cast<double>(b[jcol]);
  }
  r_idx_0 = lla[0];
  Rm_idx_0 = lla[2];
  Rn_idx_1 = std::abs(lla[0]);
  Rn_idx_0 = Rn_idx_1;
  nanIdx[0] = (Rn_idx_1 > 180.0);
  r_idx_1 = lla[1];
  Rm_idx_1 = lla[3];
  Rn_idx_1 = std::abs(lla[1]);
  nanIdx[1] = (Rn_idx_1 > 180.0);
  y = false;
  jcol = 0;
  exitg1 = false;
  while ((!exitg1) && (jcol <= 1)) {
    if (nanIdx[jcol]) {
      y = true;
      exitg1 = true;
    } else {
      jcol++;
    }
  }
  if (y) {
    d = lla[0] * static_cast<double>(nanIdx[0]) + 180.0;
    if (std::isnan(d) || std::isinf(d)) {
      absx = rtNaN;
    } else if (d == 0.0) {
      absx = 0.0;
    } else {
      absx = std::fmod(d, 360.0);
      if (absx == 0.0) {
        absx = 0.0;
      } else if (d < 0.0) {
        absx += 360.0;
      }
    }
    r_idx_1 = lla[0] * static_cast<double>(!nanIdx[0]) +
              (absx - 180.0) * static_cast<double>(nanIdx[0]);
    r_idx_0 = r_idx_1;
    Rn_idx_0 = std::abs(r_idx_1);
    d = lla[1] * static_cast<double>(nanIdx[1]) + 180.0;
    if (std::isnan(d) || std::isinf(d)) {
      absx = rtNaN;
    } else if (d == 0.0) {
      absx = 0.0;
    } else {
      absx = std::fmod(d, 360.0);
      if (absx == 0.0) {
        absx = 0.0;
      } else if (d < 0.0) {
        absx += 360.0;
      }
    }
    r_idx_1 = lla[1] * static_cast<double>(!nanIdx[1]) +
              (absx - 180.0) * static_cast<double>(nanIdx[1]);
    Rn_idx_1 = std::abs(r_idx_1);
  }
  nanIdx[0] = (Rn_idx_0 > 90.0);
  nanIdx[1] = (Rn_idx_1 > 90.0);
  y = false;
  jcol = 0;
  exitg1 = false;
  while ((!exitg1) && (jcol <= 1)) {
    if (nanIdx[jcol]) {
      y = true;
      exitg1 = true;
    } else {
      jcol++;
    }
  }
  if (y) {
    d = std::abs(r_idx_0);
    y = (d > 90.0);
    Rm_idx_0 = lla[2] + 180.0 * static_cast<double>(nanIdx[0]);
    absx = r_idx_0 * static_cast<double>(y);
    if (std::isnan(absx)) {
      absx = rtNaN;
    } else if (absx < 0.0) {
      absx = -1.0;
    } else {
      absx = (absx > 0.0);
    }
    r_idx_0 = r_idx_0 * static_cast<double>(!y) +
              absx * (90.0 - (d * static_cast<double>(y) - 90.0)) *
                  static_cast<double>(y);
    d = std::abs(r_idx_1);
    y = (d > 90.0);
    Rm_idx_1 = lla[3] + 180.0 * static_cast<double>(nanIdx[1]);
    absx = r_idx_1 * static_cast<double>(y);
    if (std::isnan(absx)) {
      absx = rtNaN;
    } else if (absx < 0.0) {
      absx = -1.0;
    } else {
      absx = (absx > 0.0);
    }
    r_idx_1 = r_idx_1 * static_cast<double>(!y) +
              absx * (90.0 - (d * static_cast<double>(y) - 90.0)) *
                  static_cast<double>(y);
  }
  lla[0] = r_idx_0;
  lla[2] = Rm_idx_0;
  nanIdx[0] = ((Rm_idx_0 > 180.0) || (Rm_idx_0 < -180.0));
  lla[1] = r_idx_1;
  lla[3] = Rm_idx_1;
  nanIdx[1] = ((Rm_idx_1 > 180.0) || (Rm_idx_1 < -180.0));
  y = false;
  jcol = 0;
  exitg1 = false;
  while ((!exitg1) && (jcol <= 1)) {
    if (nanIdx[jcol]) {
      y = true;
      exitg1 = true;
    } else {
      jcol++;
    }
  }
  if (y) {
    d = rt_remd_snf(Rm_idx_0 * static_cast<double>(nanIdx[0]), 360.0);
    d = Rm_idx_0 * static_cast<double>(!nanIdx[0]) +
        (d - 360.0 * std::trunc(d / 180.0)) * static_cast<double>(nanIdx[0]);
    lla[2] = d;
    d = rt_remd_snf(Rm_idx_1 * static_cast<double>(nanIdx[1]), 360.0);
    d = Rm_idx_1 * static_cast<double>(!nanIdx[1]) +
        (d - 360.0 * std::trunc(d / 180.0)) * static_cast<double>(nanIdx[1]);
    lla[3] = d;
  }
}

void enu2lla(const double xyzENU[3], const double b_lla0[3], double lla[3])
{
  double a[3];
  double xyz[3];
  double Rn;
  double d;
  double x_tmp;
  int k;
  bool x[3];
  bool exitg1;
  bool y;
  d = xyzENU[0];
  x_tmp = xyzENU[1];
  Rn = xyzENU[2];
  for (k = 0; k < 3; k++) {
    double d1;
    d1 = (static_cast<double>(iv[k]) * d +
          static_cast<double>(iv[k + 3]) * x_tmp) +
         static_cast<double>(iv[k + 6]) * Rn;
    xyz[k] = d1;
    a[k] = d1;
  }
  lla[2] = -a[2] + b_lla0[2];
  x_tmp = b_lla0[0];
  b_sind(x_tmp);
  x_tmp = 1.0 - 0.0066943799901413165 * x_tmp * x_tmp;
  Rn = 6.378137E+6 / std::sqrt(x_tmp);
  lla[0] =
      57.295779513082323 *
          (xyz[0] * rt_atan2d_snf(1.0, Rn * (0.99330562000985867 / x_tmp))) +
      b_lla0[0];
  d = b_lla0[0];
  b_cosd(d);
  lla[1] =
      57.295779513082323 * (xyz[1] * rt_atan2d_snf(1.0, Rn * d)) + b_lla0[1];
  x[0] = std::isnan(lla[0]);
  x[1] = std::isnan(lla[1]);
  x[2] = std::isnan(lla[2]);
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
  x_tmp = 0.0 / static_cast<double>(!y);
  lla[0] += x_tmp;
  lla[1] += x_tmp;
  lla[2] += x_tmp;
  matlabshared::internal::latlon::wrapLatitude(lla[0], lla[1]);
  x_tmp = lla[1];
  if ((lla[1] > 180.0) || (lla[1] < -180.0)) {
    x_tmp = rt_remd_snf(lla[1], 360.0);
    x_tmp = lla[1] * 0.0 + (x_tmp - 360.0 * std::trunc(x_tmp / 180.0));
  }
  lla[1] = x_tmp;
}

} // namespace coder

// End of code generation (enu2lla.cpp)
