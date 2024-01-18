//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// eul2quat.cpp
//
// Code generation for function 'eul2quat'
//

// Include files
#include "eul2quat.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions
namespace coder {
void eul2quat(const double eul[3], double q[4])
{
  double b_q_tmp;
  double c_idx_0;
  double c_idx_1;
  double c_idx_2;
  double d;
  double q_tmp;
  double s_idx_0;
  double s_idx_1;
  d = eul[0] / 2.0;
  c_idx_0 = std::cos(d);
  d = std::sin(d);
  s_idx_0 = d;
  d = eul[1] / 2.0;
  c_idx_1 = std::cos(d);
  d = std::sin(d);
  s_idx_1 = d;
  d = eul[2] / 2.0;
  c_idx_2 = std::cos(d);
  d = std::sin(d);
  q_tmp = c_idx_0 * c_idx_1;
  b_q_tmp = s_idx_0 * s_idx_1;
  q[0] = q_tmp * c_idx_2 + b_q_tmp * d;
  q[1] = q_tmp * d - b_q_tmp * c_idx_2;
  q_tmp = s_idx_0 * c_idx_1;
  b_q_tmp = c_idx_0 * s_idx_1;
  q[2] = b_q_tmp * c_idx_2 + q_tmp * d;
  q[3] = q_tmp * c_idx_2 - b_q_tmp * d;
}

} // namespace coder

// End of code generation (eul2quat.cpp)
