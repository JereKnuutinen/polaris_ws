//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// quat2eul.cpp
//
// Code generation for function 'quat2eul'
//

// Include files
#include "quat2eul.h"
#include "rt_nonfinite.h"
#include "coder_array.h"

// Function Definitions
void binary_expand_op(double in1[3], const coder::array<int, 2U> &in2,
                      const coder::array<double, 1U> &in3,
                      const coder::array<double, 1U> &in4)
{
  int loop_ub;
  int stride_0_0;
  int stride_1_0;
  stride_0_0 = (in3.size(0) != 1);
  stride_1_0 = (in4.size(0) != 1);
  if (in4.size(0) == 1) {
    loop_ub = in3.size(0);
  } else {
    loop_ub = in4.size(0);
  }
  for (int i{0}; i < loop_ub; i++) {
    in1[in2[i]] = -in3[i * stride_0_0] * 2.0 * in4[i * stride_1_0];
  }
}

// End of code generation (quat2eul.cpp)
