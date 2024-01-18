//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// znlmpc_getUBounds.cpp
//
// Code generation for function 'znlmpc_getUBounds'
//

// Include files
#include "znlmpc_getUBounds.h"
#include "NMPC_Node_data.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <cmath>
#include <cstring>

// Function Definitions
namespace coder {
void znlmpc_getUBounds(const double runtimedata_lastMV[2],
                       const double runtimedata_MVMin[14],
                       const double runtimedata_MVMax[14],
                       const double runtimedata_MVRateMin[14],
                       const double runtimedata_MVRateMax[14],
                       ::coder::array<double, 2U> &A,
                       ::coder::array<double, 1U> &Bu)
{
  array<double, 3U> Auf;
  array<double, 2U> y;
  array<signed char, 1U> ii;
  double b_Bu[56];
  double ic_idx_0;
  double ic_idx_1;
  double s;
  int boffset;
  int coffset;
  int idx;
  signed char Au[784];
  bool icf[56];
  bool exitg1;
  std::memset(&Au[0], 0, 784U * sizeof(signed char));
  std::memset(&b_Bu[0], 0, 56U * sizeof(double));
  for (int i{0}; i < 56; i++) {
    icf[i] = false;
  }
  ic_idx_0 = 1.0;
  ic_idx_1 = 2.0;
  for (int i{0}; i < 7; i++) {
    double d;
    double d1;
    double d2;
    double d3;
    double d4;
    double d5;
    double d6;
    s = runtimedata_MVRateMin[i];
    icf[static_cast<int>(ic_idx_0) - 1] =
        ((!std::isinf(s)) && (!std::isnan(s)));
    d = runtimedata_MVRateMin[i + 7];
    icf[static_cast<int>(ic_idx_1) - 1] =
        ((!std::isinf(d)) && (!std::isnan(d)));
    d1 = runtimedata_MVRateMax[i];
    icf[static_cast<int>(ic_idx_0 + 2.0) - 1] =
        ((!std::isinf(d1)) && (!std::isnan(d1)));
    d2 = runtimedata_MVRateMax[i + 7];
    icf[static_cast<int>(ic_idx_1 + 2.0) - 1] =
        ((!std::isinf(d2)) && (!std::isnan(d2)));
    d3 = runtimedata_MVMin[i];
    icf[static_cast<int>(ic_idx_0 + 4.0) - 1] =
        ((!std::isinf(d3)) && (!std::isnan(d3)));
    d4 = runtimedata_MVMin[i + 7];
    icf[static_cast<int>(ic_idx_1 + 4.0) - 1] =
        ((!std::isinf(d4)) && (!std::isnan(d4)));
    d5 = runtimedata_MVMax[i];
    icf[static_cast<int>(ic_idx_0 + 6.0) - 1] =
        ((!std::isinf(d5)) && (!std::isnan(d5)));
    idx = static_cast<int>(ic_idx_0) + 112 * i;
    Au[idx - 1] = -1;
    coffset = static_cast<int>(ic_idx_1) + 112 * i;
    Au[coffset - 1] = 0;
    d6 = runtimedata_MVMax[i + 7];
    icf[static_cast<int>(ic_idx_1 + 6.0) - 1] =
        ((!std::isinf(d6)) && (!std::isnan(d6)));
    Au[idx + 55] = 0;
    Au[coffset + 55] = -1;
    idx = static_cast<int>(ic_idx_0 + 2.0) + 112 * i;
    Au[idx - 1] = 1;
    coffset = static_cast<int>(ic_idx_1 + 2.0) + 112 * i;
    Au[coffset - 1] = 0;
    Au[idx + 55] = 0;
    Au[coffset + 55] = 1;
    idx = static_cast<int>(ic_idx_0 + 4.0) + 112 * i;
    Au[idx - 1] = -1;
    coffset = static_cast<int>(ic_idx_1 + 4.0) + 112 * i;
    Au[coffset - 1] = 0;
    Au[idx + 55] = 0;
    Au[coffset + 55] = -1;
    idx = static_cast<int>(ic_idx_0 + 6.0) + 112 * i;
    Au[idx - 1] = 1;
    coffset = static_cast<int>(ic_idx_1 + 6.0) + 112 * i;
    Au[coffset - 1] = 0;
    Au[idx + 55] = 0;
    Au[coffset + 55] = 1;
    b_Bu[static_cast<int>(ic_idx_0) - 1] = -s;
    b_Bu[static_cast<int>(ic_idx_1) - 1] = -d;
    b_Bu[static_cast<int>(ic_idx_0 + 2.0) - 1] = d1;
    b_Bu[static_cast<int>(ic_idx_1 + 2.0) - 1] = d2;
    b_Bu[static_cast<int>(ic_idx_0 + 4.0) - 1] = -d3;
    b_Bu[static_cast<int>(ic_idx_1 + 4.0) - 1] = -d4;
    b_Bu[static_cast<int>(ic_idx_0 + 6.0) - 1] = d5;
    b_Bu[static_cast<int>(ic_idx_1 + 6.0) - 1] = d6;
    if (i + 1 == 1) {
      s = b_Bu[static_cast<int>(ic_idx_1) - 1] - runtimedata_lastMV[1];
      b_Bu[static_cast<int>(ic_idx_0) - 1] -= runtimedata_lastMV[0];
      b_Bu[static_cast<int>(ic_idx_1) - 1] = s;
      s = b_Bu[static_cast<int>(ic_idx_1 + 2.0) - 1] + runtimedata_lastMV[1];
      b_Bu[static_cast<int>(ic_idx_0 + 2.0) - 1] += runtimedata_lastMV[0];
      b_Bu[static_cast<int>(ic_idx_1 + 2.0) - 1] = s;
    } else {
      idx = 112 * (i - 1);
      coffset = static_cast<int>(ic_idx_0) + idx;
      Au[coffset - 1] = 1;
      boffset = static_cast<int>(ic_idx_1) + idx;
      Au[boffset - 1] = 0;
      Au[coffset + 55] = 0;
      Au[boffset + 55] = 1;
      coffset = static_cast<int>(ic_idx_0 + 2.0) + idx;
      Au[coffset - 1] = -1;
      idx += static_cast<int>(ic_idx_1 + 2.0);
      Au[idx - 1] = 0;
      Au[coffset + 55] = 0;
      Au[idx + 55] = -1;
    }
    ic_idx_0 += 8.0;
    ic_idx_1 += 8.0;
  }
  idx = 0;
  ii.set_size(56);
  coffset = 0;
  exitg1 = false;
  while ((!exitg1) && (coffset < 56)) {
    if (icf[coffset]) {
      idx++;
      ii[idx - 1] = static_cast<signed char>(coffset + 1);
      if (idx >= 56) {
        exitg1 = true;
      } else {
        coffset++;
      }
    } else {
      coffset++;
    }
  }
  if (idx < 1) {
    idx = 0;
  }
  ii.set_size(idx);
  if (ii.size(0) > 0) {
    int b_i;
    Bu.set_size(ii.size(0));
    idx = ii.size(0);
    for (b_i = 0; b_i < idx; b_i++) {
      Bu[b_i] = b_Bu[ii[b_i] - 1];
    }
    Auf.set_size(ii.size(0), 2, 7);
    b_i = ii.size(0);
    for (int j{0}; j < 2; j++) {
      for (int k{0}; k < 7; k++) {
        for (int i{0}; i < b_i; i++) {
          Auf[(i + Auf.size(0) * j) + Auf.size(0) * 2 * k] =
              Au[((ii[i] + 56 * j) + 112 * k) - 1];
        }
      }
    }
    idx = ii.size(0);
    y.set_size(ii.size(0), 14);
    for (int j{0}; j < 14; j++) {
      coffset = j * idx;
      boffset = j * 14;
      for (int i{0}; i < idx; i++) {
        s = 0.0;
        for (int k{0}; k < 14; k++) {
          s += Auf[k * ii.size(0) + i] * static_cast<double>(iv[boffset + k]);
        }
        y[coffset + i] = s;
      }
    }
    coffset = ii.size(0);
    boffset = ii.size(0);
    A.set_size(ii.size(0), 113);
    idx = ii.size(0);
    for (b_i = 0; b_i < 98; b_i++) {
      for (int j{0}; j < idx; j++) {
        A[j + A.size(0) * b_i] = 0.0;
      }
    }
    for (b_i = 0; b_i < 14; b_i++) {
      for (int j{0}; j < coffset; j++) {
        A[j + A.size(0) * (b_i + 98)] = y[j + coffset * b_i];
      }
    }
    for (b_i = 0; b_i < boffset; b_i++) {
      A[b_i + A.size(0) * 112] = 0.0;
    }
  } else {
    Bu.set_size(0);
    A.set_size(0, 113);
  }
}

} // namespace coder

// End of code generation (znlmpc_getUBounds.cpp)
