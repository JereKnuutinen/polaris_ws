//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// squareQ_appendCol.cpp
//
// Code generation for function 'squareQ_appendCol'
//

// Include files
#include "squareQ_appendCol.h"
#include "NMPC_Node_internal_types.h"
#include "rt_nonfinite.h"
#include "xrotg.h"
#include "coder_array.h"
#include <cstring>

// Function Definitions
namespace coder {
namespace optim {
namespace coder {
namespace QRManager {
void squareQ_appendCol(e_struct_T &obj, const ::coder::array<double, 1U> &vec,
                       int iv0)
{
  double c;
  double s;
  double temp;
  int Qk0;
  int b_iy;
  int i;
  int iy;
  int iyend;
  int lda;
  iyend = obj.mrows;
  Qk0 = obj.ncols + 1;
  if (iyend <= Qk0) {
    Qk0 = iyend;
  }
  obj.minRowCol = Qk0;
  iy = obj.ldq * obj.ncols;
  lda = obj.ldq;
  if (obj.mrows != 0) {
    iyend = iy + obj.mrows;
    for (b_iy = iy + 1; b_iy <= iyend; b_iy++) {
      obj.QR[b_iy - 1] = 0.0;
    }
    i = obj.ldq * (obj.mrows - 1) + 1;
    for (iyend = 1; lda < 0 ? iyend >= i : iyend <= i; iyend += lda) {
      c = 0.0;
      Qk0 = (iyend + obj.mrows) - 1;
      for (int idx{iyend}; idx <= Qk0; idx++) {
        c += obj.Q[idx - 1] * vec[((iv0 + idx) - iyend) - 1];
      }
      obj.QR[iy] = obj.QR[iy] + c;
      iy++;
    }
  }
  obj.ncols++;
  i = obj.ncols - 1;
  obj.jpvt[i] = obj.ncols;
  for (int idx{obj.mrows - 2}; idx + 2 > obj.ncols; idx--) {
    Qk0 = idx + obj.ldq * i;
    temp = obj.QR[Qk0 + 1];
    c = internal::blas::xrotg(&obj.QR[Qk0], temp, s);
    obj.QR[Qk0 + 1] = temp;
    Qk0 = obj.ldq * idx;
    iyend = obj.mrows;
    if (obj.mrows >= 1) {
      b_iy = obj.ldq + Qk0;
      for (int k{0}; k < iyend; k++) {
        lda = b_iy + k;
        iy = Qk0 + k;
        temp = c * obj.Q[iy] + s * obj.Q[lda];
        obj.Q[lda] = c * obj.Q[lda] - s * obj.Q[iy];
        obj.Q[iy] = temp;
      }
    }
  }
}

} // namespace QRManager
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (squareQ_appendCol.cpp)
