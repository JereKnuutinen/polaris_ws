//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// deleteColMoveEnd.cpp
//
// Code generation for function 'deleteColMoveEnd'
//

// Include files
#include "deleteColMoveEnd.h"
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
void deleteColMoveEnd(e_struct_T &obj, int idx)
{
  double s;
  int i;
  if (obj.usedPivoting) {
    i = 1;
    while ((i <= obj.ncols) && (obj.jpvt[i - 1] != idx)) {
      i++;
    }
    idx = i;
  }
  if (idx >= obj.ncols) {
    obj.ncols--;
  } else {
    int b_i;
    int ix;
    int k;
    b_i = obj.ncols - 1;
    obj.jpvt[idx - 1] = obj.jpvt[b_i];
    i = obj.minRowCol;
    for (k = 0; k < i; k++) {
      obj.QR[k + obj.ldq * (idx - 1)] = obj.QR[k + obj.ldq * b_i];
    }
    obj.ncols = b_i;
    ix = obj.mrows;
    i = obj.ncols;
    if (ix <= i) {
      i = ix;
    }
    obj.minRowCol = i;
    if (idx < obj.mrows) {
      double c;
      double temp;
      int endIdx;
      int idxRotGCol;
      int n;
      int temp_tmp;
      ix = obj.mrows - 1;
      endIdx = obj.ncols;
      if (ix <= endIdx) {
        endIdx = ix;
      }
      k = endIdx;
      idxRotGCol = obj.ldq * (idx - 1);
      while (k >= idx) {
        b_i = k + idxRotGCol;
        temp = obj.QR[b_i];
        c = internal::blas::xrotg(&obj.QR[b_i - 1], temp, s);
        obj.QR[b_i] = temp;
        b_i = obj.ldq * (k - 1);
        obj.QR[k + b_i] = 0.0;
        i = k + obj.ldq * idx;
        n = obj.ncols - idx;
        if (n >= 1) {
          ix = i - 1;
          for (int b_k{0}; b_k < n; b_k++) {
            temp = c * obj.QR[ix] + s * obj.QR[i];
            obj.QR[i] = c * obj.QR[i] - s * obj.QR[ix];
            obj.QR[ix] = temp;
            i += obj.ldq;
            ix += obj.ldq;
          }
        }
        i = obj.ldq + b_i;
        n = obj.mrows;
        for (int b_k{0}; b_k < n; b_k++) {
          ix = i + b_k;
          temp_tmp = b_i + b_k;
          temp = c * obj.Q[temp_tmp] + s * obj.Q[ix];
          obj.Q[ix] = c * obj.Q[ix] - s * obj.Q[temp_tmp];
          obj.Q[temp_tmp] = temp;
        }
        k--;
      }
      b_i = idx + 1;
      for (k = b_i; k <= endIdx; k++) {
        idxRotGCol = obj.ldq * (k - 1);
        i = k + idxRotGCol;
        temp = obj.QR[i];
        c = internal::blas::xrotg(&obj.QR[i - 1], temp, s);
        obj.QR[i] = temp;
        i = k * (obj.ldq + 1);
        n = obj.ncols - k;
        if (n >= 1) {
          ix = i - 1;
          for (int b_k{0}; b_k < n; b_k++) {
            temp = c * obj.QR[ix] + s * obj.QR[i];
            obj.QR[i] = c * obj.QR[i] - s * obj.QR[ix];
            obj.QR[ix] = temp;
            i += obj.ldq;
            ix += obj.ldq;
          }
        }
        i = obj.ldq + idxRotGCol;
        n = obj.mrows;
        for (int b_k{0}; b_k < n; b_k++) {
          ix = i + b_k;
          temp_tmp = idxRotGCol + b_k;
          temp = c * obj.Q[temp_tmp] + s * obj.Q[ix];
          obj.Q[ix] = c * obj.Q[ix] - s * obj.Q[temp_tmp];
          obj.Q[temp_tmp] = temp;
        }
      }
    }
  }
}

} // namespace QRManager
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (deleteColMoveEnd.cpp)
