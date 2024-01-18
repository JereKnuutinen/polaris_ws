//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// factorQR.cpp
//
// Code generation for function 'factorQR'
//

// Include files
#include "factorQR.h"
#include "NMPC_Node_internal_types.h"
#include "rt_nonfinite.h"
#include "xzgeqp3.h"
#include "coder_array.h"
#include <cstring>

// Function Definitions
namespace coder {
namespace optim {
namespace coder {
namespace QRManager {
void factorQR(e_struct_T &obj, const ::coder::array<double, 1U> &A, int mrows,
              int ncols, int ldA)
{
  int ix0;
  int iy0;
  int minmana;
  bool guard1{false};
  guard1 = false;
  if ((A.size(0) != 0) && (mrows * ncols > 0)) {
    for (minmana = 0; minmana < ncols; minmana++) {
      ix0 = ldA * minmana;
      iy0 = obj.ldq * minmana;
      for (int k{0}; k < mrows; k++) {
        obj.QR[iy0 + k] = A[ix0 + k];
      }
    }
    guard1 = true;
  } else if (mrows * ncols == 0) {
    obj.mrows = mrows;
    obj.ncols = ncols;
    obj.minRowCol = 0;
  } else {
    guard1 = true;
  }
  if (guard1) {
    obj.usedPivoting = false;
    obj.mrows = mrows;
    obj.ncols = ncols;
    for (minmana = 0; minmana < ncols; minmana++) {
      obj.jpvt[minmana] = minmana + 1;
    }
    if (mrows <= ncols) {
      iy0 = mrows;
    } else {
      iy0 = ncols;
    }
    obj.minRowCol = iy0;
    ix0 = obj.QR.size(0);
    minmana = obj.QR.size(1);
    if (ix0 <= minmana) {
      minmana = ix0;
    }
    obj.tau.set_size(minmana);
    for (ix0 = 0; ix0 < minmana; ix0++) {
      obj.tau[ix0] = 0.0;
    }
    if ((obj.QR.size(0) != 0) && (obj.QR.size(1) != 0) && (iy0 >= 1)) {
      internal::reflapack::qrf(obj.QR, mrows, ncols, iy0, obj.tau);
    }
  }
}

} // namespace QRManager
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (factorQR.cpp)
