//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// saveJacobian.cpp
//
// Code generation for function 'saveJacobian'
//

// Include files
#include "saveJacobian.h"
#include "NMPC_Node_internal_types.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <cstring>

// Function Definitions
namespace coder {
namespace optim {
namespace coder {
namespace fminconsqp {
namespace TrialState {
void saveJacobian(d_struct_T &obj, int nVar, int mIneq,
                  const ::coder::array<double, 1U> &JacCineqTrans, int ineqCol0,
                  int mEq, const ::coder::array<double, 1U> &JacCeqTrans,
                  int eqCol0, int ldJ)
{
  int i;
  int iCol;
  int iCol_old;
  iCol = ldJ * (ineqCol0 - 1);
  iCol_old = -1;
  i = mIneq - ineqCol0;
  for (int idx_col{0}; idx_col <= i; idx_col++) {
    for (int k{0}; k < nVar; k++) {
      obj.JacCineqTrans_old[(iCol_old + k) + 1] = JacCineqTrans[iCol + k];
    }
    iCol += ldJ;
    iCol_old += ldJ;
  }
  iCol = ldJ * (eqCol0 - 1);
  iCol_old = -1;
  i = mEq - eqCol0;
  for (int idx_col{0}; idx_col <= i; idx_col++) {
    for (int k{0}; k < nVar; k++) {
      obj.JacCeqTrans_old[(iCol_old + k) + 1] = JacCeqTrans[iCol + k];
    }
    iCol += ldJ;
    iCol_old += ldJ;
  }
}

} // namespace TrialState
} // namespace fminconsqp
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (saveJacobian.cpp)
