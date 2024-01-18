//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// computeLambdaLSQ.cpp
//
// Code generation for function 'computeLambdaLSQ'
//

// Include files
#include "computeLambdaLSQ.h"
#include "NMPC_Node_internal_types.h"
#include "computeQ_.h"
#include "rt_nonfinite.h"
#include "xgemv.h"
#include "xgeqp3.h"
#include "coder_array.h"
#include <cmath>
#include <cstring>

// Function Definitions
namespace coder {
namespace optim {
namespace coder {
namespace fminconsqp {
namespace stopping {
void computeLambdaLSQ(int nVar, int mConstr, e_struct_T &b_QRManager,
                      const ::coder::array<double, 1U> &ATwset, int ldA,
                      const ::coder::array<double, 1U> &grad,
                      ::coder::array<double, 1U> &lambdaLSQ,
                      ::coder::array<double, 2U> &workspace)
{
  double tol;
  int fullRank_R;
  int iQR_diag;
  int ix;
  int rankR;
  bool guard1{false};
  for (rankR = 0; rankR < mConstr; rankR++) {
    lambdaLSQ[rankR] = 0.0;
  }
  guard1 = false;
  if ((ATwset.size(0) != 0) && (nVar * mConstr > 0)) {
    for (int idx{0}; idx < mConstr; idx++) {
      iQR_diag = ldA * idx;
      ix = b_QRManager.ldq * idx;
      for (rankR = 0; rankR < nVar; rankR++) {
        b_QRManager.QR[ix + rankR] = ATwset[iQR_diag + rankR];
      }
    }
    guard1 = true;
  } else if (nVar * mConstr == 0) {
    b_QRManager.mrows = nVar;
    b_QRManager.ncols = mConstr;
    b_QRManager.minRowCol = 0;
  } else {
    guard1 = true;
  }
  if (guard1) {
    b_QRManager.usedPivoting = true;
    b_QRManager.mrows = nVar;
    b_QRManager.ncols = mConstr;
    if (nVar <= mConstr) {
      iQR_diag = nVar;
    } else {
      iQR_diag = mConstr;
    }
    b_QRManager.minRowCol = iQR_diag;
    ::coder::internal::lapack::xgeqp3(b_QRManager.QR, nVar, mConstr,
                                      b_QRManager.jpvt, b_QRManager.tau);
  }
  QRManager::computeQ_(b_QRManager, b_QRManager.mrows);
  ::coder::internal::blas::xgemv(nVar, nVar, b_QRManager.Q, b_QRManager.ldq,
                                 grad, workspace);
  if (nVar >= mConstr) {
    iQR_diag = nVar;
  } else {
    iQR_diag = mConstr;
  }
  tol = std::abs(b_QRManager.QR[0]) *
        std::fmin(1.4901161193847656E-8,
                  static_cast<double>(iQR_diag) * 2.2204460492503131E-16);
  if (nVar <= mConstr) {
    fullRank_R = nVar;
  } else {
    fullRank_R = mConstr;
  }
  rankR = 0;
  iQR_diag = 0;
  while ((rankR < fullRank_R) && (std::abs(b_QRManager.QR[iQR_diag]) > tol)) {
    rankR++;
    iQR_diag = (iQR_diag + b_QRManager.ldq) + 1;
  }
  if ((b_QRManager.QR.size(0) != 0) && (b_QRManager.QR.size(1) != 0) &&
      ((workspace.size(0) != 0) && (workspace.size(1) != 0)) && (rankR != 0)) {
    for (int idx{rankR}; idx >= 1; idx--) {
      iQR_diag = (idx + (idx - 1) * b_QRManager.ldq) - 1;
      workspace[idx - 1] = workspace[idx - 1] / b_QRManager.QR[iQR_diag];
      for (int i{0}; i <= idx - 2; i++) {
        ix = (idx - i) - 2;
        workspace[ix] = workspace[ix] -
                        workspace[idx - 1] * b_QRManager.QR[(iQR_diag - i) - 1];
      }
    }
  }
  if (mConstr <= fullRank_R) {
    fullRank_R = mConstr;
  }
  for (int idx{0}; idx < fullRank_R; idx++) {
    lambdaLSQ[b_QRManager.jpvt[idx] - 1] = workspace[idx];
  }
}

} // namespace stopping
} // namespace fminconsqp
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (computeLambdaLSQ.cpp)
