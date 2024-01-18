//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// feasibleX0ForWorkingSet.cpp
//
// Code generation for function 'feasibleX0ForWorkingSet'
//

// Include files
#include "feasibleX0ForWorkingSet.h"
#include "NMPC_Node_internal_types.h"
#include "computeQ_.h"
#include "factorQR.h"
#include "maxConstraintViolation.h"
#include "rt_nonfinite.h"
#include "xgemv.h"
#include "xzgeqp3.h"
#include "coder_array.h"
#include <cmath>
#include <cstring>

// Function Definitions
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace initialize {
bool feasibleX0ForWorkingSet(::coder::array<double, 2U> &workspace,
                             ::coder::array<double, 1U> &xCurrent,
                             i_struct_T &workingset, e_struct_T &qrmanager)
{
  array<double, 2U> B;
  int mWConstr;
  int nVar;
  bool nonDegenerateWset;
  mWConstr = workingset.nActiveConstr;
  nVar = workingset.nVar;
  nonDegenerateWset = true;
  if (mWConstr != 0) {
    double c;
    int ar;
    int br;
    int i;
    int i1;
    int iAcol;
    int iac;
    int iy;
    int jBcol;
    int ldq;
    int mEq;
    for (ldq = 0; ldq < mWConstr; ldq++) {
      c = workingset.bwset[ldq];
      workspace[ldq] = c;
      workspace[ldq + workspace.size(0)] = c;
    }
    iAcol = workingset.ldA;
    if ((nVar != 0) && (mWConstr != 0)) {
      iy = 0;
      i = workingset.ldA * (mWConstr - 1) + 1;
      for (iac = 1; iAcol < 0 ? iac >= i : iac <= i; iac += iAcol) {
        c = 0.0;
        i1 = (iac + nVar) - 1;
        for (jBcol = iac; jBcol <= i1; jBcol++) {
          c += workingset.ATwset[jBcol - 1] * xCurrent[jBcol - iac];
        }
        workspace[iy] = workspace[iy] - c;
        iy++;
      }
    }
    if (mWConstr >= nVar) {
      int ldw;
      for (iy = 0; iy < nVar; iy++) {
        iAcol = qrmanager.ldq * iy;
        for (iac = 0; iac < mWConstr; iac++) {
          qrmanager.QR[iac + iAcol] =
              workingset.ATwset[iy + workingset.ldA * iac];
        }
      }
      if (mWConstr * nVar == 0) {
        qrmanager.mrows = mWConstr;
        qrmanager.ncols = nVar;
        qrmanager.minRowCol = 0;
      } else {
        qrmanager.usedPivoting = false;
        qrmanager.mrows = mWConstr;
        qrmanager.ncols = nVar;
        for (ldq = 0; ldq < nVar; ldq++) {
          qrmanager.jpvt[ldq] = ldq + 1;
        }
        if (mWConstr <= nVar) {
          i = mWConstr;
        } else {
          i = nVar;
        }
        qrmanager.minRowCol = i;
        B.set_size(qrmanager.QR.size(0), qrmanager.QR.size(1));
        iAcol = qrmanager.QR.size(0) * qrmanager.QR.size(1);
        for (i1 = 0; i1 < iAcol; i1++) {
          B[i1] = qrmanager.QR[i1];
        }
        iAcol = qrmanager.QR.size(0);
        iy = qrmanager.QR.size(1);
        if (iAcol <= iy) {
          iy = iAcol;
        }
        qrmanager.tau.set_size(iy);
        for (i1 = 0; i1 < iy; i1++) {
          qrmanager.tau[i1] = 0.0;
        }
        if ((qrmanager.QR.size(0) != 0) && (qrmanager.QR.size(1) != 0) &&
            (i >= 1)) {
          internal::reflapack::qrf(B, mWConstr, nVar, i, qrmanager.tau);
        }
        qrmanager.QR.set_size(B.size(0), B.size(1));
        iAcol = B.size(0) * B.size(1);
        for (i = 0; i < iAcol; i++) {
          qrmanager.QR[i] = B[i];
        }
      }
      QRManager::computeQ_(qrmanager, qrmanager.mrows);
      ldq = qrmanager.ldq;
      ldw = workspace.size(0);
      B.set_size(workspace.size(0), workspace.size(1));
      iAcol = workspace.size(0) * workspace.size(1);
      for (i = 0; i < iAcol; i++) {
        B[i] = workspace[i];
      }
      if (nVar != 0) {
        for (iac = 0; ldw < 0 ? iac >= ldw : iac <= ldw; iac += ldw) {
          i = iac + 1;
          i1 = iac + nVar;
          for (jBcol = i; jBcol <= i1; jBcol++) {
            workspace[jBcol - 1] = 0.0;
          }
        }
        br = -1;
        for (iac = 0; ldw < 0 ? iac >= ldw : iac <= ldw; iac += ldw) {
          ar = -1;
          i = iac + 1;
          i1 = iac + nVar;
          for (jBcol = i; jBcol <= i1; jBcol++) {
            c = 0.0;
            for (iy = 0; iy < mWConstr; iy++) {
              c += qrmanager.Q[(iy + ar) + 1] * B[(iy + br) + 1];
            }
            workspace[jBcol - 1] = workspace[jBcol - 1] + c;
            ar += ldq;
          }
          br += ldw;
        }
      }
      if ((workspace.size(0) != 0) && (workspace.size(1) != 0)) {
        for (mEq = 0; mEq < 2; mEq++) {
          jBcol = ldw * mEq - 1;
          for (int k{nVar}; k >= 1; k--) {
            iy = ldq * (k - 1) - 1;
            i = k + jBcol;
            c = workspace[i];
            if (c != 0.0) {
              workspace[i] = c / qrmanager.QR[k + iy];
              for (iac = 0; iac <= k - 2; iac++) {
                i1 = (iac + jBcol) + 1;
                workspace[i1] =
                    workspace[i1] - workspace[i] * qrmanager.QR[(iac + iy) + 1];
              }
            }
          }
        }
      }
    } else {
      int ldw;
      QRManager::factorQR(qrmanager, workingset.ATwset, nVar, mWConstr,
                          workingset.ldA);
      QRManager::computeQ_(qrmanager, qrmanager.minRowCol);
      ldq = qrmanager.ldq;
      ldw = workspace.size(0);
      if ((workspace.size(0) != 0) && (workspace.size(1) != 0)) {
        for (mEq = 0; mEq < 2; mEq++) {
          jBcol = ldw * mEq;
          for (iac = 0; iac < mWConstr; iac++) {
            iAcol = ldq * iac;
            iy = iac + jBcol;
            c = workspace[iy];
            for (int k{0}; k < iac; k++) {
              c -= qrmanager.QR[k + iAcol] * workspace[k + jBcol];
            }
            workspace[iy] = c / qrmanager.QR[iac + iAcol];
          }
        }
      }
      B.set_size(workspace.size(0), workspace.size(1));
      iAcol = workspace.size(0) * workspace.size(1);
      for (i = 0; i < iAcol; i++) {
        B[i] = workspace[i];
      }
      if (nVar != 0) {
        for (iac = 0; ldw < 0 ? iac >= ldw : iac <= ldw; iac += ldw) {
          i = iac + 1;
          i1 = iac + nVar;
          for (jBcol = i; jBcol <= i1; jBcol++) {
            workspace[jBcol - 1] = 0.0;
          }
        }
        br = 0;
        for (iac = 0; ldw < 0 ? iac >= ldw : iac <= ldw; iac += ldw) {
          ar = -1;
          i = br + 1;
          i1 = br + mWConstr;
          for (mEq = i; mEq <= i1; mEq++) {
            iy = iac + 1;
            iAcol = iac + nVar;
            for (jBcol = iy; jBcol <= iAcol; jBcol++) {
              workspace[jBcol - 1] =
                  workspace[jBcol - 1] +
                  B[mEq - 1] * qrmanager.Q[(ar + jBcol) - iac];
            }
            ar += ldq;
          }
          br += ldw;
        }
      }
    }
    ldq = 0;
    int exitg1;
    do {
      exitg1 = 0;
      if (ldq <= nVar - 1) {
        c = workspace[ldq];
        if (std::isinf(c) || std::isnan(c)) {
          nonDegenerateWset = false;
          exitg1 = 1;
        } else {
          c = workspace[ldq + workspace.size(0)];
          if (std::isinf(c) || std::isnan(c)) {
            nonDegenerateWset = false;
            exitg1 = 1;
          } else {
            ldq++;
          }
        }
      } else {
        double constrViolation_minnormX;
        double v;
        if (nVar >= 1) {
          iAcol = nVar - 1;
          for (int k{0}; k <= iAcol; k++) {
            workspace[k] = workspace[k] + xCurrent[k];
          }
        }
        constrViolation_minnormX =
            WorkingSet::maxConstraintViolation(workingset, workspace);
        iac = workspace.size(0);
        jBcol = workingset.sizes[3];
        br = workingset.sizes[4];
        ar = workingset.sizes[0];
        if (workingset.probType == 2) {
          v = 0.0;
          iAcol = workingset.sizes[2] - 1;
          mEq = workingset.sizes[1] - 1;
          if (workingset.Aineq.size(0) != 0) {
            for (int k{0}; k <= iAcol; k++) {
              workingset.maxConstrWorkspace[k] = workingset.bineq[k];
            }
            internal::blas::xgemv(workingset.nVarOrig, workingset.sizes[2],
                                  workingset.Aineq, workingset.ldA, workspace,
                                  workspace.size(0) + 1,
                                  workingset.maxConstrWorkspace);
            for (ldq = 0; ldq <= iAcol; ldq++) {
              workingset.maxConstrWorkspace[ldq] =
                  workingset.maxConstrWorkspace[ldq] -
                  workspace[(iac + workingset.nVarOrig) + ldq];
              v = std::fmax(v, workingset.maxConstrWorkspace[ldq]);
            }
          }
          if (workingset.Aeq.size(0) != 0) {
            for (int k{0}; k <= mEq; k++) {
              workingset.maxConstrWorkspace[k] = workingset.beq[k];
            }
            internal::blas::xgemv(workingset.nVarOrig, workingset.sizes[1],
                                  workingset.Aeq, workingset.ldA, workspace,
                                  workspace.size(0) + 1,
                                  workingset.maxConstrWorkspace);
            iAcol = workingset.nVarOrig + workingset.sizes[2];
            iy = iAcol + workingset.sizes[1];
            for (ldq = 0; ldq <= mEq; ldq++) {
              c = (workingset.maxConstrWorkspace[ldq] -
                   workspace[(iac + iAcol) + ldq]) +
                  workspace[(iac + iy) + ldq];
              workingset.maxConstrWorkspace[ldq] = c;
              v = std::fmax(v, std::abs(c));
            }
          }
        } else {
          v = 0.0;
          iAcol = workingset.sizes[2] - 1;
          mEq = workingset.sizes[1] - 1;
          if (workingset.Aineq.size(0) != 0) {
            for (int k{0}; k <= iAcol; k++) {
              workingset.maxConstrWorkspace[k] = workingset.bineq[k];
            }
            internal::blas::xgemv(workingset.nVar, workingset.sizes[2],
                                  workingset.Aineq, workingset.ldA, workspace,
                                  workspace.size(0) + 1,
                                  workingset.maxConstrWorkspace);
            for (ldq = 0; ldq <= iAcol; ldq++) {
              v = std::fmax(v, workingset.maxConstrWorkspace[ldq]);
            }
          }
          if (workingset.Aeq.size(0) != 0) {
            for (int k{0}; k <= mEq; k++) {
              workingset.maxConstrWorkspace[k] = workingset.beq[k];
            }
            internal::blas::xgemv(workingset.nVar, workingset.sizes[1],
                                  workingset.Aeq, workingset.ldA, workspace,
                                  workspace.size(0) + 1,
                                  workingset.maxConstrWorkspace);
            for (ldq = 0; ldq <= mEq; ldq++) {
              v = std::fmax(v, std::abs(workingset.maxConstrWorkspace[ldq]));
            }
          }
        }
        if (workingset.sizes[3] > 0) {
          for (ldq = 0; ldq < jBcol; ldq++) {
            v = std::fmax(v, -workspace[(iac + workingset.indexLB[ldq]) - 1] -
                                 workingset.lb[workingset.indexLB[ldq] - 1]);
          }
        }
        if (workingset.sizes[4] > 0) {
          for (ldq = 0; ldq < br; ldq++) {
            v = std::fmax(v, workspace[(iac + workingset.indexUB[ldq]) - 1] -
                                 workingset.ub[workingset.indexUB[ldq] - 1]);
          }
        }
        if (workingset.sizes[0] > 0) {
          for (ldq = 0; ldq < ar; ldq++) {
            v = std::fmax(
                v, std::abs(workspace[(iac + workingset.indexFixed[ldq]) - 1] -
                            workingset.ub[workingset.indexFixed[ldq] - 1]));
          }
        }
        if ((constrViolation_minnormX <= 2.2204460492503131E-16) ||
            (constrViolation_minnormX < v)) {
          for (int k{0}; k < nVar; k++) {
            xCurrent[k] = workspace[k];
          }
        } else {
          for (int k{0}; k < nVar; k++) {
            xCurrent[k] = workspace[workspace.size(0) + k];
          }
        }
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  return nonDegenerateWset;
}

} // namespace initialize
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (feasibleX0ForWorkingSet.cpp)
