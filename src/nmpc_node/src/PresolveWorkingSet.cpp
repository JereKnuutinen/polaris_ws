//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// PresolveWorkingSet.cpp
//
// Code generation for function 'PresolveWorkingSet'
//

// Include files
#include "PresolveWorkingSet.h"
#include "NMPC_Node_internal_types.h"
#include "computeQ_.h"
#include "countsort.h"
#include "feasibleX0ForWorkingSet.h"
#include "maxConstraintViolation.h"
#include "removeConstr.h"
#include "rt_nonfinite.h"
#include "xgeqp3.h"
#include "coder_array.h"
#include <cmath>
#include <cstring>

// Function Definitions
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace initialize {
void PresolveWorkingSet(d_struct_T &solution, h_struct_T &memspace,
                        i_struct_T &workingset, e_struct_T &qrmanager,
                        const n_struct_T &options)
{
  double tol;
  int i;
  int idxDiag;
  int idx_col;
  int idx_row;
  int ix;
  int mTotalWorkingEq_tmp_tmp;
  int mWorkingFixed;
  int nDepInd;
  int nVar;
  solution.state = 82;
  nVar = workingset.nVar - 1;
  mWorkingFixed = workingset.nWConstr[0];
  mTotalWorkingEq_tmp_tmp = workingset.nWConstr[0] + workingset.nWConstr[1];
  nDepInd = 0;
  if (mTotalWorkingEq_tmp_tmp > 0) {
    for (idx_row = 0; idx_row < mTotalWorkingEq_tmp_tmp; idx_row++) {
      for (idx_col = 0; idx_col <= nVar; idx_col++) {
        qrmanager.QR[idx_row + qrmanager.ldq * idx_col] =
            workingset.ATwset[idx_col + workingset.ldA * idx_row];
      }
    }
    nDepInd = mTotalWorkingEq_tmp_tmp - workingset.nVar;
    if (nDepInd <= 0) {
      nDepInd = 0;
    }
    for (idx_col = 0; idx_col <= nVar; idx_col++) {
      qrmanager.jpvt[idx_col] = 0;
    }
    i = mTotalWorkingEq_tmp_tmp * workingset.nVar;
    if (i == 0) {
      qrmanager.mrows = mTotalWorkingEq_tmp_tmp;
      qrmanager.ncols = workingset.nVar;
      qrmanager.minRowCol = 0;
    } else {
      qrmanager.usedPivoting = true;
      qrmanager.mrows = mTotalWorkingEq_tmp_tmp;
      qrmanager.ncols = workingset.nVar;
      ix = workingset.nVar;
      if (mTotalWorkingEq_tmp_tmp <= ix) {
        ix = mTotalWorkingEq_tmp_tmp;
      }
      qrmanager.minRowCol = ix;
      internal::lapack::xgeqp3(qrmanager.QR, mTotalWorkingEq_tmp_tmp,
                               workingset.nVar, qrmanager.jpvt, qrmanager.tau);
    }
    tol = 100.0 * static_cast<double>(workingset.nVar) * 2.2204460492503131E-16;
    idx_row = workingset.nVar;
    if (idx_row > mTotalWorkingEq_tmp_tmp) {
      idx_row = mTotalWorkingEq_tmp_tmp;
    }
    idxDiag = idx_row + qrmanager.ldq * (idx_row - 1);
    while ((idxDiag > 0) && (std::abs(qrmanager.QR[idxDiag - 1]) < tol)) {
      idxDiag = (idxDiag - qrmanager.ldq) - 1;
      nDepInd++;
    }
    if (nDepInd > 0) {
      bool exitg1;
      QRManager::computeQ_(qrmanager, qrmanager.mrows);
      idx_col = 0;
      exitg1 = false;
      while ((!exitg1) && (idx_col <= nDepInd - 1)) {
        double qtb;
        ix = qrmanager.ldq * ((mTotalWorkingEq_tmp_tmp - idx_col) - 1);
        qtb = 0.0;
        for (int k{0}; k < mTotalWorkingEq_tmp_tmp; k++) {
          qtb += qrmanager.Q[ix + k] * workingset.bwset[k];
        }
        if (std::abs(qtb) >= tol) {
          nDepInd = -1;
          exitg1 = true;
        } else {
          idx_col++;
        }
      }
    }
    if (nDepInd > 0) {
      for (idx_col = 0; idx_col < mTotalWorkingEq_tmp_tmp; idx_col++) {
        idxDiag = qrmanager.ldq * idx_col;
        ix = workingset.ldA * idx_col;
        for (int k{0}; k <= nVar; k++) {
          qrmanager.QR[idxDiag + k] = workingset.ATwset[ix + k];
        }
      }
      for (idx_col = 0; idx_col < mWorkingFixed; idx_col++) {
        qrmanager.jpvt[idx_col] = 1;
      }
      mWorkingFixed = workingset.nWConstr[0] + 1;
      for (idx_col = mWorkingFixed; idx_col <= mTotalWorkingEq_tmp_tmp;
           idx_col++) {
        qrmanager.jpvt[idx_col - 1] = 0;
      }
      if (i == 0) {
        qrmanager.mrows = workingset.nVar;
        qrmanager.ncols = mTotalWorkingEq_tmp_tmp;
        qrmanager.minRowCol = 0;
      } else {
        qrmanager.usedPivoting = true;
        qrmanager.mrows = workingset.nVar;
        qrmanager.ncols = mTotalWorkingEq_tmp_tmp;
        qrmanager.minRowCol = idx_row;
        internal::lapack::xgeqp3(qrmanager.QR, workingset.nVar,
                                 mTotalWorkingEq_tmp_tmp, qrmanager.jpvt,
                                 qrmanager.tau);
      }
      for (idx_col = 0; idx_col < nDepInd; idx_col++) {
        memspace.workspace_int[idx_col] =
            qrmanager.jpvt[(mTotalWorkingEq_tmp_tmp - nDepInd) + idx_col];
      }
      utils::countsort(memspace.workspace_int, nDepInd, memspace.workspace_sort,
                       1, mTotalWorkingEq_tmp_tmp);
      for (idx_col = nDepInd; idx_col >= 1; idx_col--) {
        i = workingset.nWConstr[0] + workingset.nWConstr[1];
        if (i != 0) {
          mWorkingFixed = memspace.workspace_int[idx_col - 1];
          if (mWorkingFixed <= i) {
            if ((workingset.nActiveConstr == i) || (mWorkingFixed == i)) {
              workingset.mEqRemoved++;
              workingset.indexEqRemoved[workingset.mEqRemoved - 1] =
                  workingset.Wlocalidx[mWorkingFixed - 1];
              WorkingSet::removeConstr(workingset, mWorkingFixed);
            } else {
              workingset.mEqRemoved++;
              ix = workingset.Wid[mWorkingFixed - 1] - 1;
              workingset.indexEqRemoved[workingset.mEqRemoved - 1] =
                  workingset.Wlocalidx[mWorkingFixed - 1];
              workingset
                  .isActiveConstr[(workingset.isActiveIdx
                                       [workingset.Wid[mWorkingFixed - 1] - 1] +
                                   workingset.Wlocalidx[mWorkingFixed - 1]) -
                                  2] = false;
              workingset.Wid[mWorkingFixed - 1] = workingset.Wid[i - 1];
              workingset.Wlocalidx[mWorkingFixed - 1] =
                  workingset.Wlocalidx[i - 1];
              idxDiag = workingset.nVar;
              for (idx_row = 0; idx_row < idxDiag; idx_row++) {
                workingset
                    .ATwset[idx_row + workingset.ldA * (mWorkingFixed - 1)] =
                    workingset.ATwset[idx_row + workingset.ldA * (i - 1)];
              }
              workingset.bwset[mWorkingFixed - 1] = workingset.bwset[i - 1];
              workingset.Wid[i - 1] =
                  workingset.Wid[workingset.nActiveConstr - 1];
              workingset.Wlocalidx[i - 1] =
                  workingset.Wlocalidx[workingset.nActiveConstr - 1];
              mWorkingFixed = workingset.nVar;
              for (idx_row = 0; idx_row < mWorkingFixed; idx_row++) {
                workingset.ATwset[idx_row + workingset.ldA * (i - 1)] =
                    workingset
                        .ATwset[idx_row + workingset.ldA *
                                              (workingset.nActiveConstr - 1)];
              }
              workingset.bwset[i - 1] =
                  workingset.bwset[workingset.nActiveConstr - 1];
              workingset.nActiveConstr--;
              workingset.nWConstr[ix]--;
            }
          }
        }
      }
    }
  }
  if ((nDepInd != -1) && (workingset.nActiveConstr <= qrmanager.ldq)) {
    bool guard1{false};
    bool okWorkingSet;
    ix = workingset.nActiveConstr;
    i = workingset.nWConstr[0] + workingset.nWConstr[1];
    nVar = workingset.nVar;
    if ((workingset.nWConstr[2] + workingset.nWConstr[3]) +
            workingset.nWConstr[4] >
        0) {
      tol =
          100.0 * static_cast<double>(workingset.nVar) * 2.2204460492503131E-16;
      for (idx_col = 0; idx_col < i; idx_col++) {
        qrmanager.jpvt[idx_col] = 1;
      }
      mWorkingFixed = i + 1;
      for (idx_col = mWorkingFixed; idx_col <= ix; idx_col++) {
        qrmanager.jpvt[idx_col - 1] = 0;
      }
      mWorkingFixed = workingset.nActiveConstr;
      for (idx_col = 0; idx_col < mWorkingFixed; idx_col++) {
        idxDiag = qrmanager.ldq * idx_col;
        ix = workingset.ldA * idx_col;
        for (int k{0}; k < nVar; k++) {
          qrmanager.QR[idxDiag + k] = workingset.ATwset[ix + k];
        }
      }
      if (workingset.nVar * workingset.nActiveConstr == 0) {
        qrmanager.mrows = workingset.nVar;
        qrmanager.ncols = workingset.nActiveConstr;
        qrmanager.minRowCol = 0;
      } else {
        qrmanager.usedPivoting = true;
        qrmanager.mrows = workingset.nVar;
        qrmanager.ncols = workingset.nActiveConstr;
        idx_row = workingset.nVar;
        ix = workingset.nActiveConstr;
        if (idx_row <= ix) {
          ix = idx_row;
        }
        qrmanager.minRowCol = ix;
        internal::lapack::xgeqp3(qrmanager.QR, workingset.nVar,
                                 workingset.nActiveConstr, qrmanager.jpvt,
                                 qrmanager.tau);
      }
      ix = 0;
      for (idx_col = workingset.nActiveConstr - 1; idx_col + 1 > nVar;
           idx_col--) {
        ix++;
        memspace.workspace_int[ix - 1] = qrmanager.jpvt[idx_col];
      }
      if (idx_col + 1 <= workingset.nVar) {
        idxDiag = idx_col + qrmanager.ldq * idx_col;
        while ((idx_col + 1 > i) && (std::abs(qrmanager.QR[idxDiag]) < tol)) {
          ix++;
          memspace.workspace_int[ix - 1] = qrmanager.jpvt[idx_col];
          idx_col--;
          idxDiag = (idxDiag - qrmanager.ldq) - 1;
        }
      }
      utils::countsort(memspace.workspace_int, ix, memspace.workspace_sort,
                       i + 1, workingset.nActiveConstr);
      for (idx_col = ix; idx_col >= 1; idx_col--) {
        WorkingSet::removeConstr(workingset,
                                 memspace.workspace_int[idx_col - 1]);
      }
    }
    okWorkingSet = feasibleX0ForWorkingSet(
        memspace.workspace_double, solution.xstar, workingset, qrmanager);
    guard1 = false;
    if (!okWorkingSet) {
      ix = workingset.nActiveConstr;
      i = workingset.nWConstr[0] + workingset.nWConstr[1];
      nVar = workingset.nVar;
      if ((workingset.nWConstr[2] + workingset.nWConstr[3]) +
              workingset.nWConstr[4] >
          0) {
        tol = 1000.0 * static_cast<double>(workingset.nVar) *
              2.2204460492503131E-16;
        for (idx_col = 0; idx_col < i; idx_col++) {
          qrmanager.jpvt[idx_col] = 1;
        }
        mWorkingFixed = i + 1;
        for (idx_col = mWorkingFixed; idx_col <= ix; idx_col++) {
          qrmanager.jpvt[idx_col - 1] = 0;
        }
        mWorkingFixed = workingset.nActiveConstr;
        for (idx_col = 0; idx_col < mWorkingFixed; idx_col++) {
          idxDiag = qrmanager.ldq * idx_col;
          ix = workingset.ldA * idx_col;
          for (int k{0}; k < nVar; k++) {
            qrmanager.QR[idxDiag + k] = workingset.ATwset[ix + k];
          }
        }
        if (workingset.nVar * workingset.nActiveConstr == 0) {
          qrmanager.mrows = workingset.nVar;
          qrmanager.ncols = workingset.nActiveConstr;
          qrmanager.minRowCol = 0;
        } else {
          qrmanager.usedPivoting = true;
          qrmanager.mrows = workingset.nVar;
          qrmanager.ncols = workingset.nActiveConstr;
          idx_row = workingset.nVar;
          ix = workingset.nActiveConstr;
          if (idx_row <= ix) {
            ix = idx_row;
          }
          qrmanager.minRowCol = ix;
          internal::lapack::xgeqp3(qrmanager.QR, workingset.nVar,
                                   workingset.nActiveConstr, qrmanager.jpvt,
                                   qrmanager.tau);
        }
        ix = 0;
        for (idx_col = workingset.nActiveConstr - 1; idx_col + 1 > nVar;
             idx_col--) {
          ix++;
          memspace.workspace_int[ix - 1] = qrmanager.jpvt[idx_col];
        }
        if (idx_col + 1 <= workingset.nVar) {
          idxDiag = idx_col + qrmanager.ldq * idx_col;
          while ((idx_col + 1 > i) && (std::abs(qrmanager.QR[idxDiag]) < tol)) {
            ix++;
            memspace.workspace_int[ix - 1] = qrmanager.jpvt[idx_col];
            idx_col--;
            idxDiag = (idxDiag - qrmanager.ldq) - 1;
          }
        }
        utils::countsort(memspace.workspace_int, ix, memspace.workspace_sort,
                         i + 1, workingset.nActiveConstr);
        for (idx_col = ix; idx_col >= 1; idx_col--) {
          WorkingSet::removeConstr(workingset,
                                   memspace.workspace_int[idx_col - 1]);
        }
      }
      okWorkingSet = feasibleX0ForWorkingSet(
          memspace.workspace_double, solution.xstar, workingset, qrmanager);
      if (!okWorkingSet) {
        solution.state = -7;
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }
    if (guard1 &&
        (workingset.nWConstr[0] + workingset.nWConstr[1] == workingset.nVar)) {
      tol = WorkingSet::maxConstraintViolation(workingset, solution.xstar);
      if (tol > options.ConstraintTolerance) {
        solution.state = -2;
      }
    }
  } else {
    solution.state = -3;
    ix = (workingset.nWConstr[0] + workingset.nWConstr[1]) + 1;
    idxDiag = workingset.nActiveConstr;
    for (idx_row = ix; idx_row <= idxDiag; idx_row++) {
      workingset.isActiveConstr
          [(workingset.isActiveIdx[workingset.Wid[idx_row - 1] - 1] +
            workingset.Wlocalidx[idx_row - 1]) -
           2] = false;
    }
    workingset.nWConstr[2] = 0;
    workingset.nWConstr[3] = 0;
    workingset.nWConstr[4] = 0;
    workingset.nActiveConstr = workingset.nWConstr[0] + workingset.nWConstr[1];
  }
}

} // namespace initialize
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (PresolveWorkingSet.cpp)
