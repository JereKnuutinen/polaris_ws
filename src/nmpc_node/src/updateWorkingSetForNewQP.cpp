//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// updateWorkingSetForNewQP.cpp
//
// Code generation for function 'updateWorkingSetForNewQP'
//

// Include files
#include "updateWorkingSetForNewQP.h"
#include "NMPC_Node_internal_types.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <cstring>

// Function Definitions
namespace coder {
namespace optim {
namespace coder {
namespace fminconsqp {
namespace internal {
void updateWorkingSetForNewQP(const double xk[113], i_struct_T &WorkingSet,
                              int mIneq, int mNonlinIneq,
                              const ::coder::array<double, 1U> &cIneq,
                              int mNonlinEq, const double cEq[98], int mLB,
                              const double lb[113], int mUB,
                              const double ub[113], int mFixed)
{
  int i;
  int iEq0;
  int iw0;
  int nVar;
  nVar = WorkingSet.nVar - 1;
  for (int idx{0}; idx < 98; idx++) {
    WorkingSet.beq[idx] = -cEq[idx];
    WorkingSet.bwset[mFixed + idx] = WorkingSet.beq[idx];
  }
  iw0 = WorkingSet.ldA * ((mFixed - mNonlinEq) + 98);
  iEq0 = WorkingSet.ldA * (98 - mNonlinEq);
  for (int idx{0}; idx < mNonlinEq; idx++) {
    for (i = 0; i <= nVar; i++) {
      WorkingSet.ATwset[iw0 + i] = WorkingSet.Aeq[iEq0 + i];
    }
    iw0 += WorkingSet.ldA;
    iEq0 += WorkingSet.ldA;
  }
  for (int idx{0}; idx < mIneq; idx++) {
    WorkingSet.bineq[idx] = -cIneq[idx];
  }
  for (int idx{0}; idx < mLB; idx++) {
    WorkingSet.lb[WorkingSet.indexLB[idx] - 1] =
        -lb[WorkingSet.indexLB[idx] - 1] + xk[WorkingSet.indexLB[idx] - 1];
  }
  for (int idx{0}; idx < mUB; idx++) {
    WorkingSet.ub[WorkingSet.indexUB[idx] - 1] =
        ub[WorkingSet.indexUB[idx] - 1] - xk[WorkingSet.indexUB[idx] - 1];
  }
  for (int idx{0}; idx < mFixed; idx++) {
    double d;
    d = ub[WorkingSet.indexFixed[idx] - 1] - xk[WorkingSet.indexFixed[idx] - 1];
    WorkingSet.ub[WorkingSet.indexFixed[idx] - 1] = d;
    WorkingSet.bwset[idx] = d;
  }
  if (WorkingSet.nActiveConstr > mFixed + 98) {
    iEq0 = mFixed + 99;
    if (iEq0 < 1) {
      iEq0 = 1;
    }
    i = WorkingSet.nActiveConstr;
    for (int idx{iEq0}; idx <= i; idx++) {
      switch (WorkingSet.Wid[idx - 1]) {
      case 4:
        WorkingSet.bwset[idx - 1] =
            WorkingSet
                .lb[WorkingSet.indexLB[WorkingSet.Wlocalidx[idx - 1] - 1] - 1];
        break;
      case 5:
        WorkingSet.bwset[idx - 1] =
            WorkingSet
                .ub[WorkingSet.indexUB[WorkingSet.Wlocalidx[idx - 1] - 1] - 1];
        break;
      default: {
        iw0 = WorkingSet.Wlocalidx[idx - 1];
        WorkingSet.bwset[idx - 1] = WorkingSet.bineq[iw0 - 1];
        if ((mNonlinIneq > 0) && (iw0 >= mNonlinIneq)) {
          int iy0;
          iy0 = WorkingSet.ldA * (idx - 1);
          iw0 = WorkingSet.ldA * (iw0 - 1);
          for (int k{0}; k <= nVar; k++) {
            WorkingSet.ATwset[iy0 + k] = WorkingSet.Aineq[iw0 + k];
          }
        }
      } break;
      }
    }
  }
}

void updateWorkingSetForNewQP(const double xk[113], i_struct_T &WorkingSet,
                              int mIneq, int mNonlinIneq,
                              const ::coder::array<double, 1U> &cIneq, int mEq,
                              int mNonlinEq, const double cEq[98], int mLB,
                              const double lb[113], int mUB,
                              const double ub[113], int mFixed)
{
  int ineqStart;
  int iw0;
  int mLinEq;
  int nVar;
  nVar = WorkingSet.nVar - 1;
  for (int idx{0}; idx < mEq; idx++) {
    WorkingSet.beq[idx] = -cEq[idx];
    WorkingSet.bwset[mFixed + idx] = WorkingSet.beq[idx];
  }
  mLinEq = mEq - mNonlinEq;
  iw0 = WorkingSet.ldA * (mFixed + mLinEq);
  mLinEq *= WorkingSet.ldA;
  for (int idx{0}; idx < mNonlinEq; idx++) {
    for (ineqStart = 0; ineqStart <= nVar; ineqStart++) {
      WorkingSet.ATwset[iw0 + ineqStart] = WorkingSet.Aeq[mLinEq + ineqStart];
    }
    iw0 += WorkingSet.ldA;
    mLinEq += WorkingSet.ldA;
  }
  for (int idx{0}; idx < mIneq; idx++) {
    WorkingSet.bineq[idx] = -cIneq[idx];
  }
  for (int idx{0}; idx < mLB; idx++) {
    WorkingSet.lb[WorkingSet.indexLB[idx] - 1] =
        -lb[WorkingSet.indexLB[idx] - 1] + xk[WorkingSet.indexLB[idx] - 1];
  }
  for (int idx{0}; idx < mUB; idx++) {
    WorkingSet.ub[WorkingSet.indexUB[idx] - 1] =
        ub[WorkingSet.indexUB[idx] - 1] - xk[WorkingSet.indexUB[idx] - 1];
  }
  for (int idx{0}; idx < mFixed; idx++) {
    double d;
    d = ub[WorkingSet.indexFixed[idx] - 1] - xk[WorkingSet.indexFixed[idx] - 1];
    WorkingSet.ub[WorkingSet.indexFixed[idx] - 1] = d;
    WorkingSet.bwset[idx] = d;
  }
  iw0 = mFixed + mEq;
  if (WorkingSet.nActiveConstr > iw0) {
    ineqStart = iw0 + 1;
    if (ineqStart < 1) {
      ineqStart = 1;
    }
    iw0 = WorkingSet.nActiveConstr;
    for (int idx{ineqStart}; idx <= iw0; idx++) {
      switch (WorkingSet.Wid[idx - 1]) {
      case 4:
        WorkingSet.bwset[idx - 1] =
            WorkingSet
                .lb[WorkingSet.indexLB[WorkingSet.Wlocalidx[idx - 1] - 1] - 1];
        break;
      case 5:
        WorkingSet.bwset[idx - 1] =
            WorkingSet
                .ub[WorkingSet.indexUB[WorkingSet.Wlocalidx[idx - 1] - 1] - 1];
        break;
      default: {
        mLinEq = WorkingSet.Wlocalidx[idx - 1];
        WorkingSet.bwset[idx - 1] = WorkingSet.bineq[mLinEq - 1];
        if ((mNonlinIneq > 0) && (mLinEq >= mNonlinIneq)) {
          int iy0;
          iy0 = WorkingSet.ldA * (idx - 1);
          mLinEq = WorkingSet.ldA * (mLinEq - 1);
          for (int k{0}; k <= nVar; k++) {
            WorkingSet.ATwset[iy0 + k] = WorkingSet.Aineq[mLinEq + k];
          }
        }
      } break;
      }
    }
  }
}

} // namespace internal
} // namespace fminconsqp
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (updateWorkingSetForNewQP.cpp)
