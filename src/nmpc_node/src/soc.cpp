//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// soc.cpp
//
// Code generation for function 'soc'
//

// Include files
#include "soc.h"
#include "NMPC_Node_internal_types.h"
#include "addAeqConstr.h"
#include "addBoundToActiveSetMatrix_.h"
#include "driver.h"
#include "rt_nonfinite.h"
#include "sortLambdaQP.h"
#include "xnrm2.h"
#include "coder_array.h"
#include <cstring>

// Function Definitions
namespace coder {
namespace optim {
namespace coder {
namespace fminconsqp {
namespace step {
bool soc(const double Hessian[12769], const ::coder::array<double, 1U> &grad,
         d_struct_T &b_TrialState, h_struct_T &memspace, i_struct_T &WorkingSet,
         e_struct_T &b_QRManager, f_struct_T &b_CholManager,
         g_struct_T &QPObjective, const n_struct_T &qpoptions)
{
  double c;
  int i;
  int i1;
  int idxIneqOffset;
  int idx_Aineq;
  int idx_Partition;
  int idx_upper;
  int iy;
  int mConstrMax;
  int mIneq;
  int nVar_tmp_tmp;
  int nWIneq_old;
  int nWLower_old;
  int nWUpper_old;
  bool success;
  nWIneq_old = WorkingSet.nWConstr[2];
  nWLower_old = WorkingSet.nWConstr[3];
  nWUpper_old = WorkingSet.nWConstr[4];
  nVar_tmp_tmp = WorkingSet.nVar - 1;
  mConstrMax = WorkingSet.mConstrMax - 1;
  for (idx_Aineq = 0; idx_Aineq <= nVar_tmp_tmp; idx_Aineq++) {
    b_TrialState.xstarsqp[idx_Aineq] = b_TrialState.xstarsqp_old[idx_Aineq];
    b_TrialState.socDirection[idx_Aineq] = b_TrialState.xstar[idx_Aineq];
  }
  for (idx_Aineq = 0; idx_Aineq <= mConstrMax; idx_Aineq++) {
    b_TrialState.lambdaStopTest[idx_Aineq] = b_TrialState.lambda[idx_Aineq];
  }
  idx_upper = WorkingSet.sizes[1] - 1;
  mIneq = WorkingSet.sizes[2];
  idxIneqOffset = WorkingSet.isActiveIdx[2];
  if (WorkingSet.sizes[1] > 0) {
    for (int idx{0}; idx <= idx_upper; idx++) {
      WorkingSet.beq[idx] = -b_TrialState.cEq[idx];
    }
    idx_Aineq = WorkingSet.ldA;
    if (WorkingSet.nVar != 0) {
      iy = 0;
      i = WorkingSet.ldA * (WorkingSet.sizes[1] - 1) + 1;
      for (idx_Partition = 1;
           idx_Aineq < 0 ? idx_Partition >= i : idx_Partition <= i;
           idx_Partition += idx_Aineq) {
        c = 0.0;
        i1 = (idx_Partition + WorkingSet.nVar) - 1;
        for (int idx{idx_Partition}; idx <= i1; idx++) {
          c += WorkingSet.Aeq[idx - 1] *
               b_TrialState.searchDir[idx - idx_Partition];
        }
        WorkingSet.beq[iy] += c;
        iy++;
      }
    }
    for (idx_Aineq = 0; idx_Aineq <= idx_upper; idx_Aineq++) {
      WorkingSet.bwset[WorkingSet.sizes[0] + idx_Aineq] =
          WorkingSet.beq[idx_Aineq];
    }
  }
  if (WorkingSet.sizes[2] > 0) {
    for (int idx{0}; idx < mIneq; idx++) {
      WorkingSet.bineq[idx] = -b_TrialState.cIneq[idx];
    }
    idx_Aineq = WorkingSet.ldA;
    if (WorkingSet.nVar != 0) {
      iy = 0;
      i = WorkingSet.ldA * (WorkingSet.sizes[2] - 1) + 1;
      for (idx_Partition = 1;
           idx_Aineq < 0 ? idx_Partition >= i : idx_Partition <= i;
           idx_Partition += idx_Aineq) {
        c = 0.0;
        i1 = (idx_Partition + WorkingSet.nVar) - 1;
        for (int idx{idx_Partition}; idx <= i1; idx++) {
          c += WorkingSet.Aineq[idx - 1] *
               b_TrialState.searchDir[idx - idx_Partition];
        }
        WorkingSet.bineq[iy] = WorkingSet.bineq[iy] + c;
        iy++;
      }
    }
    idx_Aineq = 1;
    iy = WorkingSet.sizes[2] + 1;
    idx_upper = (WorkingSet.sizes[2] + WorkingSet.sizes[3]) + 1;
    i = WorkingSet.nActiveConstr;
    for (int idx{idxIneqOffset}; idx <= i; idx++) {
      switch (WorkingSet.Wid[idx - 1]) {
      case 3:
        idx_Partition = idx_Aineq;
        idx_Aineq++;
        WorkingSet.bwset[idx - 1] =
            WorkingSet.bineq[WorkingSet.Wlocalidx[idx - 1] - 1];
        break;
      case 4:
        idx_Partition = iy;
        iy++;
        break;
      default:
        idx_Partition = idx_upper;
        idx_upper++;
        break;
      }
      b_TrialState.workingset_old[idx_Partition - 1] =
          WorkingSet.Wlocalidx[idx - 1];
    }
  }
  for (idx_Aineq = 0; idx_Aineq <= nVar_tmp_tmp; idx_Aineq++) {
    b_TrialState.xstar[idx_Aineq] = b_TrialState.xstarsqp[idx_Aineq];
  }
  n_struct_T b_qpoptions;
  b_qpoptions = qpoptions;
  ::coder::optim::coder::qpactiveset::driver(
      Hessian, grad, b_TrialState, memspace, WorkingSet, b_QRManager,
      b_CholManager, QPObjective, qpoptions, b_qpoptions);
  while ((WorkingSet.mEqRemoved > 0) &&
         (WorkingSet.indexEqRemoved[WorkingSet.mEqRemoved - 1] >=
          b_TrialState.iNonEq0)) {
    qpactiveset::WorkingSet::addAeqConstr(
        WorkingSet, WorkingSet.indexEqRemoved[WorkingSet.mEqRemoved - 1]);
    WorkingSet.mEqRemoved--;
  }
  for (int idx{0}; idx <= nVar_tmp_tmp; idx++) {
    double oldDirIdx;
    c = b_TrialState.socDirection[idx];
    oldDirIdx = c;
    c = b_TrialState.xstar[idx] - c;
    b_TrialState.socDirection[idx] = c;
    b_TrialState.xstar[idx] = oldDirIdx;
  }
  success = (::coder::internal::blas::xnrm2(nVar_tmp_tmp + 1,
                                            b_TrialState.socDirection) <=
             2.0 * ::coder::internal::blas::xnrm2(nVar_tmp_tmp + 1,
                                                  b_TrialState.xstar));
  i = WorkingSet.sizes[1] - 1;
  mIneq = WorkingSet.sizes[2];
  idx_Partition = WorkingSet.sizes[3];
  if (WorkingSet.sizes[1] > 0) {
    for (int idx{0}; idx <= i; idx++) {
      WorkingSet.beq[idx] = -b_TrialState.cEq[idx];
      WorkingSet.bwset[WorkingSet.sizes[0] + idx] = WorkingSet.beq[idx];
    }
  }
  if (WorkingSet.sizes[2] > 0) {
    for (int idx{0}; idx < mIneq; idx++) {
      WorkingSet.bineq[idx] = -b_TrialState.cIneq[idx];
    }
    if (!success) {
      idx_Aineq = WorkingSet.nWConstr[0] + WorkingSet.nWConstr[1];
      iy = idx_Aineq + 1;
      idx_upper = WorkingSet.nActiveConstr;
      for (nVar_tmp_tmp = iy; nVar_tmp_tmp <= idx_upper; nVar_tmp_tmp++) {
        WorkingSet.isActiveConstr
            [(WorkingSet.isActiveIdx[WorkingSet.Wid[nVar_tmp_tmp - 1] - 1] +
              WorkingSet.Wlocalidx[nVar_tmp_tmp - 1]) -
             2] = false;
      }
      WorkingSet.nWConstr[2] = 0;
      WorkingSet.nWConstr[3] = 0;
      WorkingSet.nWConstr[4] = 0;
      WorkingSet.nActiveConstr = idx_Aineq;
      for (int idx{0}; idx < nWIneq_old; idx++) {
        idx_Aineq = b_TrialState.workingset_old[idx];
        WorkingSet.nWConstr[2]++;
        WorkingSet.isActiveConstr[(WorkingSet.isActiveIdx[2] + idx_Aineq) - 2] =
            true;
        WorkingSet.nActiveConstr++;
        i = WorkingSet.nActiveConstr - 1;
        WorkingSet.Wid[i] = 3;
        WorkingSet.Wlocalidx[i] = idx_Aineq;
        iy = WorkingSet.ldA * (idx_Aineq - 1);
        idx_upper = WorkingSet.ldA * i;
        i1 = WorkingSet.nVar - 1;
        for (nVar_tmp_tmp = 0; nVar_tmp_tmp <= i1; nVar_tmp_tmp++) {
          WorkingSet.ATwset[idx_upper + nVar_tmp_tmp] =
              WorkingSet.Aineq[iy + nVar_tmp_tmp];
        }
        WorkingSet.bwset[i] = WorkingSet.bineq[idx_Aineq - 1];
      }
      for (int idx{0}; idx < nWLower_old; idx++) {
        qpactiveset::WorkingSet::addBoundToActiveSetMatrix_(
            WorkingSet, 4, b_TrialState.workingset_old[idx + mIneq]);
      }
      for (int idx{0}; idx < nWUpper_old; idx++) {
        qpactiveset::WorkingSet::addBoundToActiveSetMatrix_(
            WorkingSet, 5,
            b_TrialState.workingset_old[(idx + mIneq) + idx_Partition]);
      }
    }
  }
  if (!success) {
    for (idx_Aineq = 0; idx_Aineq <= mConstrMax; idx_Aineq++) {
      b_TrialState.lambda[idx_Aineq] = b_TrialState.lambdaStopTest[idx_Aineq];
    }
  } else {
    qpactiveset::parseoutput::sortLambdaQP(
        b_TrialState.lambda, WorkingSet.nActiveConstr, WorkingSet.sizes,
        WorkingSet.isActiveIdx, WorkingSet.Wid, WorkingSet.Wlocalidx,
        memspace.workspace_double);
  }
  return success;
}

} // namespace step
} // namespace fminconsqp
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (soc.cpp)
