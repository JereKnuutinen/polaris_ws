//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// step.cpp
//
// Code generation for function 'step'
//

// Include files
#include "step.h"
#include "NMPC_Node_internal_types.h"
#include "addAeqConstr.h"
#include "driver.h"
#include "relaxed.h"
#include "rt_nonfinite.h"
#include "soc.h"
#include "sortLambdaQP.h"
#include "coder_array.h"
#include <cmath>
#include <cstring>

// Function Definitions
namespace coder {
namespace optim {
namespace coder {
namespace fminconsqp {
bool b_step(int &STEP_TYPE, double Hessian[12769], const double lb[113],
            const double ub[113], d_struct_T &b_TrialState,
            struct_T &b_MeritFunction, h_struct_T &memspace,
            i_struct_T &WorkingSet, e_struct_T &b_QRManager,
            f_struct_T &b_CholManager, g_struct_T &QPObjective,
            n_struct_T &qpoptions)
{
  array<double, 1U> r;
  n_struct_T b_qpoptions;
  double constrViolationEq;
  double constrViolationIneq;
  int i;
  int idxEndIneq;
  int idxStartIneq;
  int kend;
  int nVar;
  bool checkBoundViolation;
  bool stepSuccess;
  stepSuccess = true;
  checkBoundViolation = true;
  nVar = WorkingSet.nVar - 1;
  if (STEP_TYPE != 3) {
    for (idxStartIneq = 0; idxStartIneq <= nVar; idxStartIneq++) {
      b_TrialState.xstar[idxStartIneq] = b_TrialState.xstarsqp[idxStartIneq];
    }
  } else {
    for (idxStartIneq = 0; idxStartIneq <= nVar; idxStartIneq++) {
      b_TrialState.searchDir[idxStartIneq] = b_TrialState.xstar[idxStartIneq];
    }
  }
  int exitg1;
  bool guard1{false};
  do {
    exitg1 = 0;
    guard1 = false;
    switch (STEP_TYPE) {
    case 1: {
      bool nonlinEqRemoved;
      r.set_size(b_TrialState.grad.size(0));
      kend = b_TrialState.grad.size(0);
      for (i = 0; i < kend; i++) {
        r[i] = b_TrialState.grad[i];
      }
      b_qpoptions = qpoptions;
      ::coder::optim::coder::qpactiveset::driver(
          Hessian, r, b_TrialState, memspace, WorkingSet, b_QRManager,
          b_CholManager, QPObjective, qpoptions, b_qpoptions);
      if (b_TrialState.state > 0) {
        double constrViolation;
        double penaltyParamTrial;
        i = WorkingSet.sizes[2];
        penaltyParamTrial = b_MeritFunction.penaltyParam;
        constrViolationEq = 0.0;
        if (WorkingSet.sizes[1] >= 1) {
          kend = WorkingSet.sizes[1];
          for (idxStartIneq = 0; idxStartIneq < kend; idxStartIneq++) {
            constrViolationEq += std::abs(b_TrialState.cEq[idxStartIneq]);
          }
        }
        constrViolationIneq = 0.0;
        for (int idx{0}; idx < i; idx++) {
          constrViolation = b_TrialState.cIneq[idx];
          if (constrViolation > 0.0) {
            constrViolationIneq += constrViolation;
          }
        }
        constrViolation = constrViolationEq + constrViolationIneq;
        constrViolationEq = b_MeritFunction.linearizedConstrViol;
        b_MeritFunction.linearizedConstrViol = 0.0;
        constrViolationIneq = constrViolation + constrViolationEq;
        if ((constrViolationIneq > 2.2204460492503131E-16) &&
            (b_TrialState.fstar > 0.0)) {
          if (b_TrialState.sqpFval == 0.0) {
            penaltyParamTrial = 1.0;
          } else {
            penaltyParamTrial = 1.5;
          }
          penaltyParamTrial =
              penaltyParamTrial * b_TrialState.fstar / constrViolationIneq;
        }
        if (penaltyParamTrial < b_MeritFunction.penaltyParam) {
          b_MeritFunction.phi =
              b_TrialState.sqpFval + penaltyParamTrial * constrViolation;
          if ((b_MeritFunction.initFval +
               penaltyParamTrial * (b_MeritFunction.initConstrViolationEq +
                                    b_MeritFunction.initConstrViolationIneq)) -
                  b_MeritFunction.phi >
              static_cast<double>(b_MeritFunction.nPenaltyDecreases) *
                  b_MeritFunction.threshold) {
            b_MeritFunction.nPenaltyDecreases++;
            if ((b_MeritFunction.nPenaltyDecreases << 1) >
                b_TrialState.sqpIterations) {
              b_MeritFunction.threshold *= 10.0;
            }
            b_MeritFunction.penaltyParam =
                std::fmax(penaltyParamTrial, 1.0E-10);
          } else {
            b_MeritFunction.phi =
                b_TrialState.sqpFval +
                b_MeritFunction.penaltyParam * constrViolation;
          }
        } else {
          b_MeritFunction.penaltyParam = std::fmax(penaltyParamTrial, 1.0E-10);
          b_MeritFunction.phi = b_TrialState.sqpFval +
                                b_MeritFunction.penaltyParam * constrViolation;
        }
        b_MeritFunction.phiPrimePlus = std::fmin(
            b_TrialState.fstar - b_MeritFunction.penaltyParam * constrViolation,
            0.0);
      }
      qpactiveset::parseoutput::sortLambdaQP(
          b_TrialState.lambda, WorkingSet.nActiveConstr, WorkingSet.sizes,
          WorkingSet.isActiveIdx, WorkingSet.Wid, WorkingSet.Wlocalidx,
          memspace.workspace_double);
      nonlinEqRemoved = (WorkingSet.mEqRemoved > 0);
      while ((WorkingSet.mEqRemoved > 0) &&
             (WorkingSet.indexEqRemoved[WorkingSet.mEqRemoved - 1] >=
              b_TrialState.iNonEq0)) {
        qpactiveset::WorkingSet::addAeqConstr(
            WorkingSet, WorkingSet.indexEqRemoved[WorkingSet.mEqRemoved - 1]);
        WorkingSet.mEqRemoved--;
      }
      if (nonlinEqRemoved) {
        kend = (WorkingSet.sizes[0] + b_TrialState.iNonEq0) - 1;
        i = b_TrialState.mNonlinEq;
        for (int idx{0}; idx < i; idx++) {
          WorkingSet.Wlocalidx[kend + idx] = b_TrialState.iNonEq0 + idx;
        }
      }
      if ((b_TrialState.state <= 0) && (b_TrialState.state != -6)) {
        STEP_TYPE = 2;
      } else {
        for (idxStartIneq = 0; idxStartIneq <= nVar; idxStartIneq++) {
          b_TrialState.delta_x[idxStartIneq] = b_TrialState.xstar[idxStartIneq];
        }
        guard1 = true;
      }
    } break;
    case 2: {
      double constrViolation;
      kend = WorkingSet.nWConstr[0] + WorkingSet.nWConstr[1];
      idxStartIneq = kend + 1;
      idxEndIneq = WorkingSet.nActiveConstr;
      for (int idx{idxStartIneq}; idx <= idxEndIneq; idx++) {
        WorkingSet.isActiveConstr
            [(WorkingSet.isActiveIdx[WorkingSet.Wid[idx - 1] - 1] +
              WorkingSet.Wlocalidx[idx - 1]) -
             2] = false;
      }
      WorkingSet.nWConstr[2] = 0;
      WorkingSet.nWConstr[3] = 0;
      WorkingSet.nWConstr[4] = 0;
      WorkingSet.nActiveConstr = kend;
      r.set_size(b_TrialState.xstar.size(0));
      kend = b_TrialState.xstar.size(0);
      for (i = 0; i < kend; i++) {
        r[i] = b_TrialState.xstar[i];
      }
      idxStartIneq = WorkingSet.sizes[3];
      idxEndIneq = WorkingSet.sizes[4];
      for (int idx{0}; idx < idxStartIneq; idx++) {
        constrViolation = WorkingSet.lb[WorkingSet.indexLB[idx] - 1];
        if (-r[WorkingSet.indexLB[idx] - 1] > constrViolation) {
          if (std::isinf(ub[WorkingSet.indexLB[idx] - 1])) {
            r[WorkingSet.indexLB[idx] - 1] =
                -constrViolation + std::abs(constrViolation);
          } else {
            r[WorkingSet.indexLB[idx] - 1] =
                (WorkingSet.ub[WorkingSet.indexLB[idx] - 1] - constrViolation) /
                2.0;
          }
        }
      }
      for (int idx{0}; idx < idxEndIneq; idx++) {
        constrViolation = WorkingSet.ub[WorkingSet.indexUB[idx] - 1];
        if (r[WorkingSet.indexUB[idx] - 1] > constrViolation) {
          if (std::isinf(lb[WorkingSet.indexUB[idx] - 1])) {
            r[WorkingSet.indexUB[idx] - 1] =
                constrViolation - std::abs(constrViolation);
          } else {
            r[WorkingSet.indexUB[idx] - 1] =
                (constrViolation - WorkingSet.lb[WorkingSet.indexUB[idx] - 1]) /
                2.0;
          }
        }
      }
      b_TrialState.xstar.set_size(r.size(0));
      kend = r.size(0);
      for (i = 0; i < kend; i++) {
        b_TrialState.xstar[i] = r[i];
      }
      step::relaxed(Hessian, b_TrialState.grad, b_TrialState, b_MeritFunction,
                    memspace, WorkingSet, b_QRManager, b_CholManager,
                    QPObjective, qpoptions);
      for (idxStartIneq = 0; idxStartIneq <= nVar; idxStartIneq++) {
        b_TrialState.delta_x[idxStartIneq] = b_TrialState.xstar[idxStartIneq];
      }
      guard1 = true;
    } break;
    default:
      r.set_size(b_TrialState.grad.size(0));
      kend = b_TrialState.grad.size(0);
      for (i = 0; i < kend; i++) {
        r[i] = b_TrialState.grad[i];
      }
      stepSuccess =
          step::soc(Hessian, r, b_TrialState, memspace, WorkingSet, b_QRManager,
                    b_CholManager, QPObjective, qpoptions);
      checkBoundViolation = stepSuccess;
      if (stepSuccess && (b_TrialState.state != -6)) {
        for (int idx{0}; idx <= nVar; idx++) {
          b_TrialState.delta_x[idx] =
              b_TrialState.xstar[idx] + b_TrialState.socDirection[idx];
        }
      }
      guard1 = true;
      break;
    }
    if (guard1) {
      if (b_TrialState.state != -6) {
        exitg1 = 1;
      } else {
        constrViolationEq = 0.0;
        constrViolationIneq = 1.0;
        for (int idx{0}; idx < 113; idx++) {
          constrViolationEq =
              std::fmax(constrViolationEq, std::abs(b_TrialState.grad[idx]));
          constrViolationIneq =
              std::fmax(constrViolationIneq, std::abs(b_TrialState.xstar[idx]));
        }
        constrViolationEq = std::fmax(2.2204460492503131E-16,
                                      constrViolationEq / constrViolationIneq);
        for (idxEndIneq = 0; idxEndIneq < 113; idxEndIneq++) {
          kend = 113 * idxEndIneq;
          for (idxStartIneq = 0; idxStartIneq < idxEndIneq; idxStartIneq++) {
            Hessian[kend + idxStartIneq] = 0.0;
          }
          kend = idxEndIneq + 113 * idxEndIneq;
          Hessian[kend] = constrViolationEq;
          idxStartIneq = 111 - idxEndIneq;
          if (idxStartIneq >= 0) {
            std::memset(
                &Hessian[kend + 1], 0,
                static_cast<unsigned int>(((idxStartIneq + kend) - kend) + 1) *
                    sizeof(double));
          }
        }
      }
    }
  } while (exitg1 == 0);
  if (checkBoundViolation) {
    idxStartIneq = WorkingSet.sizes[3];
    idxEndIneq = WorkingSet.sizes[4];
    r.set_size(b_TrialState.delta_x.size(0));
    kend = b_TrialState.delta_x.size(0);
    for (i = 0; i < kend; i++) {
      r[i] = b_TrialState.delta_x[i];
    }
    for (int idx{0}; idx < idxStartIneq; idx++) {
      constrViolationEq = r[WorkingSet.indexLB[idx] - 1];
      constrViolationIneq =
          (b_TrialState.xstarsqp[WorkingSet.indexLB[idx] - 1] +
           constrViolationEq) -
          lb[WorkingSet.indexLB[idx] - 1];
      if (constrViolationIneq < 0.0) {
        r[WorkingSet.indexLB[idx] - 1] =
            constrViolationEq - constrViolationIneq;
        b_TrialState.xstar[WorkingSet.indexLB[idx] - 1] =
            b_TrialState.xstar[WorkingSet.indexLB[idx] - 1] -
            constrViolationIneq;
      }
    }
    for (int idx{0}; idx < idxEndIneq; idx++) {
      constrViolationEq = r[WorkingSet.indexUB[idx] - 1];
      constrViolationIneq =
          (ub[WorkingSet.indexUB[idx] - 1] -
           b_TrialState.xstarsqp[WorkingSet.indexUB[idx] - 1]) -
          constrViolationEq;
      if (constrViolationIneq < 0.0) {
        r[WorkingSet.indexUB[idx] - 1] =
            constrViolationEq + constrViolationIneq;
        b_TrialState.xstar[WorkingSet.indexUB[idx] - 1] =
            b_TrialState.xstar[WorkingSet.indexUB[idx] - 1] +
            constrViolationIneq;
      }
    }
    b_TrialState.delta_x.set_size(r.size(0));
    kend = r.size(0);
    for (i = 0; i < kend; i++) {
      b_TrialState.delta_x[i] = r[i];
    }
  }
  return stepSuccess;
}

} // namespace fminconsqp
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (step.cpp)
