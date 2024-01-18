//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// test_exit.cpp
//
// Code generation for function 'test_exit'
//

// Include files
#include "test_exit.h"
#include "NMPC_Node_internal_types.h"
#include "computeComplError.h"
#include "computeGradLag.h"
#include "computeLambdaLSQ.h"
#include "computePrimalFeasError.h"
#include "rt_nonfinite.h"
#include "sortLambdaQP.h"
#include "updateWorkingSetForNewQP.h"
#include "coder_array.h"
#include <cmath>
#include <cstring>

// Function Definitions
namespace coder {
namespace optim {
namespace coder {
namespace fminconsqp {
void b_test_exit(b_struct_T &Flags, h_struct_T &memspace,
                 struct_T &b_MeritFunction,
                 const ::coder::array<double, 1U> &fscales_lineq_constraint,
                 const ::coder::array<double, 1U> &fscales_cineq_constraint,
                 i_struct_T &WorkingSet, d_struct_T &b_TrialState,
                 e_struct_T &b_QRManager, const double lb[113],
                 const double ub[113])
{
  ::coder::array<double, 2U> *b_gradLag;
  ::coder::array<double, 1U> *gradLag;
  double optimRelativeFactor;
  double s;
  double smax;
  int idx_max;
  int k;
  int mFixed;
  int mIneq;
  int mLB;
  int mLambda;
  int mLambda_tmp;
  int mUB;
  int nVar_tmp;
  bool dxTooSmall;
  bool exitg1;
  bool isFeasible;
  nVar_tmp = WorkingSet.nVar;
  mFixed = WorkingSet.sizes[0];
  mIneq = WorkingSet.sizes[2];
  mLB = WorkingSet.sizes[3];
  mUB = WorkingSet.sizes[4];
  mLambda_tmp = WorkingSet.sizes[0] + WorkingSet.sizes[1];
  mLambda = (((mLambda_tmp + WorkingSet.sizes[2]) + WorkingSet.sizes[3]) +
             WorkingSet.sizes[4]) -
            1;
  for (k = 0; k <= mLambda; k++) {
    b_TrialState.lambdaStopTest[k] = b_TrialState.lambdasqp[k];
  }
  stopping::computeGradLag(
      b_TrialState.gradLag, WorkingSet.ldA, WorkingSet.nVar, b_TrialState.grad,
      WorkingSet.sizes[2], WorkingSet.Aineq, WorkingSet.sizes[1],
      WorkingSet.Aeq, WorkingSet.indexFixed, WorkingSet.sizes[0],
      WorkingSet.indexLB, WorkingSet.sizes[3], WorkingSet.indexUB,
      WorkingSet.sizes[4], b_TrialState.lambdaStopTest);
  if (WorkingSet.nVar < 1) {
    idx_max = 0;
  } else {
    idx_max = 1;
    if (WorkingSet.nVar > 1) {
      smax = std::abs(b_TrialState.grad[0]);
      for (k = 2; k <= nVar_tmp; k++) {
        s = std::abs(b_TrialState.grad[k - 1]);
        if (s > smax) {
          idx_max = k;
          smax = s;
        }
      }
    }
  }
  optimRelativeFactor =
      std::fmax(1.0, std::abs(b_TrialState.grad[idx_max - 1]));
  if (std::isinf(optimRelativeFactor)) {
    optimRelativeFactor = 1.0;
  }
  b_MeritFunction.nlpPrimalFeasError = stopping::computePrimalFeasError(
      b_TrialState.xstarsqp, WorkingSet.sizes[2] - b_TrialState.mNonlinIneq,
      b_TrialState.mNonlinIneq, b_TrialState.cIneq,
      WorkingSet.sizes[1] - b_TrialState.mNonlinEq, b_TrialState.mNonlinEq,
      b_TrialState.cEq, WorkingSet.indexLB, WorkingSet.sizes[3], lb,
      WorkingSet.indexUB, WorkingSet.sizes[4], ub);
  if (b_TrialState.sqpIterations == 0) {
    b_MeritFunction.feasRelativeFactor =
        std::fmax(1.0, b_MeritFunction.nlpPrimalFeasError);
  }
  isFeasible = (b_MeritFunction.nlpPrimalFeasError <=
                1.0E-6 * b_MeritFunction.feasRelativeFactor);
  gradLag = &b_TrialState.gradLag;
  dxTooSmall = true;
  smax = 0.0;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k <= nVar_tmp - 1)) {
    dxTooSmall = ((!std::isinf((*gradLag)[k])) && (!std::isnan((*gradLag)[k])));
    if (!dxTooSmall) {
      exitg1 = true;
    } else {
      smax = std::fmax(smax, std::abs((*gradLag)[k]));
      k++;
    }
  }
  Flags.gradOK = dxTooSmall;
  b_MeritFunction.nlpDualFeasError = smax;
  if (!Flags.gradOK) {
    Flags.done = true;
    if (isFeasible) {
      b_TrialState.sqpExitFlag = 2;
    } else {
      b_TrialState.sqpExitFlag = -2;
    }
  } else {
    b_MeritFunction.nlpComplError = stopping::computeComplError(
        fscales_lineq_constraint, fscales_cineq_constraint,
        b_TrialState.xstarsqp, WorkingSet.sizes[2], b_TrialState.cIneq,
        WorkingSet.indexLB, WorkingSet.sizes[3], lb, WorkingSet.indexUB,
        WorkingSet.sizes[4], ub, b_TrialState.lambdaStopTest, mLambda_tmp + 1);
    smax = std::fmax(b_MeritFunction.nlpDualFeasError,
                     b_MeritFunction.nlpComplError);
    b_MeritFunction.firstOrderOpt = smax;
    if (b_TrialState.sqpIterations > 1) {
      double d;
      double nlpComplErrorTmp;
      stopping::computeGradLag(
          memspace.workspace_double, WorkingSet.ldA, WorkingSet.nVar,
          b_TrialState.grad, WorkingSet.sizes[2], WorkingSet.Aineq,
          WorkingSet.sizes[1], WorkingSet.Aeq, WorkingSet.indexFixed,
          WorkingSet.sizes[0], WorkingSet.indexLB, WorkingSet.sizes[3],
          WorkingSet.indexUB, WorkingSet.sizes[4],
          b_TrialState.lambdaStopTestPrev);
      b_gradLag = &memspace.workspace_double;
      s = 0.0;
      k = 0;
      exitg1 = false;
      while ((!exitg1) && (k <= nVar_tmp - 1)) {
        dxTooSmall =
            ((!std::isinf((*b_gradLag)[k])) && (!std::isnan((*b_gradLag)[k])));
        if (!dxTooSmall) {
          exitg1 = true;
        } else {
          s = std::fmax(s, std::abs((*b_gradLag)[k]));
          k++;
        }
      }
      nlpComplErrorTmp = stopping::computeComplError(
          fscales_lineq_constraint, fscales_cineq_constraint,
          b_TrialState.xstarsqp, WorkingSet.sizes[2], b_TrialState.cIneq,
          WorkingSet.indexLB, WorkingSet.sizes[3], lb, WorkingSet.indexUB,
          WorkingSet.sizes[4], ub, b_TrialState.lambdaStopTestPrev,
          mLambda_tmp + 1);
      d = std::fmax(s, nlpComplErrorTmp);
      if (d < smax) {
        b_MeritFunction.nlpDualFeasError = s;
        b_MeritFunction.nlpComplError = nlpComplErrorTmp;
        b_MeritFunction.firstOrderOpt = d;
        for (k = 0; k <= mLambda; k++) {
          b_TrialState.lambdaStopTest[k] = b_TrialState.lambdaStopTestPrev[k];
        }
      } else {
        for (k = 0; k <= mLambda; k++) {
          b_TrialState.lambdaStopTestPrev[k] = b_TrialState.lambdaStopTest[k];
        }
      }
    } else {
      for (k = 0; k <= mLambda; k++) {
        b_TrialState.lambdaStopTestPrev[k] = b_TrialState.lambdaStopTest[k];
      }
    }
    if (isFeasible &&
        (b_MeritFunction.nlpDualFeasError <= 1.0E-6 * optimRelativeFactor) &&
        (b_MeritFunction.nlpComplError <= 1.0E-6 * optimRelativeFactor)) {
      Flags.done = true;
      b_TrialState.sqpExitFlag = 1;
    } else {
      Flags.done = false;
      if (isFeasible && (b_TrialState.sqpFval < -1.0E+20)) {
        Flags.done = true;
        b_TrialState.sqpExitFlag = -3;
      } else {
        bool guard1{false};
        guard1 = false;
        if (b_TrialState.sqpIterations > 0) {
          dxTooSmall = true;
          k = 0;
          exitg1 = false;
          while ((!exitg1) && (k <= nVar_tmp - 1)) {
            if (1.0E-6 * std::fmax(1.0, std::abs(b_TrialState.xstarsqp[k])) <=
                std::abs(b_TrialState.delta_x[k])) {
              dxTooSmall = false;
              exitg1 = true;
            } else {
              k++;
            }
          }
          if (dxTooSmall) {
            if (!isFeasible) {
              if (Flags.stepType != 2) {
                Flags.stepType = 2;
                Flags.failedLineSearch = false;
                Flags.stepAccepted = false;
                guard1 = true;
              } else {
                Flags.done = true;
                b_TrialState.sqpExitFlag = -2;
              }
            } else if (WorkingSet.nActiveConstr > 0) {
              if (b_TrialState.mNonlinEq + b_TrialState.mNonlinIneq > 0) {
                internal::updateWorkingSetForNewQP(
                    b_TrialState.xstarsqp, WorkingSet, WorkingSet.sizes[2],
                    b_TrialState.mNonlinIneq, b_TrialState.cIneq,
                    WorkingSet.sizes[1], b_TrialState.mNonlinEq,
                    b_TrialState.cEq, WorkingSet.sizes[3], lb,
                    WorkingSet.sizes[4], ub, WorkingSet.sizes[0]);
              }
              stopping::computeLambdaLSQ(
                  nVar_tmp, WorkingSet.nActiveConstr, b_QRManager,
                  WorkingSet.ATwset, WorkingSet.ldA, b_TrialState.grad,
                  b_TrialState.lambda, memspace.workspace_double);
              idx_max = mFixed + 1;
              for (k = idx_max; k <= mLambda_tmp; k++) {
                b_TrialState.lambda[k - 1] = -b_TrialState.lambda[k - 1];
              }
              qpactiveset::parseoutput::sortLambdaQP(
                  b_TrialState.lambda, WorkingSet.nActiveConstr,
                  WorkingSet.sizes, WorkingSet.isActiveIdx, WorkingSet.Wid,
                  WorkingSet.Wlocalidx, memspace.workspace_double);
              stopping::computeGradLag(
                  memspace.workspace_double, WorkingSet.ldA, nVar_tmp,
                  b_TrialState.grad, mIneq, WorkingSet.Aineq,
                  WorkingSet.sizes[1], WorkingSet.Aeq, WorkingSet.indexFixed,
                  mFixed, WorkingSet.indexLB, mLB, WorkingSet.indexUB, mUB,
                  b_TrialState.lambda);
              b_gradLag = &memspace.workspace_double;
              smax = 0.0;
              k = 0;
              exitg1 = false;
              while ((!exitg1) && (k <= nVar_tmp - 1)) {
                dxTooSmall = ((!std::isinf((*b_gradLag)[k])) &&
                              (!std::isnan((*b_gradLag)[k])));
                if (!dxTooSmall) {
                  exitg1 = true;
                } else {
                  smax = std::fmax(smax, std::abs((*b_gradLag)[k]));
                  k++;
                }
              }
              s = stopping::computeComplError(
                  fscales_lineq_constraint, fscales_cineq_constraint,
                  b_TrialState.xstarsqp, mIneq, b_TrialState.cIneq,
                  WorkingSet.indexLB, mLB, lb, WorkingSet.indexUB, mUB, ub,
                  b_TrialState.lambda, mFixed + 1);
              if ((smax <= 1.0E-6 * optimRelativeFactor) &&
                  (s <= 1.0E-6 * optimRelativeFactor)) {
                b_MeritFunction.nlpDualFeasError = smax;
                b_MeritFunction.nlpComplError = s;
                b_MeritFunction.firstOrderOpt = std::fmax(smax, s);
                for (k = 0; k <= mLambda; k++) {
                  b_TrialState.lambdaStopTest[k] = b_TrialState.lambda[k];
                }
                Flags.done = true;
                b_TrialState.sqpExitFlag = 1;
              } else {
                Flags.done = true;
                b_TrialState.sqpExitFlag = 2;
              }
            } else {
              Flags.done = true;
              b_TrialState.sqpExitFlag = 2;
            }
          } else {
            guard1 = true;
          }
        } else {
          guard1 = true;
        }
        if (guard1) {
          if (b_TrialState.sqpIterations >= 400) {
            Flags.done = true;
            b_TrialState.sqpExitFlag = 0;
          } else if (b_TrialState.FunctionEvaluations >= 11300) {
            Flags.done = true;
            b_TrialState.sqpExitFlag = 0;
          }
        }
      }
    }
  }
}

bool test_exit(h_struct_T &memspace, struct_T &b_MeritFunction,
               const ::coder::array<double, 1U> &fscales_lineq_constraint,
               const ::coder::array<double, 1U> &fscales_cineq_constraint,
               i_struct_T &WorkingSet, d_struct_T &b_TrialState,
               e_struct_T &b_QRManager, const double lb[113],
               const double ub[113], bool &Flags_fevalOK, bool &Flags_done,
               bool &Flags_stepAccepted, bool &Flags_failedLineSearch,
               int &Flags_stepType)
{
  ::coder::array<double, 2U> *b_gradLag;
  ::coder::array<double, 1U> *gradLag;
  double optimRelativeFactor;
  double s;
  double smax;
  int idx_max;
  int k;
  int mFixed;
  int mIneq;
  int mLB;
  int mLambda;
  int mLambda_tmp;
  int mUB;
  int nVar_tmp;
  bool Flags_gradOK;
  bool exitg1;
  bool isFeasible;
  Flags_fevalOK = true;
  Flags_done = false;
  Flags_stepAccepted = false;
  Flags_failedLineSearch = false;
  Flags_stepType = 1;
  nVar_tmp = WorkingSet.nVar;
  mFixed = WorkingSet.sizes[0];
  mIneq = WorkingSet.sizes[2];
  mLB = WorkingSet.sizes[3];
  mUB = WorkingSet.sizes[4];
  mLambda_tmp = WorkingSet.sizes[0] + WorkingSet.sizes[1];
  mLambda = (((mLambda_tmp + WorkingSet.sizes[2]) + WorkingSet.sizes[3]) +
             WorkingSet.sizes[4]) -
            1;
  for (k = 0; k <= mLambda; k++) {
    b_TrialState.lambdaStopTest[k] = b_TrialState.lambdasqp[k];
  }
  stopping::computeGradLag(
      b_TrialState.gradLag, WorkingSet.ldA, WorkingSet.nVar, b_TrialState.grad,
      WorkingSet.sizes[2], WorkingSet.Aineq, WorkingSet.sizes[1],
      WorkingSet.Aeq, WorkingSet.indexFixed, WorkingSet.sizes[0],
      WorkingSet.indexLB, WorkingSet.sizes[3], WorkingSet.indexUB,
      WorkingSet.sizes[4], b_TrialState.lambdaStopTest);
  if (WorkingSet.nVar < 1) {
    idx_max = 0;
  } else {
    idx_max = 1;
    if (WorkingSet.nVar > 1) {
      smax = std::abs(b_TrialState.grad[0]);
      for (k = 2; k <= nVar_tmp; k++) {
        s = std::abs(b_TrialState.grad[k - 1]);
        if (s > smax) {
          idx_max = k;
          smax = s;
        }
      }
    }
  }
  optimRelativeFactor =
      std::fmax(1.0, std::abs(b_TrialState.grad[idx_max - 1]));
  if (std::isinf(optimRelativeFactor)) {
    optimRelativeFactor = 1.0;
  }
  b_MeritFunction.nlpPrimalFeasError = stopping::computePrimalFeasError(
      b_TrialState.xstarsqp, WorkingSet.sizes[2] - b_TrialState.mNonlinIneq,
      b_TrialState.mNonlinIneq, b_TrialState.cIneq,
      WorkingSet.sizes[1] - b_TrialState.mNonlinEq, b_TrialState.mNonlinEq,
      b_TrialState.cEq, WorkingSet.indexLB, WorkingSet.sizes[3], lb,
      WorkingSet.indexUB, WorkingSet.sizes[4], ub);
  if (b_TrialState.sqpIterations == 0) {
    b_MeritFunction.feasRelativeFactor =
        std::fmax(1.0, b_MeritFunction.nlpPrimalFeasError);
  }
  isFeasible = (b_MeritFunction.nlpPrimalFeasError <=
                1.0E-6 * b_MeritFunction.feasRelativeFactor);
  gradLag = &b_TrialState.gradLag;
  Flags_gradOK = true;
  smax = 0.0;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k <= nVar_tmp - 1)) {
    Flags_gradOK =
        ((!std::isinf((*gradLag)[k])) && (!std::isnan((*gradLag)[k])));
    if (!Flags_gradOK) {
      exitg1 = true;
    } else {
      smax = std::fmax(smax, std::abs((*gradLag)[k]));
      k++;
    }
  }
  b_MeritFunction.nlpDualFeasError = smax;
  if (!Flags_gradOK) {
    Flags_done = true;
    if (isFeasible) {
      b_TrialState.sqpExitFlag = 2;
    } else {
      b_TrialState.sqpExitFlag = -2;
    }
  } else {
    bool dxTooSmall;
    b_MeritFunction.nlpComplError = stopping::computeComplError(
        fscales_lineq_constraint, fscales_cineq_constraint,
        b_TrialState.xstarsqp, WorkingSet.sizes[2], b_TrialState.cIneq,
        WorkingSet.indexLB, WorkingSet.sizes[3], lb, WorkingSet.indexUB,
        WorkingSet.sizes[4], ub, b_TrialState.lambdaStopTest, mLambda_tmp + 1);
    smax = std::fmax(b_MeritFunction.nlpDualFeasError,
                     b_MeritFunction.nlpComplError);
    b_MeritFunction.firstOrderOpt = smax;
    if (b_TrialState.sqpIterations > 1) {
      double d;
      double nlpComplErrorTmp;
      stopping::computeGradLag(
          memspace.workspace_double, WorkingSet.ldA, WorkingSet.nVar,
          b_TrialState.grad, WorkingSet.sizes[2], WorkingSet.Aineq,
          WorkingSet.sizes[1], WorkingSet.Aeq, WorkingSet.indexFixed,
          WorkingSet.sizes[0], WorkingSet.indexLB, WorkingSet.sizes[3],
          WorkingSet.indexUB, WorkingSet.sizes[4],
          b_TrialState.lambdaStopTestPrev);
      b_gradLag = &memspace.workspace_double;
      s = 0.0;
      k = 0;
      exitg1 = false;
      while ((!exitg1) && (k <= nVar_tmp - 1)) {
        dxTooSmall =
            ((!std::isinf((*b_gradLag)[k])) && (!std::isnan((*b_gradLag)[k])));
        if (!dxTooSmall) {
          exitg1 = true;
        } else {
          s = std::fmax(s, std::abs((*b_gradLag)[k]));
          k++;
        }
      }
      nlpComplErrorTmp = stopping::computeComplError(
          fscales_lineq_constraint, fscales_cineq_constraint,
          b_TrialState.xstarsqp, WorkingSet.sizes[2], b_TrialState.cIneq,
          WorkingSet.indexLB, WorkingSet.sizes[3], lb, WorkingSet.indexUB,
          WorkingSet.sizes[4], ub, b_TrialState.lambdaStopTestPrev,
          mLambda_tmp + 1);
      d = std::fmax(s, nlpComplErrorTmp);
      if (d < smax) {
        b_MeritFunction.nlpDualFeasError = s;
        b_MeritFunction.nlpComplError = nlpComplErrorTmp;
        b_MeritFunction.firstOrderOpt = d;
        for (k = 0; k <= mLambda; k++) {
          b_TrialState.lambdaStopTest[k] = b_TrialState.lambdaStopTestPrev[k];
        }
      } else {
        for (k = 0; k <= mLambda; k++) {
          b_TrialState.lambdaStopTestPrev[k] = b_TrialState.lambdaStopTest[k];
        }
      }
    } else {
      for (k = 0; k <= mLambda; k++) {
        b_TrialState.lambdaStopTestPrev[k] = b_TrialState.lambdaStopTest[k];
      }
    }
    if (isFeasible &&
        (b_MeritFunction.nlpDualFeasError <= 1.0E-6 * optimRelativeFactor) &&
        (b_MeritFunction.nlpComplError <= 1.0E-6 * optimRelativeFactor)) {
      Flags_done = true;
      b_TrialState.sqpExitFlag = 1;
    } else if (isFeasible && (b_TrialState.sqpFval < -1.0E+20)) {
      Flags_done = true;
      b_TrialState.sqpExitFlag = -3;
    } else {
      bool guard1{false};
      guard1 = false;
      if (b_TrialState.sqpIterations > 0) {
        dxTooSmall = true;
        k = 0;
        exitg1 = false;
        while ((!exitg1) && (k <= nVar_tmp - 1)) {
          if (1.0E-6 * std::fmax(1.0, std::abs(b_TrialState.xstarsqp[k])) <=
              std::abs(b_TrialState.delta_x[k])) {
            dxTooSmall = false;
            exitg1 = true;
          } else {
            k++;
          }
        }
        if (dxTooSmall) {
          if (!isFeasible) {
            Flags_stepType = 2;
            guard1 = true;
          } else if (WorkingSet.nActiveConstr > 0) {
            if (b_TrialState.mNonlinEq + b_TrialState.mNonlinIneq > 0) {
              internal::updateWorkingSetForNewQP(
                  b_TrialState.xstarsqp, WorkingSet, WorkingSet.sizes[2],
                  b_TrialState.mNonlinIneq, b_TrialState.cIneq,
                  WorkingSet.sizes[1], b_TrialState.mNonlinEq, b_TrialState.cEq,
                  WorkingSet.sizes[3], lb, WorkingSet.sizes[4], ub,
                  WorkingSet.sizes[0]);
            }
            stopping::computeLambdaLSQ(
                nVar_tmp, WorkingSet.nActiveConstr, b_QRManager,
                WorkingSet.ATwset, WorkingSet.ldA, b_TrialState.grad,
                b_TrialState.lambda, memspace.workspace_double);
            idx_max = mFixed + 1;
            for (k = idx_max; k <= mLambda_tmp; k++) {
              b_TrialState.lambda[k - 1] = -b_TrialState.lambda[k - 1];
            }
            qpactiveset::parseoutput::sortLambdaQP(
                b_TrialState.lambda, WorkingSet.nActiveConstr, WorkingSet.sizes,
                WorkingSet.isActiveIdx, WorkingSet.Wid, WorkingSet.Wlocalidx,
                memspace.workspace_double);
            stopping::computeGradLag(
                memspace.workspace_double, WorkingSet.ldA, nVar_tmp,
                b_TrialState.grad, mIneq, WorkingSet.Aineq, WorkingSet.sizes[1],
                WorkingSet.Aeq, WorkingSet.indexFixed, mFixed,
                WorkingSet.indexLB, mLB, WorkingSet.indexUB, mUB,
                b_TrialState.lambda);
            b_gradLag = &memspace.workspace_double;
            smax = 0.0;
            k = 0;
            exitg1 = false;
            while ((!exitg1) && (k <= nVar_tmp - 1)) {
              dxTooSmall = ((!std::isinf((*b_gradLag)[k])) &&
                            (!std::isnan((*b_gradLag)[k])));
              if (!dxTooSmall) {
                exitg1 = true;
              } else {
                smax = std::fmax(smax, std::abs((*b_gradLag)[k]));
                k++;
              }
            }
            s = stopping::computeComplError(
                fscales_lineq_constraint, fscales_cineq_constraint,
                b_TrialState.xstarsqp, mIneq, b_TrialState.cIneq,
                WorkingSet.indexLB, mLB, lb, WorkingSet.indexUB, mUB, ub,
                b_TrialState.lambda, mFixed + 1);
            if ((smax <= 1.0E-6 * optimRelativeFactor) &&
                (s <= 1.0E-6 * optimRelativeFactor)) {
              b_MeritFunction.nlpDualFeasError = smax;
              b_MeritFunction.nlpComplError = s;
              b_MeritFunction.firstOrderOpt = std::fmax(smax, s);
              for (k = 0; k <= mLambda; k++) {
                b_TrialState.lambdaStopTest[k] = b_TrialState.lambda[k];
              }
              Flags_done = true;
              b_TrialState.sqpExitFlag = 1;
            } else {
              Flags_done = true;
              b_TrialState.sqpExitFlag = 2;
            }
          } else {
            Flags_done = true;
            b_TrialState.sqpExitFlag = 2;
          }
        } else {
          guard1 = true;
        }
      } else {
        guard1 = true;
      }
      if (guard1) {
        if (b_TrialState.sqpIterations >= 400) {
          Flags_done = true;
          b_TrialState.sqpExitFlag = 0;
        } else if (b_TrialState.FunctionEvaluations >= 11300) {
          Flags_done = true;
          b_TrialState.sqpExitFlag = 0;
        }
      }
    }
  }
  return Flags_gradOK;
}

} // namespace fminconsqp
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (test_exit.cpp)
