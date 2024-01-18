//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// fmincon.cpp
//
// Code generation for function 'fmincon'
//

// Include files
#include "fmincon.h"
#include "NMPC_Node_internal_types.h"
#include "NMPC_Node_internal_types1.h"
#include "NMPC_Node_internal_types2.h"
#include "anonymous_function.h"
#include "driver1.h"
#include "evalObjAndConstrAndDerivatives.h"
#include "factoryConstruct.h"
#include "factoryConstruct1.h"
#include "factoryConstruct2.h"
#include "initActiveSet.h"
#include "nlmpcmoveCodeGeneration.h"
#include "rt_nonfinite.h"
#include "stickyStruct.h"
#include "updateWorkingSetForNewQP.h"
#include "coder_array.h"
#include <algorithm>
#include <cmath>
#include <cstring>

// Function Definitions
namespace coder {
double fmincon(const anonymous_function &fun, const double x0[113],
               const ::coder::array<double, 2U> &Aineq,
               const ::coder::array<double, 1U> &bineq, const double lb[113],
               const double ub[113], const anonymous_function &nonlcon,
               double x[113], double &exitflag, m_struct_T &output)
{
  static double unusedExpr[12769];
  static double varargout_4[11074];
  internal::i_stickyStruct FcnEvaluator;
  array<double, 2U> varargout_1;
  array<double, 2U> varargout_3;
  array<double, 1U> fscales_cineq_constraint;
  array<double, 1U> fscales_lineq_constraint;
  d_struct_T TrialState;
  e_struct_T QRManager;
  f_struct_T CholManager;
  g_struct_T QPObjective;
  h_struct_T memspace;
  i_struct_T WorkingSet;
  o_struct_T FiniteDifferences;
  struct_T MeritFunction;
  double c;
  double fval;
  int i;
  int loop_ub;
  int mConstrMax;
  int mFixed;
  int mIneq;
  int mLB;
  int mLinIneq;
  int mNonlinIneq;
  int mUB;
  int maxDims;
  nlmpcmoveCodeGeneration_anonFcn2(
      nonlcon.workspace.runtimedata.x, nonlcon.workspace.runtimedata.OutputMin,
      nonlcon.workspace.runtimedata.OutputMax, x0, varargout_1, TrialState.cEq,
      varargout_3, varargout_4);
  mNonlinIneq = varargout_1.size(0) * varargout_1.size(1);
  mIneq = bineq.size(0) + mNonlinIneq;
  mConstrMax = (mIneq + mIneq) + 521;
  maxDims = mIneq + 310;
  if (maxDims < mConstrMax) {
    maxDims = mConstrMax;
  }
  optim::coder::fminconsqp::TrialState::factoryConstruct(
      mIneq + 310, mConstrMax, mIneq, mNonlinIneq, TrialState);
  std::copy(&x0[0], &x0[113], &TrialState.xstarsqp[0]);
  FcnEvaluator.next.next.next.next.next.value = mNonlinIneq;
  FcnEvaluator.next.next.next.next.next.next.next.next.value = fun;
  FcnEvaluator.next.next.next.next.next.next.next.value = nonlcon;
  optim::coder::utils::FiniteDifferences::factoryConstruct(
      fun, nonlcon, mNonlinIneq, lb, ub, FiniteDifferences);
  QRManager.ldq = maxDims;
  QRManager.QR.set_size(maxDims, maxDims);
  QRManager.Q.set_size(maxDims, maxDims);
  loop_ub = maxDims * maxDims;
  for (i = 0; i < loop_ub; i++) {
    QRManager.Q[i] = 0.0;
  }
  QRManager.jpvt.set_size(maxDims);
  for (i = 0; i < maxDims; i++) {
    QRManager.jpvt[i] = 0;
  }
  QRManager.mrows = 0;
  QRManager.ncols = 0;
  QRManager.tau.set_size(maxDims);
  QRManager.minRowCol = 0;
  QRManager.usedPivoting = false;
  CholManager.FMat.set_size(maxDims, maxDims);
  CholManager.ldm = maxDims;
  CholManager.ndims = 0;
  CholManager.info = 0;
  CholManager.scaleFactor = 0.0;
  CholManager.ConvexCheck = true;
  CholManager.regTol_ = rtInf;
  CholManager.workspace_ = rtInf;
  CholManager.workspace2_ = rtInf;
  QPObjective.grad.set_size(mIneq + 310);
  QPObjective.Hx.set_size(mIneq + 309);
  QPObjective.maxVar = mIneq + 310;
  QPObjective.beta = 0.0;
  QPObjective.rho = 0.0;
  QPObjective.prev_objtype = 3;
  QPObjective.prev_nvar = 0;
  QPObjective.prev_hasLinear = false;
  QPObjective.gammaScalar = 0.0;
  QPObjective.hasLinear = true;
  QPObjective.nvar = 113;
  QPObjective.objtype = 3;
  loop_ub = mIneq + 310;
  if (loop_ub < 2) {
    loop_ub = 2;
  }
  memspace.workspace_double.set_size(maxDims, loop_ub);
  memspace.workspace_int.set_size(maxDims);
  memspace.workspace_sort.set_size(maxDims);
  fscales_lineq_constraint.set_size(bineq.size(0));
  loop_ub = bineq.size(0);
  for (i = 0; i < loop_ub; i++) {
    fscales_lineq_constraint[i] = 1.0;
  }
  fscales_cineq_constraint.set_size(mNonlinIneq);
  for (i = 0; i < mNonlinIneq; i++) {
    fscales_cineq_constraint[i] = 1.0;
  }
  optim::coder::qpactiveset::WorkingSet::factoryConstruct(
      mIneq, mIneq + 310, mConstrMax, WorkingSet);
  mLB = 0;
  mUB = 0;
  mFixed = 0;
  for (loop_ub = 0; loop_ub < 113; loop_ub++) {
    bool guard1{false};
    c = lb[loop_ub];
    guard1 = false;
    if ((!std::isinf(c)) && (!std::isnan(c))) {
      if (std::abs(c - ub[loop_ub]) < 1.0E-6) {
        mFixed++;
        WorkingSet.indexFixed[mFixed - 1] = loop_ub + 1;
      } else {
        mLB++;
        WorkingSet.indexLB[mLB - 1] = loop_ub + 1;
        guard1 = true;
      }
    } else {
      guard1 = true;
    }
    if (guard1) {
      c = ub[loop_ub];
      if ((!std::isinf(c)) && (!std::isnan(c))) {
        mUB++;
        WorkingSet.indexUB[mUB - 1] = loop_ub + 1;
      }
    }
  }
  mLinIneq = bineq.size(0);
  mNonlinIneq = mIneq + mLB;
  loop_ub = ((mNonlinIneq + mUB) + mFixed) + 98;
  WorkingSet.mConstr = loop_ub;
  WorkingSet.mConstrOrig = loop_ub;
  WorkingSet.mConstrMax = mConstrMax;
  WorkingSet.sizes[0] = mFixed;
  WorkingSet.sizes[1] = 98;
  WorkingSet.sizes[2] = mIneq;
  WorkingSet.sizes[3] = mLB;
  WorkingSet.sizes[4] = mUB;
  WorkingSet.sizesPhaseOne[0] = mFixed;
  WorkingSet.sizesPhaseOne[1] = 98;
  WorkingSet.sizesPhaseOne[2] = mIneq;
  WorkingSet.sizesPhaseOne[3] = mLB + 1;
  WorkingSet.sizesPhaseOne[4] = mUB;
  WorkingSet.sizesRegularized[0] = mFixed;
  WorkingSet.sizesRegularized[1] = 98;
  WorkingSet.sizesRegularized[2] = mIneq;
  WorkingSet.sizesRegularized[3] = mNonlinIneq + 196;
  WorkingSet.sizesRegularized[4] = mUB;
  WorkingSet.sizesRegPhaseOne[0] = mFixed;
  WorkingSet.sizesRegPhaseOne[1] = 98;
  WorkingSet.sizesRegPhaseOne[2] = mIneq;
  WorkingSet.sizesRegPhaseOne[3] = mNonlinIneq + 197;
  WorkingSet.sizesRegPhaseOne[4] = mUB;
  WorkingSet.isActiveIdxRegPhaseOne[0] = 1;
  WorkingSet.isActiveIdxRegPhaseOne[1] = mFixed;
  WorkingSet.isActiveIdxRegPhaseOne[2] = 98;
  WorkingSet.isActiveIdxRegPhaseOne[3] = mIneq;
  WorkingSet.isActiveIdxRegPhaseOne[4] = mLB;
  WorkingSet.isActiveIdxRegPhaseOne[5] = mUB;
  for (loop_ub = 0; loop_ub < 5; loop_ub++) {
    WorkingSet.sizesNormal[loop_ub] = WorkingSet.sizes[loop_ub];
    WorkingSet.isActiveIdxRegPhaseOne[loop_ub + 1] +=
        WorkingSet.isActiveIdxRegPhaseOne[loop_ub];
  }
  for (i = 0; i < 6; i++) {
    WorkingSet.isActiveIdx[i] = WorkingSet.isActiveIdxRegPhaseOne[i];
    WorkingSet.isActiveIdxNormal[i] = WorkingSet.isActiveIdxRegPhaseOne[i];
  }
  WorkingSet.isActiveIdxRegPhaseOne[0] = 1;
  WorkingSet.isActiveIdxRegPhaseOne[1] = mFixed;
  WorkingSet.isActiveIdxRegPhaseOne[2] = 98;
  WorkingSet.isActiveIdxRegPhaseOne[3] = mIneq;
  WorkingSet.isActiveIdxRegPhaseOne[4] = mLB + 1;
  WorkingSet.isActiveIdxRegPhaseOne[5] = mUB;
  for (loop_ub = 0; loop_ub < 5; loop_ub++) {
    WorkingSet.isActiveIdxRegPhaseOne[loop_ub + 1] +=
        WorkingSet.isActiveIdxRegPhaseOne[loop_ub];
  }
  for (i = 0; i < 6; i++) {
    WorkingSet.isActiveIdxPhaseOne[i] = WorkingSet.isActiveIdxRegPhaseOne[i];
  }
  WorkingSet.isActiveIdxRegPhaseOne[0] = 1;
  WorkingSet.isActiveIdxRegPhaseOne[1] = mFixed;
  WorkingSet.isActiveIdxRegPhaseOne[2] = 98;
  WorkingSet.isActiveIdxRegPhaseOne[3] = mIneq;
  WorkingSet.isActiveIdxRegPhaseOne[4] = mNonlinIneq + 196;
  WorkingSet.isActiveIdxRegPhaseOne[5] = mUB;
  for (loop_ub = 0; loop_ub < 5; loop_ub++) {
    WorkingSet.isActiveIdxRegPhaseOne[loop_ub + 1] +=
        WorkingSet.isActiveIdxRegPhaseOne[loop_ub];
  }
  for (i = 0; i < 6; i++) {
    WorkingSet.isActiveIdxRegularized[i] = WorkingSet.isActiveIdxRegPhaseOne[i];
  }
  WorkingSet.isActiveIdxRegPhaseOne[0] = 1;
  WorkingSet.isActiveIdxRegPhaseOne[1] = mFixed;
  WorkingSet.isActiveIdxRegPhaseOne[2] = 98;
  WorkingSet.isActiveIdxRegPhaseOne[3] = mIneq;
  WorkingSet.isActiveIdxRegPhaseOne[4] = mNonlinIneq + 197;
  WorkingSet.isActiveIdxRegPhaseOne[5] = mUB;
  for (loop_ub = 0; loop_ub < 5; loop_ub++) {
    WorkingSet.isActiveIdxRegPhaseOne[loop_ub + 1] +=
        WorkingSet.isActiveIdxRegPhaseOne[loop_ub];
  }
  if (mIneq > 0) {
    for (loop_ub = 0; loop_ub < mLinIneq; loop_ub++) {
      i = WorkingSet.nVar;
      for (mNonlinIneq = 0; mNonlinIneq < i; mNonlinIneq++) {
        WorkingSet.Aineq[mNonlinIneq + WorkingSet.ldA * loop_ub] =
            Aineq[loop_ub + mLinIneq * mNonlinIneq];
      }
    }
  }
  for (loop_ub = 0; loop_ub < mLB; loop_ub++) {
    TrialState.xstarsqp[WorkingSet.indexLB[loop_ub] - 1] =
        std::fmax(TrialState.xstarsqp[WorkingSet.indexLB[loop_ub] - 1],
                  lb[WorkingSet.indexLB[loop_ub] - 1]);
  }
  for (loop_ub = 0; loop_ub < mUB; loop_ub++) {
    TrialState.xstarsqp[WorkingSet.indexUB[loop_ub] - 1] =
        std::fmin(TrialState.xstarsqp[WorkingSet.indexUB[loop_ub] - 1],
                  ub[WorkingSet.indexUB[loop_ub] - 1]);
  }
  for (loop_ub = 0; loop_ub < mFixed; loop_ub++) {
    TrialState.xstarsqp[WorkingSet.indexFixed[loop_ub] - 1] =
        ub[WorkingSet.indexFixed[loop_ub] - 1];
  }
  TrialState.sqpFval =
      optim::coder::utils::ObjNonlinEvaluator::evalObjAndConstrAndDerivatives(
          FcnEvaluator, TrialState.xstarsqp, TrialState.grad, TrialState.cIneq,
          TrialState.iNonIneq0, TrialState.cEq, TrialState.iNonEq0,
          WorkingSet.Aineq, TrialState.iNonIneq0, WorkingSet.ldA,
          WorkingSet.Aeq, TrialState.iNonEq0, WorkingSet.ldA, loop_ub);
  TrialState.FunctionEvaluations = FiniteDifferences.numEvals + 1;
  mLinIneq = bineq.size(0) - 1;
  maxDims = WorkingSet.ldA;
  if (bineq.size(0) > 0) {
    for (loop_ub = 0; loop_ub <= mLinIneq; loop_ub++) {
      TrialState.cIneq[loop_ub] = -bineq[loop_ub];
    }
    loop_ub = 0;
    i = WorkingSet.ldA * (bineq.size(0) - 1) + 1;
    for (mLinIneq = 1; maxDims < 0 ? mLinIneq >= i : mLinIneq <= i;
         mLinIneq += maxDims) {
      c = 0.0;
      mNonlinIneq = mLinIneq + 112;
      for (mConstrMax = mLinIneq; mConstrMax <= mNonlinIneq; mConstrMax++) {
        c += WorkingSet.Aineq[mConstrMax - 1] *
             TrialState.xstarsqp[mConstrMax - mLinIneq];
      }
      TrialState.cIneq[loop_ub] = TrialState.cIneq[loop_ub] + c;
      loop_ub++;
    }
  }
  optim::coder::fminconsqp::internal::updateWorkingSetForNewQP(
      x0, WorkingSet, mIneq, TrialState.mNonlinIneq, TrialState.cIneq,
      TrialState.mNonlinEq, TrialState.cEq, mLB, lb, mUB, ub, mFixed);
  optim::coder::qpactiveset::WorkingSet::initActiveSet(WorkingSet);
  MeritFunction.initFval = TrialState.sqpFval;
  MeritFunction.penaltyParam = 1.0;
  MeritFunction.threshold = 0.0001;
  MeritFunction.nPenaltyDecreases = 0;
  MeritFunction.linearizedConstrViol = 0.0;
  c = 0.0;
  for (loop_ub = 0; loop_ub < 98; loop_ub++) {
    c += std::abs(TrialState.cEq[loop_ub]);
  }
  MeritFunction.initConstrViolationEq = c;
  c = 0.0;
  for (loop_ub = 0; loop_ub < mIneq; loop_ub++) {
    if (TrialState.cIneq[loop_ub] > 0.0) {
      c += TrialState.cIneq[loop_ub];
    }
  }
  MeritFunction.initConstrViolationIneq = c;
  MeritFunction.phi = 0.0;
  MeritFunction.phiPrimePlus = 0.0;
  MeritFunction.phiFullStep = 0.0;
  MeritFunction.feasRelativeFactor = 0.0;
  MeritFunction.nlpPrimalFeasError = 0.0;
  MeritFunction.nlpDualFeasError = 0.0;
  MeritFunction.nlpComplError = 0.0;
  MeritFunction.firstOrderOpt = 0.0;
  MeritFunction.hasObjective = true;
  optim::coder::fminconsqp::driver(
      bineq, lb, ub, TrialState, MeritFunction, FcnEvaluator, FiniteDifferences,
      memspace, WorkingSet, QRManager, CholManager, QPObjective,
      fscales_lineq_constraint, fscales_cineq_constraint, unusedExpr);
  std::copy(&TrialState.xstarsqp[0], &TrialState.xstarsqp[113], &x[0]);
  fval = TrialState.sqpFval;
  exitflag = TrialState.sqpExitFlag;
  output.constrviolation = MeritFunction.nlpPrimalFeasError;
  return fval;
}

} // namespace coder

// End of code generation (fmincon.cpp)
