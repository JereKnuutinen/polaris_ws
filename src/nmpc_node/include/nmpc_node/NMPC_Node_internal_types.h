//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// NMPC_Node_internal_types.h
//
// Code generation for function 'NMPC_Node'
//

#ifndef NMPC_NODE_INTERNAL_TYPES_H
#define NMPC_NODE_INTERNAL_TYPES_H

// Include files
#include "NMPC_Node_types.h"
#include "anonymous_function.h"
#include "rtwtypes.h"
#include "coder_array.h"

// Type Definitions
struct struct_T {
  double penaltyParam;
  double threshold;
  int nPenaltyDecreases;
  double linearizedConstrViol;
  double initFval;
  double initConstrViolationEq;
  double initConstrViolationIneq;
  double phi;
  double phiPrimePlus;
  double phiFullStep;
  double feasRelativeFactor;
  double nlpPrimalFeasError;
  double nlpDualFeasError;
  double nlpComplError;
  double firstOrderOpt;
  bool hasObjective;
};

struct b_struct_T {
  bool gradOK;
  bool fevalOK;
  bool done;
  bool stepAccepted;
  bool failedLineSearch;
  int stepType;
};

struct d_struct_T {
  int nVarMax;
  int mNonlinIneq;
  int mNonlinEq;
  int mIneq;
  int mEq;
  int iNonIneq0;
  int iNonEq0;
  double sqpFval;
  double sqpFval_old;
  double xstarsqp[113];
  double xstarsqp_old[113];
  coder::array<double, 1U> cIneq;
  coder::array<double, 1U> cIneq_old;
  double cEq[98];
  double cEq_old[98];
  coder::array<double, 1U> grad;
  coder::array<double, 1U> grad_old;
  int FunctionEvaluations;
  int sqpIterations;
  int sqpExitFlag;
  coder::array<double, 1U> lambdasqp;
  coder::array<double, 1U> lambdaStopTest;
  coder::array<double, 1U> lambdaStopTestPrev;
  double steplength;
  coder::array<double, 1U> delta_x;
  coder::array<double, 1U> socDirection;
  coder::array<int, 1U> workingset_old;
  coder::array<double, 2U> JacCineqTrans_old;
  coder::array<double, 2U> JacCeqTrans_old;
  coder::array<double, 1U> gradLag;
  coder::array<double, 1U> delta_gradLag;
  coder::array<double, 1U> xstar;
  double fstar;
  double firstorderopt;
  coder::array<double, 1U> lambda;
  int state;
  double maxConstr;
  int iterations;
  coder::array<double, 1U> searchDir;
};

struct e_struct_T {
  int ldq;
  coder::array<double, 2U> QR;
  coder::array<double, 2U> Q;
  coder::array<int, 1U> jpvt;
  int mrows;
  int ncols;
  coder::array<double, 1U> tau;
  int minRowCol;
  bool usedPivoting;
};

struct f_struct_T {
  coder::array<double, 2U> FMat;
  int ldm;
  int ndims;
  int info;
  double scaleFactor;
  bool ConvexCheck;
  double regTol_;
  double workspace_;
  double workspace2_;
};

struct g_struct_T {
  coder::array<double, 1U> grad;
  coder::array<double, 1U> Hx;
  bool hasLinear;
  int nvar;
  int maxVar;
  double beta;
  double rho;
  int objtype;
  int prev_objtype;
  int prev_nvar;
  bool prev_hasLinear;
  double gammaScalar;
};

struct h_struct_T {
  coder::array<double, 2U> workspace_double;
  coder::array<int, 1U> workspace_int;
  coder::array<int, 1U> workspace_sort;
};

struct i_struct_T {
  int mConstr;
  int mConstrOrig;
  int mConstrMax;
  int nVar;
  int nVarOrig;
  int nVarMax;
  int ldA;
  coder::array<double, 1U> Aineq;
  coder::array<double, 1U> bineq;
  coder::array<double, 1U> Aeq;
  double beq[98];
  coder::array<double, 1U> lb;
  coder::array<double, 1U> ub;
  coder::array<int, 1U> indexLB;
  coder::array<int, 1U> indexUB;
  coder::array<int, 1U> indexFixed;
  int mEqRemoved;
  int indexEqRemoved[98];
  coder::array<double, 1U> ATwset;
  coder::array<double, 1U> bwset;
  int nActiveConstr;
  coder::array<double, 1U> maxConstrWorkspace;
  int sizes[5];
  int sizesNormal[5];
  int sizesPhaseOne[5];
  int sizesRegularized[5];
  int sizesRegPhaseOne[5];
  int isActiveIdx[6];
  int isActiveIdxNormal[6];
  int isActiveIdxPhaseOne[6];
  int isActiveIdxRegularized[6];
  int isActiveIdxRegPhaseOne[6];
  coder::array<bool, 1U> isActiveConstr;
  coder::array<int, 1U> Wid;
  coder::array<int, 1U> Wlocalidx;
  int nWConstr[5];
  int probType;
  double SLACK0;
};

struct m_struct_T {
  double constrviolation;
};

struct n_struct_T {
  char SolverName[7];
  int MaxIterations;
  double StepTolerance;
  double ConstraintTolerance;
  double ObjectiveLimit;
  double PricingTolerance;
  double ConstrRelTolFactor;
  double ProbRelTolFactor;
  bool RemainFeasible;
  bool IterDisplayQP;
};

struct o_struct_T {
  coder::anonymous_function objfun;
  coder::anonymous_function nonlin;
  double f_1;
  coder::array<double, 1U> cIneq_1;
  double cEq_1[98];
  double f_2;
  coder::array<double, 1U> cIneq_2;
  double cEq_2[98];
  int nVar;
  int mIneq;
  int mEq;
  int numEvals;
  bool SpecifyObjectiveGradient;
  bool SpecifyConstraintGradient;
  bool isEmptyNonlcon;
  bool hasLB[113];
  bool hasUB[113];
  bool hasBounds;
  int FiniteDifferenceType;
};

#endif
// End of code generation (NMPC_Node_internal_types.h)
