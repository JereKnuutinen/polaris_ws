//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// iterate.cpp
//
// Code generation for function 'iterate'
//

// Include files
#include "iterate.h"
#include "NMPC_Node_internal_types.h"
#include "NMPC_Node_rtwutil.h"
#include "addBoundToActiveSetMatrix_.h"
#include "checkStoppingAndUpdateFval.h"
#include "computeFval_ReuseHx.h"
#include "computeGrad_StoreHx.h"
#include "computeQ_.h"
#include "compute_deltax.h"
#include "deleteColMoveEnd.h"
#include "factorQR.h"
#include "feasibleratiotest.h"
#include "maxConstraintViolation.h"
#include "printInfo.h"
#include "ratiotest.h"
#include "removeConstr.h"
#include "rt_nonfinite.h"
#include "squareQ_appendCol.h"
#include "xnrm2.h"
#include "coder_array.h"
#include <cmath>
#include <cstdio>
#include <cstring>

// Function Definitions
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
void iterate(const double H[12769], const ::coder::array<double, 1U> &f,
             d_struct_T &solution, h_struct_T &memspace, i_struct_T &workingset,
             e_struct_T &qrmanager, f_struct_T &cholmanager,
             g_struct_T &objective, const char options_SolverName[7],
             double options_StepTolerance, double options_ConstraintTolerance,
             double options_ObjectiveLimit, double options_PricingTolerance,
             bool options_IterDisplayQP, const n_struct_T &runTimeOptions)
{
  static const char cv[13]{'N', 'o', 'r', 'm', 'a', 'l', ' ',
                           ' ', ' ', ' ', ' ', ' ', ' '};
  static const char cv1[13]{'P', 'h', 'a', 's', 'e', ' ', 'O',
                            'n', 'e', ' ', ' ', ' ', ' '};
  static const char cv2[13]{'R', 'e', 'g', 'u', 'l', 'a', 'r',
                            'i', 'z', 'e', 'd', ' ', ' '};
  static const char cv3[13]{'P', 'h', 'a', 's', 'e', ' ', 'O',
                            'n', 'e', ' ', 'R', 'e', 'g'};
  static const char b[7]{'f', 'm', 'i', 'n', 'c', 'o', 'n'};
  ::coder::array<double, 2U> *y;
  double alpha;
  double normDelta;
  double tolDelta;
  int TYPE;
  int activeConstrChangedType;
  int activeSetChangeID;
  int globalActiveConstrIdx;
  int i;
  int iAw0;
  int localActiveConstrIdx;
  int nVar_tmp;
  int ret;
  bool newBlocking;
  bool subProblemChanged;
  bool updateFval;
  subProblemChanged = true;
  updateFval = true;
  activeSetChangeID = 0;
  TYPE = objective.objtype;
  tolDelta = 6.7434957617430445E-7;
  nVar_tmp = workingset.nVar;
  activeConstrChangedType = 1;
  localActiveConstrIdx = 0;
  globalActiveConstrIdx = 0;
  Objective::computeGrad_StoreHx(objective, H, f, solution.xstar);
  solution.fstar = Objective::computeFval_ReuseHx(
      objective, memspace.workspace_double, f, solution.xstar);
  if (solution.iterations < runTimeOptions.MaxIterations) {
    solution.state = -5;
  } else {
    solution.state = 0;
  }
  ret = workingset.mConstrMax;
  for (iAw0 = 0; iAw0 < ret; iAw0++) {
    solution.lambda[iAw0] = 0.0;
  }
  if ((solution.iterations == 0) && options_IterDisplayQP) {
    char varargin_4[14];
    char stepType_str[13];
    std::printf("                          First-order                         "
                "                                                 Active\n");
    std::fflush(stdout);
    std::printf(
        " Iter            Fval      Optimality     Feasibility           alpha "
        "   Norm of step           Action     Constraints    Step T"
        "ype\n");
    std::fflush(stdout);
    std::printf("\n");
    std::fflush(stdout);
    switch (workingset.probType) {
    case 1:
      for (i = 0; i < 13; i++) {
        stepType_str[i] = cv1[i];
      }
      break;
    case 2:
      for (i = 0; i < 13; i++) {
        stepType_str[i] = cv2[i];
      }
      break;
    case 4:
      for (i = 0; i < 13; i++) {
        stepType_str[i] = cv3[i];
      }
      break;
    default:
      for (i = 0; i < 13; i++) {
        stepType_str[i] = cv[i];
      }
      break;
    }
    for (i = 0; i < 13; i++) {
      varargin_4[i] = stepType_str[i];
    }
    varargin_4[13] = '\x00';
    std::printf("%5i  %14.6e                                                   "
                "                                         %5i    %s",
                0, solution.fstar, workingset.nActiveConstr, &varargin_4[0]);
    std::fflush(stdout);
    std::printf("\n");
    std::fflush(stdout);
  }
  int exitg1;
  do {
    exitg1 = 0;
    if (solution.state == -5) {
      double smax;
      int b_i;
      int idx;
      bool guard1{false};
      bool guard2{false};
      newBlocking = false;
      guard1 = false;
      guard2 = false;
      if (subProblemChanged) {
        switch (activeSetChangeID) {
        case 1:
          QRManager::squareQ_appendCol(
              qrmanager, workingset.ATwset,
              workingset.ldA * (workingset.nActiveConstr - 1) + 1);
          break;
        case -1:
          QRManager::deleteColMoveEnd(qrmanager, globalActiveConstrIdx);
          break;
        default:
          QRManager::factorQR(qrmanager, workingset.ATwset, nVar_tmp,
                              workingset.nActiveConstr, workingset.ldA);
          QRManager::computeQ_(qrmanager, qrmanager.mrows);
          break;
        }
        ret = std::memcmp(&options_SolverName[0], &b[0], 7);
        compute_deltax(H, solution, memspace, qrmanager, cholmanager, objective,
                       ret == 0);
        if (solution.state != -5) {
          exitg1 = 1;
        } else {
          normDelta = internal::blas::xnrm2(nVar_tmp, solution.searchDir);
          if ((normDelta < options_StepTolerance) ||
              (workingset.nActiveConstr >= nVar_tmp)) {
            guard2 = true;
          } else {
            updateFval = (TYPE == 5);
            if (updateFval || runTimeOptions.RemainFeasible) {
              alpha = feasibleratiotest(
                  solution.xstar, solution.searchDir, memspace.workspace_double,
                  workingset.nVar, workingset.ldA, workingset.Aineq,
                  workingset.bineq, workingset.lb, workingset.ub,
                  workingset.indexLB, workingset.indexUB, workingset.sizes,
                  workingset.isActiveIdx, workingset.isActiveConstr,
                  workingset.nWConstr, updateFval, options_ConstraintTolerance,
                  newBlocking, activeConstrChangedType, localActiveConstrIdx);
            } else {
              alpha = ratiotest(
                  solution.xstar, solution.searchDir, memspace.workspace_double,
                  workingset.nVar, workingset.ldA, workingset.Aineq,
                  workingset.bineq, workingset.lb, workingset.ub,
                  workingset.indexLB, workingset.indexUB, workingset.sizes,
                  workingset.isActiveIdx, workingset.isActiveConstr,
                  workingset.nWConstr, options_ConstraintTolerance, tolDelta,
                  newBlocking, activeConstrChangedType, localActiveConstrIdx);
            }
            if (newBlocking) {
              switch (activeConstrChangedType) {
              case 3:
                workingset.nWConstr[2]++;
                workingset.isActiveConstr[(workingset.isActiveIdx[2] +
                                           localActiveConstrIdx) -
                                          2] = true;
                workingset.nActiveConstr++;
                i = workingset.nActiveConstr - 1;
                workingset.Wid[i] = 3;
                workingset.Wlocalidx[i] = localActiveConstrIdx;
                ret = workingset.ldA * (localActiveConstrIdx - 1);
                iAw0 = workingset.ldA * i;
                b_i = workingset.nVar - 1;
                for (idx = 0; idx <= b_i; idx++) {
                  workingset.ATwset[iAw0 + idx] = workingset.Aineq[ret + idx];
                }
                workingset.bwset[i] =
                    workingset.bineq[localActiveConstrIdx - 1];
                break;
              case 4:
                WorkingSet::addBoundToActiveSetMatrix_(workingset, 4,
                                                       localActiveConstrIdx);
                break;
              default:
                WorkingSet::addBoundToActiveSetMatrix_(workingset, 5,
                                                       localActiveConstrIdx);
                break;
              }
              activeSetChangeID = 1;
            } else {
              if (objective.objtype == 5) {
                if (internal::blas::xnrm2(objective.nvar, solution.searchDir) >
                    100.0 * static_cast<double>(objective.nvar) *
                        1.4901161193847656E-8) {
                  solution.state = 3;
                } else {
                  solution.state = 4;
                }
              }
              subProblemChanged = false;
              if (workingset.nActiveConstr == 0) {
                solution.state = 1;
              }
            }
            if ((nVar_tmp >= 1) && (!(alpha == 0.0))) {
              ret = nVar_tmp - 1;
              for (iAw0 = 0; iAw0 <= ret; iAw0++) {
                solution.xstar[iAw0] =
                    solution.xstar[iAw0] + alpha * solution.searchDir[iAw0];
              }
            }
            Objective::computeGrad_StoreHx(objective, H, f, solution.xstar);
            updateFval = true;
            guard1 = true;
          }
        }
      } else {
        for (iAw0 = 0; iAw0 < nVar_tmp; iAw0++) {
          solution.searchDir[iAw0] = 0.0;
        }
        normDelta = 0.0;
        guard2 = true;
      }
      if (guard2) {
        int nActiveConstr_tmp;
        nActiveConstr_tmp = qrmanager.ncols;
        if (qrmanager.ncols > 0) {
          bool b_guard1{false};
          b_guard1 = false;
          if (objective.objtype != 4) {
            alpha = 100.0 * static_cast<double>(qrmanager.mrows) *
                    2.2204460492503131E-16;
            if ((qrmanager.mrows > 0) && (qrmanager.ncols > 0)) {
              updateFval = true;
            } else {
              updateFval = false;
            }
            if (updateFval) {
              bool b_guard2{false};
              idx = nActiveConstr_tmp;
              b_guard2 = false;
              if (qrmanager.mrows < qrmanager.ncols) {
                ret = qrmanager.mrows + qrmanager.ldq * (qrmanager.ncols - 1);
                while ((idx > qrmanager.mrows) &&
                       (std::abs(qrmanager.QR[ret - 1]) >= alpha)) {
                  idx--;
                  ret -= qrmanager.ldq;
                }
                updateFval = (idx == qrmanager.mrows);
                if (updateFval) {
                  b_guard2 = true;
                }
              } else {
                b_guard2 = true;
              }
              if (b_guard2) {
                ret = idx + qrmanager.ldq * (idx - 1);
                while ((idx >= 1) &&
                       (std::abs(qrmanager.QR[ret - 1]) >= alpha)) {
                  idx--;
                  ret = (ret - qrmanager.ldq) - 1;
                }
                updateFval = (idx == 0);
              }
            }
            if (!updateFval) {
              solution.state = -7;
            } else {
              b_guard1 = true;
            }
          } else {
            b_guard1 = true;
          }
          if (b_guard1) {
            iAw0 = qrmanager.mrows;
            ret = qrmanager.ncols;
            idx = qrmanager.ldq;
            y = &memspace.workspace_double;
            if ((iAw0 != 0) && (ret != 0)) {
              int iy;
              for (iy = 0; iy < ret; iy++) {
                (*y)[iy] = 0.0;
              }
              iy = 0;
              i = idx * (ret - 1) + 1;
              for (int iac{1}; idx < 0 ? iac >= i : iac <= i; iac += idx) {
                alpha = 0.0;
                b_i = (iac + iAw0) - 1;
                for (int ia{iac}; ia <= b_i; ia++) {
                  alpha += qrmanager.Q[ia - 1] * objective.grad[ia - iac];
                }
                (*y)[iy] = (*y)[iy] + alpha;
                iy++;
              }
            }
            if ((qrmanager.QR.size(0) != 0) && (qrmanager.QR.size(1) != 0) &&
                ((memspace.workspace_double.size(0) != 0) &&
                 (memspace.workspace_double.size(1) != 0)) &&
                (qrmanager.ncols != 0)) {
              for (idx = nActiveConstr_tmp; idx >= 1; idx--) {
                ret = (idx + (idx - 1) * qrmanager.ldq) - 1;
                memspace.workspace_double[idx - 1] =
                    memspace.workspace_double[idx - 1] / qrmanager.QR[ret];
                for (b_i = 0; b_i <= idx - 2; b_i++) {
                  iAw0 = (idx - b_i) - 2;
                  memspace.workspace_double[iAw0] =
                      memspace.workspace_double[iAw0] -
                      memspace.workspace_double[idx - 1] *
                          qrmanager.QR[(ret - b_i) - 1];
                }
              }
            }
            for (idx = 0; idx < nActiveConstr_tmp; idx++) {
              solution.lambda[idx] = -memspace.workspace_double[idx];
            }
          }
        }
        if ((solution.state != -7) || (workingset.nActiveConstr > nVar_tmp)) {
          iAw0 = 0;
          alpha = options_PricingTolerance * runTimeOptions.ProbRelTolFactor *
                  static_cast<double>(TYPE != 5);
          i = (workingset.nWConstr[0] + workingset.nWConstr[1]) + 1;
          b_i = workingset.nActiveConstr;
          for (idx = i; idx <= b_i; idx++) {
            smax = solution.lambda[idx - 1];
            if (smax < alpha) {
              alpha = smax;
              iAw0 = idx;
            }
          }
          if (iAw0 == 0) {
            solution.state = 1;
          } else {
            activeSetChangeID = -1;
            globalActiveConstrIdx = iAw0;
            subProblemChanged = true;
            activeConstrChangedType = workingset.Wid[iAw0 - 1];
            localActiveConstrIdx = workingset.Wlocalidx[iAw0 - 1];
            WorkingSet::removeConstr(workingset, iAw0);
            solution.lambda[iAw0 - 1] = 0.0;
          }
        } else {
          iAw0 = workingset.nActiveConstr;
          activeSetChangeID = 0;
          globalActiveConstrIdx = workingset.nActiveConstr;
          subProblemChanged = true;
          ret = workingset.nActiveConstr - 1;
          activeConstrChangedType = workingset.Wid[ret];
          localActiveConstrIdx = workingset.Wlocalidx[ret];
          WorkingSet::removeConstr(workingset, workingset.nActiveConstr);
          solution.lambda[iAw0 - 1] = 0.0;
        }
        updateFval = false;
        alpha = rtNaN;
        guard1 = true;
      }
      if (guard1) {
        stopping::checkStoppingAndUpdateFval(
            activeSetChangeID, f, solution, memspace, objective, workingset,
            qrmanager, options_ConstraintTolerance, options_ObjectiveLimit,
            options_IterDisplayQP, runTimeOptions.MaxIterations,
            runTimeOptions.ConstrRelTolFactor, updateFval);
        if (options_IterDisplayQP) {
          if (solution.iterations - div_nzp_s32(solution.iterations) * 50 ==
              0) {
            std::printf(
                "                          First-order                         "
                "                                                 Active\n");
            std::fflush(stdout);
            std::printf(" Iter            Fval      Optimality     Feasibility "
                        "          alpha    Norm of step           Action     "
                        "Constraints    Step T"
                        "ype\n");
            std::fflush(stdout);
            std::printf("\n");
            std::fflush(stdout);
          } else {
            solution.maxConstr =
                WorkingSet::maxConstraintViolation(workingset, solution.xstar);
          }
          idx = workingset.ldA;
          for (iAw0 = 0; iAw0 < nVar_tmp; iAw0++) {
            memspace.workspace_double[iAw0] = objective.grad[iAw0];
          }
          if ((workingset.nVar != 0) && (workingset.nActiveConstr != 0)) {
            iAw0 = 0;
            i = workingset.ldA * (workingset.nActiveConstr - 1) + 1;
            for (int iac{1}; idx < 0 ? iac >= i : iac <= i; iac += idx) {
              b_i = (iac + nVar_tmp) - 1;
              for (int ia{iac}; ia <= b_i; ia++) {
                ret = ia - iac;
                memspace.workspace_double[ret] =
                    memspace.workspace_double[ret] +
                    workingset.ATwset[ia - 1] * solution.lambda[iAw0];
              }
              iAw0++;
            }
          }
          if (workingset.nVar < 1) {
            ret = 0;
          } else {
            ret = 1;
            if (workingset.nVar > 1) {
              smax = std::abs(memspace.workspace_double[0]);
              for (iAw0 = 2; iAw0 <= nVar_tmp; iAw0++) {
                double s;
                s = std::abs(memspace.workspace_double[iAw0 - 1]);
                if (s > smax) {
                  ret = iAw0;
                  smax = s;
                }
              }
            }
          }
          solution.firstorderopt = std::abs(memspace.workspace_double[ret - 1]);
          display::printInfo(newBlocking, workingset.probType, alpha, normDelta,
                             activeConstrChangedType, localActiveConstrIdx,
                             activeSetChangeID, solution.fstar,
                             solution.firstorderopt, solution.maxConstr,
                             solution.iterations, workingset.indexLB,
                             workingset.indexUB, workingset.nActiveConstr);
        }
      }
    } else {
      if (!updateFval) {
        solution.fstar = Objective::computeFval_ReuseHx(
            objective, memspace.workspace_double, f, solution.xstar);
      }
      exitg1 = 1;
    }
  } while (exitg1 == 0);
}

} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (iterate.cpp)
