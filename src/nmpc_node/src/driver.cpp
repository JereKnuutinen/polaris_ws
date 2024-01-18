//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// driver.cpp
//
// Code generation for function 'driver'
//

// Include files
#include "driver.h"
#include "NMPC_Node_internal_types.h"
#include "PresolveWorkingSet.h"
#include "computeFval.h"
#include "iterate.h"
#include "maxConstraintViolation.h"
#include "removeConstr.h"
#include "rt_nonfinite.h"
#include "setProblemType.h"
#include "coder_array.h"
#include <cstring>

// Function Definitions
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
void driver(const double H[12769], const ::coder::array<double, 1U> &f,
            d_struct_T &solution, h_struct_T &memspace, i_struct_T &workingset,
            e_struct_T &qrmanager, f_struct_T &cholmanager,
            g_struct_T &objective, const n_struct_T &options,
            n_struct_T &runTimeOptions)
{
  int idxStartIneq;
  int mConstr;
  int nVar;
  bool guard1{false};
  solution.iterations = 0;
  runTimeOptions.RemainFeasible = (options.PricingTolerance <= 0.0);
  nVar = workingset.nVar - 1;
  guard1 = false;
  if (workingset.probType == 3) {
    mConstr = workingset.sizes[0];
    for (idxStartIneq = 0; idxStartIneq < mConstr; idxStartIneq++) {
      solution.xstar[workingset.indexFixed[idxStartIneq] - 1] =
          workingset.ub[workingset.indexFixed[idxStartIneq] - 1];
    }
    mConstr = workingset.sizes[3];
    for (idxStartIneq = 0; idxStartIneq < mConstr; idxStartIneq++) {
      if (workingset
              .isActiveConstr[(workingset.isActiveIdx[3] + idxStartIneq) - 1]) {
        solution.xstar[workingset.indexLB[idxStartIneq] - 1] =
            -workingset.lb[workingset.indexLB[idxStartIneq] - 1];
      }
    }
    mConstr = workingset.sizes[4];
    for (idxStartIneq = 0; idxStartIneq < mConstr; idxStartIneq++) {
      if (workingset
              .isActiveConstr[(workingset.isActiveIdx[4] + idxStartIneq) - 1]) {
        solution.xstar[workingset.indexUB[idxStartIneq] - 1] =
            workingset.ub[workingset.indexUB[idxStartIneq] - 1];
      }
    }
    initialize::PresolveWorkingSet(solution, memspace, workingset, qrmanager,
                                   options);
    if (solution.state >= 0) {
      guard1 = true;
    }
  } else {
    solution.state = 82;
    guard1 = true;
  }
  if (guard1) {
    double maxConstr_new;
    solution.iterations = 0;
    solution.maxConstr =
        WorkingSet::maxConstraintViolation(workingset, solution.xstar);
    maxConstr_new =
        options.ConstraintTolerance * runTimeOptions.ConstrRelTolFactor;
    if (solution.maxConstr > maxConstr_new) {
      int PROBTYPE_ORIG;
      int b_nVar;
      int idxEndIneq_tmp_tmp;
      int nVarP1;
      PROBTYPE_ORIG = workingset.probType;
      b_nVar = workingset.nVar;
      nVarP1 = workingset.nVar;
      solution.xstar[workingset.nVar] = solution.maxConstr + 1.0;
      if (workingset.probType == 3) {
        mConstr = 1;
      } else {
        mConstr = 4;
      }
      WorkingSet::setProblemType(workingset, mConstr);
      mConstr = workingset.nWConstr[0] + workingset.nWConstr[1];
      idxStartIneq = mConstr + 1;
      idxEndIneq_tmp_tmp = workingset.nActiveConstr;
      for (int idx_global{idxStartIneq}; idx_global <= idxEndIneq_tmp_tmp;
           idx_global++) {
        workingset.isActiveConstr
            [(workingset.isActiveIdx[workingset.Wid[idx_global - 1] - 1] +
              workingset.Wlocalidx[idx_global - 1]) -
             2] = false;
      }
      workingset.nWConstr[2] = 0;
      workingset.nWConstr[3] = 0;
      workingset.nWConstr[4] = 0;
      workingset.nActiveConstr = mConstr;
      objective.prev_objtype = objective.objtype;
      objective.prev_nvar = objective.nvar;
      objective.prev_hasLinear = objective.hasLinear;
      objective.objtype = 5;
      objective.nvar = nVarP1 + 1;
      objective.gammaScalar = 1.0;
      objective.hasLinear = true;
      solution.fstar = Objective::computeFval(
          objective, memspace.workspace_double, H, f, solution.xstar);
      solution.state = 5;
      iterate(H, f, solution, memspace, workingset, qrmanager, cholmanager,
              objective, options.SolverName, 1.4901161193847657E-10,
              options.ConstraintTolerance, maxConstr_new,
              options.PricingTolerance, options.IterDisplayQP, runTimeOptions);
      if (workingset.isActiveConstr
              [(workingset.isActiveIdx[3] + workingset.sizes[3]) - 2]) {
        bool exitg1;
        idxStartIneq = workingset.sizes[0] + workingset.sizes[1];
        exitg1 = false;
        while ((!exitg1) && (idxStartIneq + 1 <= workingset.nActiveConstr)) {
          if ((workingset.Wid[idxStartIneq] == 4) &&
              (workingset.Wlocalidx[idxStartIneq] == workingset.sizes[3])) {
            WorkingSet::removeConstr(workingset, idxStartIneq + 1);
            exitg1 = true;
          } else {
            idxStartIneq++;
          }
        }
      }
      mConstr = workingset.nActiveConstr;
      idxStartIneq = workingset.sizes[0] + workingset.sizes[1];
      while ((mConstr > idxStartIneq) && (mConstr > b_nVar)) {
        WorkingSet::removeConstr(workingset, mConstr);
        mConstr--;
      }
      solution.maxConstr = solution.xstar[nVarP1];
      WorkingSet::setProblemType(workingset, PROBTYPE_ORIG);
      objective.objtype = objective.prev_objtype;
      objective.nvar = objective.prev_nvar;
      objective.hasLinear = objective.prev_hasLinear;
      if (solution.state != 0) {
        solution.maxConstr =
            WorkingSet::maxConstraintViolation(workingset, solution.xstar);
        if (solution.maxConstr > maxConstr_new) {
          mConstr = workingset.mConstrMax;
          for (idxStartIneq = 0; idxStartIneq < mConstr; idxStartIneq++) {
            solution.lambda[idxStartIneq] = 0.0;
          }
          solution.fstar = Objective::computeFval(
              objective, memspace.workspace_double, H, f, solution.xstar);
          solution.state = -2;
        } else {
          if (solution.maxConstr > 0.0) {
            for (idxStartIneq = 0; idxStartIneq <= nVar; idxStartIneq++) {
              solution.searchDir[idxStartIneq] = solution.xstar[idxStartIneq];
            }
            initialize::PresolveWorkingSet(solution, memspace, workingset,
                                           qrmanager, options);
            maxConstr_new =
                WorkingSet::maxConstraintViolation(workingset, solution.xstar);
            if (maxConstr_new >= solution.maxConstr) {
              solution.maxConstr = maxConstr_new;
              for (idxStartIneq = 0; idxStartIneq <= nVar; idxStartIneq++) {
                solution.xstar[idxStartIneq] = solution.searchDir[idxStartIneq];
              }
            }
          }
          iterate(H, f, solution, memspace, workingset, qrmanager, cholmanager,
                  objective, options.SolverName, options.StepTolerance,
                  options.ConstraintTolerance, options.ObjectiveLimit,
                  options.PricingTolerance, options.IterDisplayQP,
                  runTimeOptions);
        }
      }
    } else {
      iterate(H, f, solution, memspace, workingset, qrmanager, cholmanager,
              objective, options.SolverName, options.StepTolerance,
              options.ConstraintTolerance, options.ObjectiveLimit,
              options.PricingTolerance, options.IterDisplayQP, runTimeOptions);
    }
  }
}

} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (driver.cpp)
