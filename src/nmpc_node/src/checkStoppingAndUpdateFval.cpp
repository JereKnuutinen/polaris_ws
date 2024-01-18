//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// checkStoppingAndUpdateFval.cpp
//
// Code generation for function 'checkStoppingAndUpdateFval'
//

// Include files
#include "checkStoppingAndUpdateFval.h"
#include "NMPC_Node_internal_types.h"
#include "NMPC_Node_rtwutil.h"
#include "computeFval_ReuseHx.h"
#include "feasibleX0ForWorkingSet.h"
#include "maxConstraintViolation.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <cstring>

// Function Definitions
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace stopping {
void checkStoppingAndUpdateFval(
    int &activeSetChangeID, const ::coder::array<double, 1U> &f,
    d_struct_T &solution, h_struct_T &memspace, const g_struct_T &objective,
    i_struct_T &workingset, e_struct_T &qrmanager,
    double options_ConstraintTolerance, double options_ObjectiveLimit,
    bool options_IterDisplayQP, int runTimeOptions_MaxIterations,
    double runTimeOptions_ConstrRelTolFactor, bool updateFval)
{
  int nVar_tmp;
  solution.iterations++;
  nVar_tmp = objective.nvar - 1;
  if ((solution.iterations >= runTimeOptions_MaxIterations) &&
      ((solution.state != 1) || (objective.objtype == 5))) {
    solution.state = 0;
  }
  if (solution.iterations - div_nzp_s32(solution.iterations) * 50 == 0) {
    double tempMaxConstr;
    solution.maxConstr =
        WorkingSet::maxConstraintViolation(workingset, solution.xstar);
    tempMaxConstr = solution.maxConstr;
    if (objective.objtype == 5) {
      tempMaxConstr = solution.maxConstr - solution.xstar[nVar_tmp];
    }
    if (tempMaxConstr >
        options_ConstraintTolerance * runTimeOptions_ConstrRelTolFactor) {
      bool nonDegenerateWset;
      for (int k{0}; k <= nVar_tmp; k++) {
        solution.searchDir[k] = solution.xstar[k];
      }
      nonDegenerateWset = initialize::feasibleX0ForWorkingSet(
          memspace.workspace_double, solution.searchDir, workingset, qrmanager);
      if ((!nonDegenerateWset) && (solution.state != 0)) {
        solution.state = -2;
      }
      activeSetChangeID = 0;
      tempMaxConstr =
          WorkingSet::maxConstraintViolation(workingset, solution.searchDir);
      if (tempMaxConstr < solution.maxConstr) {
        for (int k{0}; k <= nVar_tmp; k++) {
          solution.xstar[k] = solution.searchDir[k];
        }
        solution.maxConstr = tempMaxConstr;
      }
    }
  }
  if (updateFval &&
      ((options_ObjectiveLimit > rtMinusInf) || options_IterDisplayQP)) {
    solution.fstar = Objective::computeFval_ReuseHx(
        objective, memspace.workspace_double, f, solution.xstar);
    if ((options_ObjectiveLimit > rtMinusInf) &&
        (solution.fstar < options_ObjectiveLimit) &&
        ((solution.state != 0) || (objective.objtype != 5))) {
      solution.state = 2;
    }
  }
}

} // namespace stopping
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (checkStoppingAndUpdateFval.cpp)
