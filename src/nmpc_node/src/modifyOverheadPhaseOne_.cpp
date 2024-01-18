//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// modifyOverheadPhaseOne_.cpp
//
// Code generation for function 'modifyOverheadPhaseOne_'
//

// Include files
#include "modifyOverheadPhaseOne_.h"
#include "NMPC_Node_internal_types.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <cstring>

// Function Definitions
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace WorkingSet {
void modifyOverheadPhaseOne_(i_struct_T &obj)
{
  int i;
  int idxEq;
  i = obj.sizes[0];
  for (int idx{0}; idx < i; idx++) {
    obj.ATwset[(obj.nVar + obj.ldA * idx) - 1] = 0.0;
  }
  i = obj.sizes[1];
  for (int idx{0}; idx < i; idx++) {
    idxEq = (obj.nVar + obj.ldA * idx) - 1;
    obj.Aeq[idxEq] = 0.0;
    obj.ATwset[idxEq + obj.ldA * (obj.isActiveIdx[1] - 1)] = 0.0;
  }
  i = obj.sizes[2];
  for (int idx{0}; idx < i; idx++) {
    obj.Aineq[(obj.nVar + obj.ldA * idx) - 1] = -1.0;
  }
  obj.indexLB[obj.sizes[3] - 1] = obj.nVar;
  obj.lb[obj.nVar - 1] = obj.SLACK0;
  idxEq = obj.isActiveIdx[2];
  i = obj.nActiveConstr;
  for (int idx{idxEq}; idx <= i; idx++) {
    obj.ATwset[(obj.nVar + obj.ldA * (idx - 1)) - 1] = -1.0;
  }
  if (obj.nWConstr[4] > 0) {
    i = obj.sizesNormal[4];
    for (int idx{0}; idx <= i; idx++) {
      obj.isActiveConstr[(obj.isActiveIdx[4] + idx) - 1] = false;
    }
  }
  obj.isActiveConstr[obj.isActiveIdx[4] - 2] = false;
}

} // namespace WorkingSet
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (modifyOverheadPhaseOne_.cpp)
