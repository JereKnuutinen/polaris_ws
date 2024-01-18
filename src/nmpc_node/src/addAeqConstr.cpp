//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// addAeqConstr.cpp
//
// Code generation for function 'addAeqConstr'
//

// Include files
#include "addAeqConstr.h"
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
void addAeqConstr(i_struct_T &obj, int idx_local)
{
  int totalEq;
  totalEq = obj.nWConstr[0] + obj.nWConstr[1];
  if ((obj.nActiveConstr == totalEq) && (idx_local > obj.nWConstr[1])) {
    int i;
    int i1;
    int iAeq0;
    int iAw0;
    obj.nWConstr[1]++;
    obj.isActiveConstr[(obj.isActiveIdx[1] + idx_local) - 2] = true;
    obj.nActiveConstr++;
    i = obj.nActiveConstr - 1;
    obj.Wid[i] = 2;
    obj.Wlocalidx[i] = idx_local;
    iAeq0 = obj.ldA * (idx_local - 1);
    iAw0 = obj.ldA * i;
    i1 = obj.nVar;
    for (int idx{0}; idx < i1; idx++) {
      obj.ATwset[iAw0 + idx] = obj.Aeq[iAeq0 + idx];
    }
    obj.bwset[i] = obj.beq[idx_local - 1];
  } else {
    int i;
    int i1;
    int iAeq0;
    int iAw0;
    obj.nActiveConstr++;
    i = obj.nActiveConstr - 1;
    obj.Wid[i] = obj.Wid[totalEq];
    obj.Wlocalidx[i] = obj.Wlocalidx[totalEq];
    i1 = obj.nVar;
    for (int idx{0}; idx < i1; idx++) {
      obj.ATwset[idx + obj.ldA * i] = obj.ATwset[idx + obj.ldA * totalEq];
    }
    obj.bwset[i] = obj.bwset[totalEq];
    obj.nWConstr[1]++;
    obj.isActiveConstr[(obj.isActiveIdx[1] + idx_local) - 2] = true;
    obj.Wid[totalEq] = 2;
    obj.Wlocalidx[totalEq] = idx_local;
    iAeq0 = obj.ldA * (idx_local - 1);
    iAw0 = obj.ldA * totalEq;
    for (int idx{0}; idx < i1; idx++) {
      obj.ATwset[iAw0 + idx] = obj.Aeq[iAeq0 + idx];
    }
    obj.bwset[totalEq] = obj.beq[idx_local - 1];
  }
}

} // namespace WorkingSet
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (addAeqConstr.cpp)
