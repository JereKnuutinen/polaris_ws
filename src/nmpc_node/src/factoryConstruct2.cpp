//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// factoryConstruct2.cpp
//
// Code generation for function 'factoryConstruct2'
//

// Include files
#include "factoryConstruct2.h"
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
void factoryConstruct(int mIneqMax, int nVarMax, int mConstrMax,
                      i_struct_T &obj)
{
  obj.mConstr = 0;
  obj.mConstrOrig = 0;
  obj.mConstrMax = mConstrMax;
  obj.nVar = 113;
  obj.nVarOrig = 113;
  obj.nVarMax = nVarMax;
  obj.ldA = nVarMax;
  obj.Aineq.set_size(mIneqMax * nVarMax);
  obj.bineq.set_size(mIneqMax);
  obj.Aeq.set_size(98 * nVarMax);
  obj.lb.set_size(nVarMax);
  obj.ub.set_size(nVarMax);
  obj.indexLB.set_size(nVarMax);
  obj.indexUB.set_size(nVarMax);
  obj.indexFixed.set_size(nVarMax);
  obj.mEqRemoved = 0;
  obj.ATwset.set_size(nVarMax * mConstrMax);
  obj.bwset.set_size(mConstrMax);
  obj.nActiveConstr = 0;
  obj.maxConstrWorkspace.set_size(mConstrMax);
  for (int i{0}; i < 5; i++) {
    obj.sizes[i] = 0;
    obj.sizesNormal[i] = 0;
    obj.sizesPhaseOne[i] = 0;
    obj.sizesRegularized[i] = 0;
    obj.sizesRegPhaseOne[i] = 0;
  }
  for (int i{0}; i < 6; i++) {
    obj.isActiveIdx[i] = 0;
    obj.isActiveIdxNormal[i] = 0;
    obj.isActiveIdxPhaseOne[i] = 0;
    obj.isActiveIdxRegularized[i] = 0;
    obj.isActiveIdxRegPhaseOne[i] = 0;
  }
  obj.isActiveConstr.set_size(mConstrMax);
  obj.Wid.set_size(mConstrMax);
  obj.Wlocalidx.set_size(mConstrMax);
  for (int i{0}; i < 5; i++) {
    obj.nWConstr[i] = 0;
  }
  obj.probType = 3;
  obj.SLACK0 = 1.0E-5;
}

} // namespace WorkingSet
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (factoryConstruct2.cpp)
