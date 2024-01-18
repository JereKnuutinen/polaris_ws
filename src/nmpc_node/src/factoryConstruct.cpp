//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// factoryConstruct.cpp
//
// Code generation for function 'factoryConstruct'
//

// Include files
#include "factoryConstruct.h"
#include "NMPC_Node_internal_types.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <cstring>

// Function Definitions
namespace coder {
namespace optim {
namespace coder {
namespace fminconsqp {
namespace TrialState {
void factoryConstruct(int nVarMax, int mConstrMax, int mIneq, int mNonlinIneq,
                      d_struct_T &obj)
{
  obj.nVarMax = nVarMax;
  obj.mNonlinIneq = mNonlinIneq;
  obj.mNonlinEq = 98;
  obj.mIneq = mIneq;
  obj.mEq = 98;
  obj.iNonIneq0 = (mIneq - mNonlinIneq) + 1;
  obj.iNonEq0 = 1;
  obj.sqpFval = 0.0;
  obj.sqpFval_old = 0.0;
  obj.cIneq.set_size(mIneq);
  obj.cIneq_old.set_size(mIneq);
  obj.grad.set_size(nVarMax);
  obj.grad_old.set_size(nVarMax);
  obj.FunctionEvaluations = 0;
  obj.sqpIterations = 0;
  obj.sqpExitFlag = 0;
  obj.lambdasqp.set_size(mConstrMax);
  for (int i{0}; i < mConstrMax; i++) {
    obj.lambdasqp[i] = 0.0;
  }
  obj.lambdaStopTest.set_size(mConstrMax);
  obj.lambdaStopTestPrev.set_size(mConstrMax);
  obj.steplength = 1.0;
  obj.delta_x.set_size(nVarMax);
  for (int i{0}; i < nVarMax; i++) {
    obj.delta_x[i] = 0.0;
  }
  obj.socDirection.set_size(nVarMax);
  obj.workingset_old.set_size(mConstrMax);
  if (mNonlinIneq > 0) {
    obj.JacCineqTrans_old.set_size(nVarMax, mNonlinIneq);
  } else {
    obj.JacCineqTrans_old.set_size(0, 0);
  }
  obj.JacCeqTrans_old.set_size(nVarMax, 98);
  obj.gradLag.set_size(nVarMax);
  obj.delta_gradLag.set_size(nVarMax);
  obj.xstar.set_size(nVarMax);
  obj.fstar = 0.0;
  obj.firstorderopt = 0.0;
  obj.lambda.set_size(mConstrMax);
  for (int i{0}; i < mConstrMax; i++) {
    obj.lambda[i] = 0.0;
  }
  obj.state = 0;
  obj.maxConstr = 0.0;
  obj.iterations = 0;
  obj.searchDir.set_size(nVarMax);
}

} // namespace TrialState
} // namespace fminconsqp
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (factoryConstruct.cpp)
