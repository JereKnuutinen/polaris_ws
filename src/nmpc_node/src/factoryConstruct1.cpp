//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// factoryConstruct1.cpp
//
// Code generation for function 'factoryConstruct1'
//

// Include files
#include "factoryConstruct1.h"
#include "NMPC_Node_internal_types.h"
#include "anonymous_function.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <cmath>
#include <cstring>

// Function Definitions
namespace coder {
namespace optim {
namespace coder {
namespace utils {
namespace FiniteDifferences {
void factoryConstruct(const anonymous_function &objfun,
                      const anonymous_function &nonlin, int mCineq,
                      const double lb[113], const double ub[113],
                      o_struct_T &obj)
{
  int idx;
  bool b;
  obj.objfun = objfun;
  obj.nonlin = nonlin;
  obj.f_1 = 0.0;
  obj.cIneq_1.set_size(mCineq);
  obj.f_2 = 0.0;
  obj.cIneq_2.set_size(mCineq);
  obj.nVar = 113;
  obj.mIneq = mCineq;
  obj.mEq = 98;
  obj.numEvals = 0;
  obj.SpecifyObjectiveGradient = true;
  obj.SpecifyConstraintGradient = true;
  obj.isEmptyNonlcon = (mCineq + 98 == 0);
  obj.FiniteDifferenceType = 0;
  b = false;
  idx = 0;
  while ((!b) && (idx + 1 <= 113)) {
    obj.hasLB[idx] = ((!std::isinf(lb[idx])) && (!std::isnan(lb[idx])));
    obj.hasUB[idx] = ((!std::isinf(ub[idx])) && (!std::isnan(ub[idx])));
    if (obj.hasLB[idx] || obj.hasUB[idx]) {
      b = true;
    }
    idx++;
  }
  while (idx + 1 <= 113) {
    obj.hasLB[idx] = ((!std::isinf(lb[idx])) && (!std::isnan(lb[idx])));
    obj.hasUB[idx] = ((!std::isinf(ub[idx])) && (!std::isnan(ub[idx])));
    idx++;
  }
  obj.hasBounds = b;
}

} // namespace FiniteDifferences
} // namespace utils
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (factoryConstruct1.cpp)
