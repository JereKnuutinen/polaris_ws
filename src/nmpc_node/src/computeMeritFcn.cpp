//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// computeMeritFcn.cpp
//
// Code generation for function 'computeMeritFcn'
//

// Include files
#include "computeMeritFcn.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <cmath>
#include <cstring>

// Function Definitions
namespace coder {
namespace optim {
namespace coder {
namespace fminconsqp {
namespace MeritFunction {
double computeMeritFcn(double obj_penaltyParam, double fval,
                       const ::coder::array<double, 1U> &Cineq_workspace,
                       int mIneq, const double Ceq_workspace[98], int mEq,
                       bool evalWellDefined)
{
  double val;
  if (evalWellDefined) {
    double constrViolationEq;
    double constrViolationIneq;
    constrViolationEq = 0.0;
    if (mEq >= 1) {
      for (int k{0}; k < mEq; k++) {
        constrViolationEq += std::abs(Ceq_workspace[k]);
      }
    }
    constrViolationIneq = 0.0;
    for (int k{0}; k < mIneq; k++) {
      if (Cineq_workspace[k] > 0.0) {
        constrViolationIneq += Cineq_workspace[k];
      }
    }
    val = fval + obj_penaltyParam * (constrViolationEq + constrViolationIneq);
  } else {
    val = rtInf;
  }
  return val;
}

} // namespace MeritFunction
} // namespace fminconsqp
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (computeMeritFcn.cpp)
