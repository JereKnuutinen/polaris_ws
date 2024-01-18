//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// maxConstraintViolation.cpp
//
// Code generation for function 'maxConstraintViolation'
//

// Include files
#include "maxConstraintViolation.h"
#include "NMPC_Node_internal_types.h"
#include "rt_nonfinite.h"
#include "xgemv.h"
#include "coder_array.h"
#include <cmath>
#include <cstring>

// Function Definitions
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace WorkingSet {
double maxConstraintViolation(i_struct_T &obj,
                              const ::coder::array<double, 2U> &x)
{
  double v;
  int mFixed;
  int mIneq;
  int mLB;
  int mUB;
  mLB = obj.sizes[3];
  mUB = obj.sizes[4];
  mFixed = obj.sizes[0];
  if (obj.probType == 2) {
    int mEq;
    int offsetEq2;
    v = 0.0;
    mIneq = obj.sizes[2] - 1;
    mEq = obj.sizes[1] - 1;
    if (obj.Aineq.size(0) != 0) {
      for (offsetEq2 = 0; offsetEq2 <= mIneq; offsetEq2++) {
        obj.maxConstrWorkspace[offsetEq2] = obj.bineq[offsetEq2];
      }
      internal::blas::xgemv(obj.nVarOrig, obj.sizes[2], obj.Aineq, obj.ldA, x,
                            obj.maxConstrWorkspace);
      for (int idx{0}; idx <= mIneq; idx++) {
        obj.maxConstrWorkspace[idx] =
            obj.maxConstrWorkspace[idx] - x[obj.nVarOrig + idx];
        v = std::fmax(v, obj.maxConstrWorkspace[idx]);
      }
    }
    if (obj.Aeq.size(0) != 0) {
      for (offsetEq2 = 0; offsetEq2 <= mEq; offsetEq2++) {
        obj.maxConstrWorkspace[offsetEq2] = obj.beq[offsetEq2];
      }
      internal::blas::xgemv(obj.nVarOrig, obj.sizes[1], obj.Aeq, obj.ldA, x,
                            obj.maxConstrWorkspace);
      mIneq = obj.nVarOrig + obj.sizes[2];
      offsetEq2 = mIneq + obj.sizes[1];
      for (int idx{0}; idx <= mEq; idx++) {
        double d;
        d = (obj.maxConstrWorkspace[idx] - x[mIneq + idx]) + x[offsetEq2 + idx];
        obj.maxConstrWorkspace[idx] = d;
        v = std::fmax(v, std::abs(d));
      }
    }
  } else {
    int mEq;
    v = 0.0;
    mIneq = obj.sizes[2] - 1;
    mEq = obj.sizes[1] - 1;
    if (obj.Aineq.size(0) != 0) {
      for (int offsetEq2{0}; offsetEq2 <= mIneq; offsetEq2++) {
        obj.maxConstrWorkspace[offsetEq2] = obj.bineq[offsetEq2];
      }
      internal::blas::xgemv(obj.nVar, obj.sizes[2], obj.Aineq, obj.ldA, x,
                            obj.maxConstrWorkspace);
      for (int idx{0}; idx <= mIneq; idx++) {
        v = std::fmax(v, obj.maxConstrWorkspace[idx]);
      }
    }
    if (obj.Aeq.size(0) != 0) {
      for (int offsetEq2{0}; offsetEq2 <= mEq; offsetEq2++) {
        obj.maxConstrWorkspace[offsetEq2] = obj.beq[offsetEq2];
      }
      internal::blas::xgemv(obj.nVar, obj.sizes[1], obj.Aeq, obj.ldA, x,
                            obj.maxConstrWorkspace);
      for (int idx{0}; idx <= mEq; idx++) {
        v = std::fmax(v, std::abs(obj.maxConstrWorkspace[idx]));
      }
    }
  }
  if (obj.sizes[3] > 0) {
    for (int idx{0}; idx < mLB; idx++) {
      mIneq = obj.indexLB[idx] - 1;
      v = std::fmax(v, -x[mIneq] - obj.lb[mIneq]);
    }
  }
  if (obj.sizes[4] > 0) {
    for (int idx{0}; idx < mUB; idx++) {
      mIneq = obj.indexUB[idx] - 1;
      v = std::fmax(v, x[mIneq] - obj.ub[mIneq]);
    }
  }
  if (obj.sizes[0] > 0) {
    for (int idx{0}; idx < mFixed; idx++) {
      v = std::fmax(v, std::abs(x[obj.indexFixed[idx] - 1] -
                                obj.ub[obj.indexFixed[idx] - 1]));
    }
  }
  return v;
}

double maxConstraintViolation(i_struct_T &obj,
                              const ::coder::array<double, 1U> &x)
{
  double v;
  int mFixed;
  int mIneq;
  int mLB;
  int mUB;
  mLB = obj.sizes[3];
  mUB = obj.sizes[4];
  mFixed = obj.sizes[0];
  if (obj.probType == 2) {
    int mEq;
    int offsetEq2;
    v = 0.0;
    mIneq = obj.sizes[2] - 1;
    mEq = obj.sizes[1] - 1;
    if (obj.Aineq.size(0) != 0) {
      for (offsetEq2 = 0; offsetEq2 <= mIneq; offsetEq2++) {
        obj.maxConstrWorkspace[offsetEq2] = obj.bineq[offsetEq2];
      }
      internal::blas::xgemv(obj.nVarOrig, obj.sizes[2], obj.Aineq, obj.ldA, x,
                            obj.maxConstrWorkspace);
      for (int idx{0}; idx <= mIneq; idx++) {
        obj.maxConstrWorkspace[idx] =
            obj.maxConstrWorkspace[idx] - x[obj.nVarOrig + idx];
        v = std::fmax(v, obj.maxConstrWorkspace[idx]);
      }
    }
    if (obj.Aeq.size(0) != 0) {
      for (offsetEq2 = 0; offsetEq2 <= mEq; offsetEq2++) {
        obj.maxConstrWorkspace[offsetEq2] = obj.beq[offsetEq2];
      }
      internal::blas::xgemv(obj.nVarOrig, obj.sizes[1], obj.Aeq, obj.ldA, x,
                            obj.maxConstrWorkspace);
      mIneq = obj.nVarOrig + obj.sizes[2];
      offsetEq2 = mIneq + obj.sizes[1];
      for (int idx{0}; idx <= mEq; idx++) {
        double d;
        d = (obj.maxConstrWorkspace[idx] - x[mIneq + idx]) + x[offsetEq2 + idx];
        obj.maxConstrWorkspace[idx] = d;
        v = std::fmax(v, std::abs(d));
      }
    }
  } else {
    int mEq;
    v = 0.0;
    mIneq = obj.sizes[2] - 1;
    mEq = obj.sizes[1] - 1;
    if (obj.Aineq.size(0) != 0) {
      for (int offsetEq2{0}; offsetEq2 <= mIneq; offsetEq2++) {
        obj.maxConstrWorkspace[offsetEq2] = obj.bineq[offsetEq2];
      }
      internal::blas::xgemv(obj.nVar, obj.sizes[2], obj.Aineq, obj.ldA, x,
                            obj.maxConstrWorkspace);
      for (int idx{0}; idx <= mIneq; idx++) {
        v = std::fmax(v, obj.maxConstrWorkspace[idx]);
      }
    }
    if (obj.Aeq.size(0) != 0) {
      for (int offsetEq2{0}; offsetEq2 <= mEq; offsetEq2++) {
        obj.maxConstrWorkspace[offsetEq2] = obj.beq[offsetEq2];
      }
      internal::blas::xgemv(obj.nVar, obj.sizes[1], obj.Aeq, obj.ldA, x,
                            obj.maxConstrWorkspace);
      for (int idx{0}; idx <= mEq; idx++) {
        v = std::fmax(v, std::abs(obj.maxConstrWorkspace[idx]));
      }
    }
  }
  if (obj.sizes[3] > 0) {
    for (int idx{0}; idx < mLB; idx++) {
      mIneq = obj.indexLB[idx] - 1;
      v = std::fmax(v, -x[mIneq] - obj.lb[mIneq]);
    }
  }
  if (obj.sizes[4] > 0) {
    for (int idx{0}; idx < mUB; idx++) {
      mIneq = obj.indexUB[idx] - 1;
      v = std::fmax(v, x[mIneq] - obj.ub[mIneq]);
    }
  }
  if (obj.sizes[0] > 0) {
    for (int idx{0}; idx < mFixed; idx++) {
      v = std::fmax(v, std::abs(x[obj.indexFixed[idx] - 1] -
                                obj.ub[obj.indexFixed[idx] - 1]));
    }
  }
  return v;
}

} // namespace WorkingSet
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (maxConstraintViolation.cpp)
