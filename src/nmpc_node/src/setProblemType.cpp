//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// setProblemType.cpp
//
// Code generation for function 'setProblemType'
//

// Include files
#include "setProblemType.h"
#include "NMPC_Node_internal_types.h"
#include "modifyOverheadPhaseOne_.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <cstring>

// Function Definitions
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace WorkingSet {
void setProblemType(i_struct_T &obj, int PROBLEM_TYPE)
{
  switch (PROBLEM_TYPE) {
  case 3: {
    int i;
    obj.nVar = obj.nVarOrig;
    obj.mConstr = obj.mConstrOrig;
    if (obj.nWConstr[4] > 0) {
      i = obj.sizesNormal[4];
      for (int mIneq{0}; mIneq < i; mIneq++) {
        obj.isActiveConstr[(obj.isActiveIdxNormal[4] + mIneq) - 1] =
            obj.isActiveConstr[(obj.isActiveIdx[4] + mIneq) - 1];
      }
    }
    for (i = 0; i < 5; i++) {
      obj.sizes[i] = obj.sizesNormal[i];
    }
    for (i = 0; i < 6; i++) {
      obj.isActiveIdx[i] = obj.isActiveIdxNormal[i];
    }
  } break;
  case 1:
    obj.nVar = obj.nVarOrig + 1;
    obj.mConstr = obj.mConstrOrig + 1;
    for (int i{0}; i < 5; i++) {
      obj.sizes[i] = obj.sizesPhaseOne[i];
    }
    for (int i{0}; i < 6; i++) {
      obj.isActiveIdx[i] = obj.isActiveIdxPhaseOne[i];
    }
    modifyOverheadPhaseOne_(obj);
    break;
  case 2: {
    int i;
    obj.nVar = obj.nVarMax - 1;
    obj.mConstr = obj.mConstrMax - 1;
    for (i = 0; i < 5; i++) {
      obj.sizes[i] = obj.sizesRegularized[i];
    }
    if (obj.probType != 4) {
      int colOffsetATw;
      int colOffsetAineq;
      int i1;
      int mEq;
      int mIneq;
      int offsetEq1_tmp;
      int offsetEq2;
      int offsetIneq_tmp;
      mIneq = obj.sizes[2];
      mEq = obj.sizes[1];
      offsetIneq_tmp = obj.nVarOrig + 1;
      offsetEq1_tmp = obj.nVarOrig + obj.sizes[2];
      offsetEq2 = offsetEq1_tmp + obj.sizes[1];
      i = obj.sizes[0];
      for (int idx_col{0}; idx_col < i; idx_col++) {
        colOffsetATw = obj.ldA * idx_col;
        i1 = obj.nVar;
        for (int idx_row{offsetIneq_tmp}; idx_row <= i1; idx_row++) {
          obj.ATwset[(idx_row + colOffsetATw) - 1] = 0.0;
        }
      }
      for (int idx_col{0}; idx_col < mIneq; idx_col++) {
        colOffsetAineq = obj.ldA * idx_col - 1;
        i = offsetIneq_tmp + idx_col;
        i1 = i - 1;
        for (int idx_row{offsetIneq_tmp}; idx_row <= i1; idx_row++) {
          obj.Aineq[idx_row + colOffsetAineq] = 0.0;
        }
        obj.Aineq[i + colOffsetAineq] = -1.0;
        i++;
        i1 = obj.nVar;
        for (int idx_row{i}; idx_row <= i1; idx_row++) {
          obj.Aineq[idx_row + colOffsetAineq] = 0.0;
        }
      }
      for (int idx_col{0}; idx_col < mEq; idx_col++) {
        colOffsetAineq = obj.ldA * idx_col - 1;
        colOffsetATw = colOffsetAineq + obj.ldA * (obj.isActiveIdx[1] - 1);
        for (int idx_row{offsetIneq_tmp}; idx_row <= offsetEq1_tmp; idx_row++) {
          obj.Aeq[idx_row + colOffsetAineq] = 0.0;
          obj.ATwset[idx_row + colOffsetATw] = 0.0;
        }
        i = offsetEq1_tmp + 1;
        i1 = (offsetEq1_tmp + idx_col) + 1;
        mIneq = i1 - 1;
        for (int idx_row{i}; idx_row <= mIneq; idx_row++) {
          obj.Aeq[idx_row + colOffsetAineq] = 0.0;
          obj.ATwset[idx_row + colOffsetATw] = 0.0;
        }
        obj.Aeq[i1 + colOffsetAineq] = -1.0;
        obj.ATwset[i1 + colOffsetATw] = -1.0;
        i = i1 + 1;
        for (int idx_row{i}; idx_row <= offsetEq2; idx_row++) {
          obj.Aeq[idx_row + colOffsetAineq] = 0.0;
          obj.ATwset[idx_row + colOffsetATw] = 0.0;
        }
        i = offsetEq2 + 1;
        i1 = (offsetEq2 + idx_col) + 1;
        mIneq = i1 - 1;
        for (int idx_row{i}; idx_row <= mIneq; idx_row++) {
          obj.Aeq[idx_row + colOffsetAineq] = 0.0;
          obj.ATwset[idx_row + colOffsetATw] = 0.0;
        }
        obj.Aeq[i1 + colOffsetAineq] = 1.0;
        obj.ATwset[i1 + colOffsetATw] = 1.0;
        i = i1 + 1;
        i1 = obj.nVar;
        for (int idx_row{i}; idx_row <= i1; idx_row++) {
          obj.Aeq[idx_row + colOffsetAineq] = 0.0;
          obj.ATwset[idx_row + colOffsetATw] = 0.0;
        }
      }
      colOffsetAineq = obj.nVarOrig;
      i = obj.sizesNormal[3] + 1;
      i1 = obj.sizesRegularized[3];
      for (mIneq = i; mIneq <= i1; mIneq++) {
        colOffsetAineq++;
        obj.indexLB[mIneq - 1] = colOffsetAineq;
      }
      if (obj.nWConstr[4] > 0) {
        i = obj.sizesRegularized[4];
        for (mIneq = 0; mIneq < i; mIneq++) {
          obj.isActiveConstr[obj.isActiveIdxRegularized[4] + mIneq] =
              obj.isActiveConstr[(obj.isActiveIdx[4] + mIneq) - 1];
        }
      }
      i = obj.isActiveIdx[4];
      i1 = obj.isActiveIdxRegularized[4] - 1;
      for (mIneq = i; mIneq <= i1; mIneq++) {
        obj.isActiveConstr[mIneq - 1] = false;
      }
      i = offsetEq1_tmp + (obj.sizes[1] << 1);
      for (mIneq = offsetIneq_tmp; mIneq <= i; mIneq++) {
        obj.lb[mIneq - 1] = 0.0;
      }
      colOffsetAineq = obj.isActiveIdx[2];
      i = obj.nActiveConstr;
      for (int idx_col{colOffsetAineq}; idx_col <= i; idx_col++) {
        colOffsetATw = obj.ldA * (idx_col - 1) - 1;
        if (obj.Wid[idx_col - 1] == 3) {
          i1 = offsetIneq_tmp + obj.Wlocalidx[idx_col - 1];
          mIneq = i1 - 2;
          for (int idx_row{offsetIneq_tmp}; idx_row <= mIneq; idx_row++) {
            obj.ATwset[idx_row + colOffsetATw] = 0.0;
          }
          obj.ATwset[(i1 + colOffsetATw) - 1] = -1.0;
          mIneq = obj.nVar;
          for (int idx_row{i1}; idx_row <= mIneq; idx_row++) {
            obj.ATwset[idx_row + colOffsetATw] = 0.0;
          }
        } else {
          i1 = obj.nVar;
          for (int idx_row{offsetIneq_tmp}; idx_row <= i1; idx_row++) {
            obj.ATwset[idx_row + colOffsetATw] = 0.0;
          }
        }
      }
    }
    for (i = 0; i < 6; i++) {
      obj.isActiveIdx[i] = obj.isActiveIdxRegularized[i];
    }
  } break;
  default:
    obj.nVar = obj.nVarMax;
    obj.mConstr = obj.mConstrMax;
    for (int i{0}; i < 5; i++) {
      obj.sizes[i] = obj.sizesRegPhaseOne[i];
    }
    for (int i{0}; i < 6; i++) {
      obj.isActiveIdx[i] = obj.isActiveIdxRegPhaseOne[i];
    }
    modifyOverheadPhaseOne_(obj);
    break;
  }
  obj.probType = PROBLEM_TYPE;
}

} // namespace WorkingSet
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (setProblemType.cpp)
