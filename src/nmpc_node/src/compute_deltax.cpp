//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// compute_deltax.cpp
//
// Code generation for function 'compute_deltax'
//

// Include files
#include "compute_deltax.h"
#include "NMPC_Node_internal_types.h"
#include "fullColLDL2_.h"
#include "partialColLDL3_.h"
#include "rt_nonfinite.h"
#include "solve.h"
#include "xgemm.h"
#include "xpotrf.h"
#include "coder_array.h"
#include <cmath>
#include <cstring>

// Function Definitions
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
void compute_deltax(const double H[12769], d_struct_T &solution,
                    h_struct_T &memspace, const e_struct_T &qrmanager,
                    f_struct_T &cholmanager, const g_struct_T &objective,
                    bool alwaysPositiveDef)
{
  int mNull_tmp;
  int nVar_tmp;
  nVar_tmp = qrmanager.mrows - 1;
  mNull_tmp = qrmanager.mrows - qrmanager.ncols;
  if (mNull_tmp <= 0) {
    for (int idx{0}; idx <= nVar_tmp; idx++) {
      solution.searchDir[idx] = 0.0;
    }
  } else {
    int idx;
    for (idx = 0; idx <= nVar_tmp; idx++) {
      solution.searchDir[idx] = -objective.grad[idx];
    }
    if (qrmanager.ncols <= 0) {
      switch (objective.objtype) {
      case 5:
        break;
      case 3: {
        double SCALED_REG_PRIMAL;
        int LDimSizeP1;
        int iac;
        int ix;
        int nVars;
        int order;
        if (alwaysPositiveDef) {
          cholmanager.ndims = qrmanager.mrows;
          for (idx = 0; idx <= nVar_tmp; idx++) {
            order = (nVar_tmp + 1) * idx;
            LDimSizeP1 = cholmanager.ldm * idx;
            for (int k{0}; k <= nVar_tmp; k++) {
              cholmanager.FMat[LDimSizeP1 + k] = H[order + k];
            }
          }
          cholmanager.info = internal::lapack::xpotrf(
              qrmanager.mrows, cholmanager.FMat, cholmanager.ldm);
        } else {
          int k;
          ix = qrmanager.mrows;
          SCALED_REG_PRIMAL = 1.4901161193847656E-8 * cholmanager.scaleFactor *
                              static_cast<double>(qrmanager.mrows);
          iac = cholmanager.ldm + 1;
          cholmanager.ndims = qrmanager.mrows;
          for (idx = 0; idx < ix; idx++) {
            order = qrmanager.mrows * idx;
            LDimSizeP1 = cholmanager.ldm * idx;
            for (k = 0; k < ix; k++) {
              cholmanager.FMat[LDimSizeP1 + k] = H[order + k];
            }
          }
          LDimSizeP1 = qrmanager.mrows;
          if ((LDimSizeP1 < 1) || (iac < 1)) {
            order = 0;
          } else {
            order = 1;
            if (LDimSizeP1 > 1) {
              double smax;
              smax = std::abs(cholmanager.FMat[0]);
              for (k = 2; k <= LDimSizeP1; k++) {
                double s;
                s = std::abs(cholmanager.FMat[(k - 1) * iac]);
                if (s > smax) {
                  order = k;
                  smax = s;
                }
              }
            }
          }
          cholmanager.regTol_ = std::fmax(
              std::abs(cholmanager
                           .FMat[(order + cholmanager.ldm * (order - 1)) - 1]) *
                  2.2204460492503131E-16,
              std::abs(SCALED_REG_PRIMAL));
          if ((cholmanager.FMat.size(0) * cholmanager.FMat.size(1) > 16384) &&
              (qrmanager.mrows > 128)) {
            bool exitg1;
            k = 0;
            exitg1 = false;
            while ((!exitg1) && (k < ix)) {
              nVars = iac * k + 1;
              order = ix - k;
              if (k + 48 <= ix) {
                DynamicRegCholManager::partialColLDL3_(
                    cholmanager, nVars, order, SCALED_REG_PRIMAL);
                k += 48;
              } else {
                DynamicRegCholManager::fullColLDL2_(cholmanager, nVars, order,
                                                    SCALED_REG_PRIMAL);
                exitg1 = true;
              }
            }
          } else {
            DynamicRegCholManager::fullColLDL2_(cholmanager, 1, qrmanager.mrows,
                                                SCALED_REG_PRIMAL);
          }
          if (cholmanager.ConvexCheck) {
            idx = 0;
            int exitg2;
            do {
              exitg2 = 0;
              if (idx <= ix - 1) {
                if (cholmanager.FMat[idx + cholmanager.ldm * idx] <= 0.0) {
                  cholmanager.info = -idx - 1;
                  exitg2 = 1;
                } else {
                  idx++;
                }
              } else {
                cholmanager.ConvexCheck = false;
                exitg2 = 1;
              }
            } while (exitg2 == 0);
          }
        }
        if (cholmanager.info != 0) {
          solution.state = -6;
        } else if (alwaysPositiveDef) {
          CholManager::solve(cholmanager, solution.searchDir);
        } else {
          int i;
          LDimSizeP1 = cholmanager.ndims - 2;
          if ((cholmanager.FMat.size(0) != 0) &&
              (cholmanager.FMat.size(1) != 0) &&
              (solution.searchDir.size(0) != 0) && (cholmanager.ndims != 0)) {
            for (order = 0; order <= LDimSizeP1 + 1; order++) {
              nVars = order + order * cholmanager.ldm;
              i = LDimSizeP1 - order;
              for (iac = 0; iac <= i; iac++) {
                ix = (order + iac) + 1;
                solution.searchDir[ix] =
                    solution.searchDir[ix] -
                    solution.searchDir[order] *
                        cholmanager.FMat[(nVars + iac) + 1];
              }
            }
          }
          i = cholmanager.ndims;
          for (idx = 0; idx < i; idx++) {
            solution.searchDir[idx] =
                solution.searchDir[idx] /
                cholmanager.FMat[idx + cholmanager.ldm * idx];
          }
          LDimSizeP1 = cholmanager.ndims;
          if ((cholmanager.FMat.size(0) != 0) &&
              (cholmanager.FMat.size(1) != 0) &&
              (solution.searchDir.size(0) != 0) && (cholmanager.ndims != 0)) {
            for (order = LDimSizeP1; order >= 1; order--) {
              nVars = (order - 1) * cholmanager.ldm;
              SCALED_REG_PRIMAL = solution.searchDir[order - 1];
              i = order + 1;
              for (iac = LDimSizeP1; iac >= i; iac--) {
                SCALED_REG_PRIMAL -= cholmanager.FMat[(nVars + iac) - 1] *
                                     solution.searchDir[iac - 1];
              }
              solution.searchDir[order - 1] = SCALED_REG_PRIMAL;
            }
          }
        }
      } break;
      case 4: {
        if (alwaysPositiveDef) {
          int nVars;
          int order;
          nVars = objective.nvar;
          cholmanager.ndims = objective.nvar;
          for (idx = 0; idx < nVars; idx++) {
            int LDimSizeP1;
            order = nVars * idx;
            LDimSizeP1 = cholmanager.ldm * idx;
            for (int k{0}; k < nVars; k++) {
              cholmanager.FMat[LDimSizeP1 + k] = H[order + k];
            }
          }
          cholmanager.info = internal::lapack::xpotrf(
              objective.nvar, cholmanager.FMat, cholmanager.ldm);
          if (cholmanager.info != 0) {
            solution.state = -6;
          } else {
            double SCALED_REG_PRIMAL;
            int i;
            CholManager::solve(cholmanager, solution.searchDir);
            SCALED_REG_PRIMAL = 1.0 / objective.beta;
            order = objective.nvar + 1;
            i = qrmanager.mrows;
            for (int k{order}; k <= i; k++) {
              solution.searchDir[k - 1] =
                  SCALED_REG_PRIMAL * solution.searchDir[k - 1];
            }
          }
        }
      } break;
      }
    } else {
      int nullStartIdx_tmp;
      nullStartIdx_tmp = qrmanager.ldq * qrmanager.ncols + 1;
      if (objective.objtype == 5) {
        int LDimSizeP1;
        for (idx = 0; idx < mNull_tmp; idx++) {
          memspace.workspace_double[idx] =
              -qrmanager.Q[nVar_tmp + qrmanager.ldq * (qrmanager.ncols + idx)];
        }
        LDimSizeP1 = qrmanager.ldq;
        if (qrmanager.mrows != 0) {
          int i;
          int ix;
          int nVars;
          for (nVars = 0; nVars <= nVar_tmp; nVars++) {
            solution.searchDir[nVars] = 0.0;
          }
          ix = 0;
          i = nullStartIdx_tmp + qrmanager.ldq * (mNull_tmp - 1);
          for (int iac{nullStartIdx_tmp}; LDimSizeP1 < 0 ? iac >= i : iac <= i;
               iac += LDimSizeP1) {
            int order;
            order = iac + nVar_tmp;
            for (idx = iac; idx <= order; idx++) {
              nVars = idx - iac;
              solution.searchDir[nVars] =
                  solution.searchDir[nVars] +
                  qrmanager.Q[idx - 1] * memspace.workspace_double[ix];
            }
            ix++;
          }
        }
      } else {
        double SCALED_REG_PRIMAL;
        int LDimSizeP1;
        int i;
        int k;
        int nVars;
        int order;
        if (objective.objtype == 3) {
          internal::blas::xgemm(qrmanager.mrows, mNull_tmp, qrmanager.mrows, H,
                                qrmanager.mrows, qrmanager.Q, nullStartIdx_tmp,
                                qrmanager.ldq, memspace.workspace_double,
                                memspace.workspace_double.size(0));
          internal::blas::xgemm(mNull_tmp, mNull_tmp, qrmanager.mrows,
                                qrmanager.Q, nullStartIdx_tmp, qrmanager.ldq,
                                memspace.workspace_double,
                                memspace.workspace_double.size(0),
                                cholmanager.FMat, cholmanager.ldm);
        } else if (alwaysPositiveDef) {
          nVars = qrmanager.mrows;
          internal::blas::xgemm(objective.nvar, mNull_tmp, objective.nvar, H,
                                objective.nvar, qrmanager.Q, nullStartIdx_tmp,
                                qrmanager.ldq, memspace.workspace_double,
                                memspace.workspace_double.size(0));
          i = objective.nvar + 1;
          for (LDimSizeP1 = 0; LDimSizeP1 < mNull_tmp; LDimSizeP1++) {
            for (order = i; order <= nVars; order++) {
              memspace
                  .workspace_double[(order + memspace.workspace_double.size(0) *
                                                 LDimSizeP1) -
                                    1] =
                  objective.beta *
                  qrmanager.Q[(order + qrmanager.Q.size(0) *
                                           (LDimSizeP1 + qrmanager.ncols)) -
                              1];
            }
          }
          internal::blas::xgemm(mNull_tmp, mNull_tmp, qrmanager.mrows,
                                qrmanager.Q, nullStartIdx_tmp, qrmanager.ldq,
                                memspace.workspace_double,
                                memspace.workspace_double.size(0),
                                cholmanager.FMat, cholmanager.ldm);
        }
        if (alwaysPositiveDef) {
          cholmanager.ndims = mNull_tmp;
          cholmanager.info = internal::lapack::xpotrf(
              mNull_tmp, cholmanager.FMat, cholmanager.ldm);
        } else {
          SCALED_REG_PRIMAL = 1.4901161193847656E-8 * cholmanager.scaleFactor *
                              static_cast<double>(mNull_tmp);
          LDimSizeP1 = cholmanager.ldm + 1;
          cholmanager.ndims = mNull_tmp;
          nVars = cholmanager.ldm + 1;
          if (nVars < 1) {
            order = 0;
          } else {
            order = 1;
            if (mNull_tmp > 1) {
              double smax;
              smax = std::abs(cholmanager.FMat[0]);
              for (k = 2; k <= mNull_tmp; k++) {
                double s;
                s = std::abs(cholmanager.FMat[(k - 1) * nVars]);
                if (s > smax) {
                  order = k;
                  smax = s;
                }
              }
            }
          }
          cholmanager.regTol_ = std::fmax(
              std::abs(cholmanager
                           .FMat[(order + cholmanager.ldm * (order - 1)) - 1]) *
                  2.2204460492503131E-16,
              std::abs(SCALED_REG_PRIMAL));
          if ((cholmanager.FMat.size(0) * cholmanager.FMat.size(1) > 16384) &&
              (mNull_tmp > 128)) {
            bool exitg1;
            k = 0;
            exitg1 = false;
            while ((!exitg1) && (k < mNull_tmp)) {
              nVars = LDimSizeP1 * k + 1;
              order = mNull_tmp - k;
              if (k + 48 <= mNull_tmp) {
                DynamicRegCholManager::partialColLDL3_(
                    cholmanager, nVars, order, SCALED_REG_PRIMAL);
                k += 48;
              } else {
                DynamicRegCholManager::fullColLDL2_(cholmanager, nVars, order,
                                                    SCALED_REG_PRIMAL);
                exitg1 = true;
              }
            }
          } else {
            DynamicRegCholManager::fullColLDL2_(cholmanager, 1, mNull_tmp,
                                                SCALED_REG_PRIMAL);
          }
          if (cholmanager.ConvexCheck) {
            idx = 0;
            int exitg2;
            do {
              exitg2 = 0;
              if (idx <= mNull_tmp - 1) {
                if (cholmanager.FMat[idx + cholmanager.ldm * idx] <= 0.0) {
                  cholmanager.info = -idx - 1;
                  exitg2 = 1;
                } else {
                  idx++;
                }
              } else {
                cholmanager.ConvexCheck = false;
                exitg2 = 1;
              }
            } while (exitg2 == 0);
          }
        }
        if (cholmanager.info != 0) {
          solution.state = -6;
        } else {
          int ix;
          k = qrmanager.ldq;
          if (qrmanager.mrows != 0) {
            for (nVars = 0; nVars < mNull_tmp; nVars++) {
              memspace.workspace_double[nVars] = 0.0;
            }
            nVars = 0;
            i = nullStartIdx_tmp + qrmanager.ldq * (mNull_tmp - 1);
            for (int iac{nullStartIdx_tmp}; k < 0 ? iac >= i : iac <= i;
                 iac += k) {
              SCALED_REG_PRIMAL = 0.0;
              order = iac + nVar_tmp;
              for (idx = iac; idx <= order; idx++) {
                SCALED_REG_PRIMAL +=
                    qrmanager.Q[idx - 1] * objective.grad[idx - iac];
              }
              memspace.workspace_double[nVars] =
                  memspace.workspace_double[nVars] - SCALED_REG_PRIMAL;
              nVars++;
            }
          }
          if (alwaysPositiveDef) {
            LDimSizeP1 = cholmanager.ndims;
            if ((cholmanager.FMat.size(0) != 0) &&
                (cholmanager.FMat.size(1) != 0) &&
                ((memspace.workspace_double.size(0) != 0) &&
                 (memspace.workspace_double.size(1) != 0)) &&
                (cholmanager.ndims != 0)) {
              for (order = 0; order < LDimSizeP1; order++) {
                nVars = order * cholmanager.ldm;
                SCALED_REG_PRIMAL = memspace.workspace_double[order];
                for (int iac{0}; iac < order; iac++) {
                  SCALED_REG_PRIMAL -= cholmanager.FMat[nVars + iac] *
                                       memspace.workspace_double[iac];
                }
                memspace.workspace_double[order] =
                    SCALED_REG_PRIMAL / cholmanager.FMat[nVars + order];
              }
            }
            LDimSizeP1 = cholmanager.ndims;
            if ((cholmanager.FMat.size(0) != 0) &&
                (cholmanager.FMat.size(1) != 0) &&
                ((memspace.workspace_double.size(0) != 0) &&
                 (memspace.workspace_double.size(1) != 0)) &&
                (cholmanager.ndims != 0)) {
              for (order = LDimSizeP1; order >= 1; order--) {
                nVars = (order + (order - 1) * cholmanager.ldm) - 1;
                memspace.workspace_double[order - 1] =
                    memspace.workspace_double[order - 1] /
                    cholmanager.FMat[nVars];
                for (int iac{0}; iac <= order - 2; iac++) {
                  ix = (order - iac) - 2;
                  memspace.workspace_double[ix] =
                      memspace.workspace_double[ix] -
                      memspace.workspace_double[order - 1] *
                          cholmanager.FMat[(nVars - iac) - 1];
                }
              }
            }
          } else {
            LDimSizeP1 = cholmanager.ndims - 2;
            if ((cholmanager.FMat.size(0) != 0) &&
                (cholmanager.FMat.size(1) != 0) &&
                ((memspace.workspace_double.size(0) != 0) &&
                 (memspace.workspace_double.size(1) != 0)) &&
                (cholmanager.ndims != 0)) {
              for (order = 0; order <= LDimSizeP1 + 1; order++) {
                nVars = order + order * cholmanager.ldm;
                i = LDimSizeP1 - order;
                for (int iac{0}; iac <= i; iac++) {
                  ix = (order + iac) + 1;
                  memspace.workspace_double[ix] =
                      memspace.workspace_double[ix] -
                      memspace.workspace_double[order] *
                          cholmanager.FMat[(nVars + iac) + 1];
                }
              }
            }
            i = cholmanager.ndims;
            for (idx = 0; idx < i; idx++) {
              memspace.workspace_double[idx] =
                  memspace.workspace_double[idx] /
                  cholmanager.FMat[idx + cholmanager.ldm * idx];
            }
            LDimSizeP1 = cholmanager.ndims;
            if ((cholmanager.FMat.size(0) != 0) &&
                (cholmanager.FMat.size(1) != 0) &&
                ((memspace.workspace_double.size(0) != 0) &&
                 (memspace.workspace_double.size(1) != 0)) &&
                (cholmanager.ndims != 0)) {
              for (order = LDimSizeP1; order >= 1; order--) {
                nVars = (order - 1) * cholmanager.ldm;
                SCALED_REG_PRIMAL = memspace.workspace_double[order - 1];
                i = order + 1;
                for (int iac{LDimSizeP1}; iac >= i; iac--) {
                  SCALED_REG_PRIMAL -= cholmanager.FMat[(nVars + iac) - 1] *
                                       memspace.workspace_double[iac - 1];
                }
                memspace.workspace_double[order - 1] = SCALED_REG_PRIMAL;
              }
            }
          }
          if (qrmanager.mrows != 0) {
            for (nVars = 0; nVars <= nVar_tmp; nVars++) {
              solution.searchDir[nVars] = 0.0;
            }
            ix = 0;
            i = nullStartIdx_tmp + qrmanager.ldq * (mNull_tmp - 1);
            for (int iac{nullStartIdx_tmp}; k < 0 ? iac >= i : iac <= i;
                 iac += k) {
              order = iac + nVar_tmp;
              for (idx = iac; idx <= order; idx++) {
                nVars = idx - iac;
                solution.searchDir[nVars] =
                    solution.searchDir[nVars] +
                    qrmanager.Q[idx - 1] * memspace.workspace_double[ix];
              }
              ix++;
            }
          }
        }
      }
    }
  }
}

} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (compute_deltax.cpp)
