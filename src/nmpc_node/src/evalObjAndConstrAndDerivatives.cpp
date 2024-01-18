//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// evalObjAndConstrAndDerivatives.cpp
//
// Code generation for function 'evalObjAndConstrAndDerivatives'
//

// Include files
#include "evalObjAndConstrAndDerivatives.h"
#include "NMPC_Node_data.h"
#include "NMPC_Node_internal_types1.h"
#include "NMPC_Node_internal_types2.h"
#include "anonymous_function.h"
#include "checkVectorNonFinite.h"
#include "custom_cost_dynamic.h"
#include "nlmpcmoveCodeGeneration.h"
#include "rt_nonfinite.h"
#include "stickyStruct.h"
#include "coder_array.h"
#include <algorithm>
#include <cmath>
#include <cstring>

// Function Definitions
namespace coder {
namespace optim {
namespace coder {
namespace utils {
namespace ObjNonlinEvaluator {
double evalObjAndConstrAndDerivatives(
    const ::coder::internal::i_stickyStruct &obj, const double x[113],
    ::coder::array<double, 1U> &grad_workspace,
    ::coder::array<double, 1U> &Cineq_workspace, int ineq0,
    double Ceq_workspace[98], int eq0,
    ::coder::array<double, 1U> &JacIneqTrans_workspace, int iJI_col, int ldJI,
    ::coder::array<double, 1U> &JacEqTrans_workspace, int iJE_col, int ldJE,
    int &status)
{
  static double varargout_4[11074];
  array<double, 2U> varargout_1;
  array<double, 2U> varargout_3;
  double varargout_2[113];
  double X[112];
  double xa[112];
  double b_x[98];
  double U[16];
  double Umv[16];
  double Gfuu[14];
  double a[14];
  double d;
  double du;
  double dx;
  double f;
  double fval;
  int col;
  int col_end;
  int idx_current;
  bool allFinite;
  std::memset(&X[0], 0, 112U * sizeof(double));
  std::memset(&Umv[0], 0, 16U * sizeof(double));
  for (idx_current = 0; idx_current < 14; idx_current++) {
    d = 0.0;
    for (col = 0; col < 14; col++) {
      d += static_cast<double>(iv[idx_current + 14 * col]) * x[col + 98];
    }
    a[idx_current] = d;
  }
  for (idx_current = 0; idx_current < 2; idx_current++) {
    for (col = 0; col < 7; col++) {
      Umv[col + (idx_current << 3)] = a[idx_current + (col << 1)];
    }
  }
  std::copy(&x[0], &x[98], &b_x[0]);
  for (idx_current = 0; idx_current < 14; idx_current++) {
    for (col = 0; col < 7; col++) {
      X[(col + (idx_current << 3)) + 1] = b_x[idx_current + 14 * col];
    }
    X[idx_current << 3] = obj.next.next.next.next.next.next.next.next.value
                              .workspace.runtimedata.x[idx_current];
  }
  for (col_end = 0; col_end < 2; col_end++) {
    idx_current = col_end << 3;
    Umv[idx_current + 7] = Umv[idx_current + 6];
    std::copy(
        &Umv[idx_current],
        &Umv[static_cast<int>(static_cast<unsigned int>(idx_current) + 8U)],
        &U[idx_current]);
  }
  fval = custom_cost_dynamic(X, U,
                             obj.next.next.next.next.next.next.next.next.value
                                 .workspace.userdata.References);
  std::memset(&Gfuu[0], 0, 14U * sizeof(double));
  for (col = 0; col < 112; col++) {
    d = std::abs(X[col]);
    xa[col] = d;
    if (d < 1.0) {
      xa[col] = 1.0;
    }
  }
  for (col_end = 0; col_end < 7; col_end++) {
    for (col = 0; col < 14; col++) {
      dx = 1.0E-6 * xa[col];
      idx_current = (col_end + (col << 3)) + 1;
      X[idx_current] += dx;
      f = custom_cost_dynamic(X, U,
                              obj.next.next.next.next.next.next.next.next.value
                                  .workspace.userdata.References);
      X[idx_current] -= dx;
      b_x[col + 14 * col_end] = (f - fval) / dx;
    }
  }
  for (col = 0; col < 16; col++) {
    d = std::abs(U[col]);
    Umv[col] = d;
    if (d < 1.0) {
      Umv[col] = 1.0;
    }
  }
  d = Umv[0];
  dx = Umv[1];
  for (col_end = 0; col_end < 6; col_end++) {
    du = 1.0E-6 * d;
    U[col_end] += du;
    f = custom_cost_dynamic(X, U,
                            obj.next.next.next.next.next.next.next.next.value
                                .workspace.userdata.References);
    U[col_end] -= du;
    idx_current = col_end << 1;
    Gfuu[idx_current] = (f - fval) / du;
    du = 1.0E-6 * dx;
    U[col_end + 8] += du;
    f = custom_cost_dynamic(X, U,
                            obj.next.next.next.next.next.next.next.next.value
                                .workspace.userdata.References);
    U[col_end + 8] -= du;
    Gfuu[idx_current + 1] = (f - fval) / du;
  }
  du = 1.0E-6 * Umv[0];
  U[6] += du;
  U[7] += du;
  f = custom_cost_dynamic(X, U,
                          obj.next.next.next.next.next.next.next.next.value
                              .workspace.userdata.References);
  U[6] -= du;
  U[7] -= du;
  Gfuu[12] = (f - fval) / du;
  du = 1.0E-6 * Umv[1];
  U[14] += du;
  U[15] += du;
  f = custom_cost_dynamic(X, U,
                          obj.next.next.next.next.next.next.next.next.value
                              .workspace.userdata.References);
  U[14] -= du;
  U[15] -= du;
  Gfuu[13] = (f - fval) / du;
  dx = custom_cost_dynamic(X, U,
                           obj.next.next.next.next.next.next.next.next.value
                               .workspace.userdata.References);
  du = custom_cost_dynamic(X, U,
                           obj.next.next.next.next.next.next.next.next.value
                               .workspace.userdata.References);
  for (idx_current = 0; idx_current < 14; idx_current++) {
    d = 0.0;
    for (col = 0; col < 14; col++) {
      d += static_cast<double>(iv[idx_current + 14 * col]) * Gfuu[col];
    }
    a[idx_current] = d;
  }
  std::copy(&b_x[0], &b_x[98], &varargout_2[0]);
  std::copy(&a[0], &a[14], &varargout_2[98]);
  varargout_2[112] =
      (dx - du) / (2.0 * (std::fmax(1.0E-6, std::abs(x[112])) * 1.0E-6));
  for (col = 0; col < 113; col++) {
    grad_workspace[col] = varargout_2[col];
  }
  status = 1;
  allFinite = std::isnan(fval);
  if (std::isinf(fval) || allFinite) {
    if (allFinite) {
      status = -3;
    } else if (fval < 0.0) {
      status = -1;
    } else {
      status = -2;
    }
  } else {
    allFinite = true;
    idx_current = 0;
    while (allFinite && (idx_current + 1 <= 113)) {
      allFinite = ((!std::isinf(grad_workspace[idx_current])) &&
                   (!std::isnan(grad_workspace[idx_current])));
      idx_current++;
    }
    if (!allFinite) {
      idx_current--;
      if (std::isnan(grad_workspace[idx_current])) {
        status = -3;
      } else if (grad_workspace[idx_current] < 0.0) {
        status = -1;
      } else {
        status = -2;
      }
    }
  }
  if (status == 1) {
    int idx_mat;
    idx_current = obj.next.next.next.next.next.value;
    if (obj.next.next.next.next.next.value > 0) {
      nlmpcmoveCodeGeneration_anonFcn2(
          obj.next.next.next.next.next.next.next.value.workspace.runtimedata.x,
          obj.next.next.next.next.next.next.next.value.workspace.runtimedata
              .OutputMin,
          obj.next.next.next.next.next.next.next.value.workspace.runtimedata
              .OutputMax,
          x, varargout_1, b_x, varargout_3, varargout_4);
      for (col = 0; col < idx_current; col++) {
        Cineq_workspace[(ineq0 + col) - 1] = varargout_1[col];
      }
      std::copy(&b_x[0], &b_x[98], &Ceq_workspace[eq0 + -1]);
      idx_current = varargout_3.size(0);
      for (col_end = 0; col_end < idx_current; col_end++) {
        col = varargout_3.size(1);
        for (idx_mat = 0; idx_mat < col; idx_mat++) {
          JacIneqTrans_workspace[col_end + ldJI * ((iJI_col + idx_mat) - 1)] =
              varargout_3[col_end + varargout_3.size(0) * idx_mat];
        }
      }
      for (col_end = 0; col_end < 113; col_end++) {
        for (idx_mat = 0; idx_mat < 98; idx_mat++) {
          JacEqTrans_workspace[col_end + ldJE * ((iJE_col + idx_mat) - 1)] =
              varargout_4[col_end + 113 * idx_mat];
        }
      }
    } else {
      nlmpcmoveCodeGeneration_anonFcn2(
          obj.next.next.next.next.next.next.next.value.workspace.runtimedata.x,
          obj.next.next.next.next.next.next.next.value.workspace.runtimedata
              .OutputMin,
          obj.next.next.next.next.next.next.next.value.workspace.runtimedata
              .OutputMax,
          x, varargout_1, b_x, varargout_3, varargout_4);
      std::copy(&b_x[0], &b_x[98], &Ceq_workspace[eq0 + -1]);
      for (col_end = 0; col_end < 113; col_end++) {
        for (idx_mat = 0; idx_mat < 98; idx_mat++) {
          JacEqTrans_workspace[col_end + ldJE * ((iJE_col + idx_mat) - 1)] =
              varargout_4[col_end + 113 * idx_mat];
        }
      }
    }
    status = internal::checkVectorNonFinite(obj.next.next.next.next.next.value,
                                            Cineq_workspace, ineq0);
    if (status == 1) {
      status = internal::checkVectorNonFinite(Ceq_workspace, eq0);
      if (status == 1) {
        allFinite = true;
        idx_current = -1;
        col = iJI_col;
        col_end = (iJI_col + obj.next.next.next.next.next.value) - 1;
        while (allFinite && (col <= col_end)) {
          idx_current = -1;
          while (allFinite && (idx_current + 2 <= 113)) {
            idx_mat = (idx_current + ldJI * (col - 1)) + 1;
            allFinite = ((!std::isinf(JacIneqTrans_workspace[idx_mat])) &&
                         (!std::isnan(JacIneqTrans_workspace[idx_mat])));
            idx_current++;
          }
          col++;
        }
        if (!allFinite) {
          idx_mat = idx_current + ldJI * (col - 2);
          if (std::isnan(JacIneqTrans_workspace[idx_mat])) {
            status = -3;
          } else if (JacIneqTrans_workspace[idx_mat] < 0.0) {
            status = -1;
          } else {
            status = -2;
          }
        } else {
          allFinite = true;
          idx_current = -1;
          col = iJE_col;
          while (allFinite && (col <= iJE_col + 97)) {
            idx_current = -1;
            while (allFinite && (idx_current + 2 <= 113)) {
              idx_mat = (idx_current + ldJE * (col - 1)) + 1;
              allFinite = ((!std::isinf(JacEqTrans_workspace[idx_mat])) &&
                           (!std::isnan(JacEqTrans_workspace[idx_mat])));
              idx_current++;
            }
            col++;
          }
          if (!allFinite) {
            idx_mat = idx_current + ldJE * (col - 2);
            if (std::isnan(JacEqTrans_workspace[idx_mat])) {
              status = -3;
            } else if (JacEqTrans_workspace[idx_mat] < 0.0) {
              status = -1;
            } else {
              status = -2;
            }
          }
        }
      }
    }
  }
  return fval;
}

} // namespace ObjNonlinEvaluator
} // namespace utils
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (evalObjAndConstrAndDerivatives.cpp)
