//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// evalObjAndConstr.cpp
//
// Code generation for function 'evalObjAndConstr'
//

// Include files
#include "evalObjAndConstr.h"
#include "NMPC_Node_data.h"
#include "NMPC_Node_internal_types1.h"
#include "NMPC_Node_internal_types2.h"
#include "all.h"
#include "anonymous_function.h"
#include "checkVectorNonFinite.h"
#include "custom_cost_dynamic.h"
#include "dynamic_model_nmpc.h"
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
double evalObjAndConstr(const ::coder::internal::i_stickyStruct &obj,
                        const double x[113],
                        ::coder::array<double, 1U> &Cineq_workspace, int ineq0,
                        double Ceq_workspace[98], int eq0, int &status)
{
  array<double, 2U> b_c;
  array<double, 2U> c_c;
  array<double, 1U> d_c;
  array<int, 2U> ineqRange;
  array<signed char, 1U> r;
  double X[112];
  double b_x[98];
  double U[16];
  double Umv[16];
  double fk[14];
  double d;
  double fval;
  int k;
  int n;
  bool y;
  std::memset(&X[0], 0, 112U * sizeof(double));
  std::memset(&Umv[0], 0, 16U * sizeof(double));
  for (int i{0}; i < 14; i++) {
    d = 0.0;
    for (k = 0; k < 14; k++) {
      d += static_cast<double>(iv[i + 14 * k]) * x[k + 98];
    }
    fk[i] = d;
  }
  for (int i{0}; i < 2; i++) {
    for (k = 0; k < 7; k++) {
      Umv[k + (i << 3)] = fk[i + (k << 1)];
    }
  }
  std::copy(&x[0], &x[98], &b_x[0]);
  for (int i{0}; i < 14; i++) {
    for (k = 0; k < 7; k++) {
      X[(k + (i << 3)) + 1] = b_x[i + 14 * k];
    }
    X[i << 3] = obj.next.next.next.next.next.next.next.next.value.workspace
                    .runtimedata.x[i];
  }
  for (int b_i{0}; b_i < 2; b_i++) {
    n = b_i << 3;
    Umv[n + 7] = Umv[n + 6];
    std::copy(&Umv[n],
              &Umv[static_cast<int>(static_cast<unsigned int>(n) + 8U)], &U[n]);
  }
  fval = custom_cost_dynamic(X, U,
                             obj.next.next.next.next.next.next.next.next.value
                                 .workspace.userdata.References);
  status = 1;
  y = std::isnan(fval);
  if (std::isinf(fval) || y) {
    if (y) {
      status = -3;
    } else if (fval < 0.0) {
      status = -1;
    } else {
      status = -2;
    }
  }
  if (status == 1) {
    double b_X[112];
    double e;
    int yk;
    signed char ic[14];
    signed char ic_idx_0;
    bool bv[21];
    bool c_x[3];
    bool exitg1;
    bool guard1{false};
    if (obj.next.next.next.next.next.value - 1 < 0) {
      n = 0;
    } else {
      n = obj.next.next.next.next.next.value;
    }
    ineqRange.set_size(1, n);
    if (n > 0) {
      ineqRange[0] = 0;
      yk = 0;
      for (k = 2; k <= n; k++) {
        yk++;
        ineqRange[k - 1] = yk;
      }
    }
    ineqRange.set_size(1, ineqRange.size(1));
    n = ineqRange.size(1) - 1;
    for (int i{0}; i <= n; i++) {
      ineqRange[i] = ineqRange[i] + ineq0;
    }
    std::memset(&X[0], 0, 112U * sizeof(double));
    std::memset(&Umv[0], 0, 16U * sizeof(double));
    for (int i{0}; i < 14; i++) {
      d = 0.0;
      for (k = 0; k < 14; k++) {
        d += static_cast<double>(iv[i + 14 * k]) * x[k + 98];
      }
      fk[i] = d;
    }
    for (int i{0}; i < 2; i++) {
      for (k = 0; k < 7; k++) {
        Umv[k + (i << 3)] = fk[i + (k << 1)];
      }
    }
    e = x[112];
    std::copy(&x[0], &x[98], &b_x[0]);
    for (int i{0}; i < 14; i++) {
      for (k = 0; k < 7; k++) {
        X[(k + (i << 3)) + 1] = b_x[i + 14 * k];
      }
      X[i << 3] = obj.next.next.next.next.next.next.next.value.workspace
                      .runtimedata.x[i];
    }
    for (int b_i{0}; b_i < 2; b_i++) {
      n = b_i << 3;
      Umv[n + 7] = Umv[n + 6];
      std::copy(&Umv[n],
                &Umv[static_cast<int>(static_cast<unsigned int>(n) + 8U)],
                &U[n]);
    }
    std::memset(&b_x[0], 0, 98U * sizeof(double));
    for (int i{0}; i < 14; i++) {
      ic[i] = static_cast<signed char>(i + 1);
    }
    for (int i{0}; i < 8; i++) {
      n = i << 1;
      Umv[n] = U[i];
      Umv[n + 1] = U[i + 8];
    }
    for (int i{0}; i < 8; i++) {
      for (k = 0; k < 14; k++) {
        b_X[k + 14 * i] = X[i + (k << 3)];
      }
    }
    for (int b_i{0}; b_i < 7; b_i++) {
      double fk1[14];
      n = b_i << 1;
      dynamic_model_nmpc(&b_X[14 * b_i], &Umv[n], fk);
      yk = 14 * (b_i + 1);
      dynamic_model_nmpc(&b_X[yk], &Umv[n], fk1);
      for (int i{0}; i < 14; i++) {
        ic_idx_0 = ic[i];
        b_x[ic_idx_0 - 1] =
            (b_X[i + 14 * b_i] + 0.05 * (fk[i] + fk1[i])) - b_X[i + yk];
        ic_idx_0 = static_cast<signed char>(ic_idx_0 + 14);
        ic[i] = ic_idx_0;
      }
    }
    for (int i{0}; i < 21; i++) {
      bv[i] = std::isinf(obj.next.next.next.next.next.next.next.value.workspace
                             .runtimedata.OutputMin[i]);
    }
    all(bv, c_x);
    y = true;
    k = 0;
    exitg1 = false;
    while ((!exitg1) && (k <= 2)) {
      if (!c_x[k]) {
        y = false;
        exitg1 = true;
      } else {
        k++;
      }
    }
    guard1 = false;
    if (y) {
      for (int i{0}; i < 21; i++) {
        bv[i] = std::isinf(obj.next.next.next.next.next.next.next.value
                               .workspace.runtimedata.OutputMax[i]);
      }
      all(bv, c_x);
      y = true;
      k = 0;
      exitg1 = false;
      while ((!exitg1) && (k <= 2)) {
        if (!c_x[k]) {
          y = false;
          exitg1 = true;
        } else {
          k++;
        }
      }
      if (y) {
        b_c.set_size(0, 0);
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }
    if (guard1) {
      double c[42];
      signed char ic_idx_1;
      signed char ic_idx_2;
      bool icf[42];
      for (int b_i{0}; b_i < 42; b_i++) {
        c[b_i] = 0.0;
        icf[b_i] = true;
      }
      ic_idx_0 = 1;
      ic_idx_1 = 2;
      ic_idx_2 = 3;
      for (int b_i{0}; b_i < 7; b_i++) {
        double d1;
        double d2;
        double d3;
        d = obj.next.next.next.next.next.next.next.value.workspace.runtimedata
                .OutputMin[b_i];
        icf[ic_idx_0 - 1] = ((!std::isinf(d)) && (!std::isnan(d)));
        d = obj.next.next.next.next.next.next.next.value.workspace.runtimedata
                .OutputMin[b_i + 7];
        icf[ic_idx_1 - 1] = ((!std::isinf(d)) && (!std::isnan(d)));
        d1 = obj.next.next.next.next.next.next.next.value.workspace.runtimedata
                 .OutputMin[b_i + 14];
        icf[ic_idx_2 - 1] = ((!std::isinf(d1)) && (!std::isnan(d1)));
        d2 = obj.next.next.next.next.next.next.next.value.workspace.runtimedata
                 .OutputMax[b_i];
        icf[ic_idx_0 + 2] = ((!std::isinf(d2)) && (!std::isnan(d2)));
        d2 = obj.next.next.next.next.next.next.next.value.workspace.runtimedata
                 .OutputMax[b_i + 7];
        icf[ic_idx_1 + 2] = ((!std::isinf(d2)) && (!std::isnan(d2)));
        d3 = obj.next.next.next.next.next.next.next.value.workspace.runtimedata
                 .OutputMax[b_i + 14];
        icf[ic_idx_2 + 2] = ((!std::isinf(d3)) && (!std::isnan(d3)));
        y = false;
        k = 0;
        exitg1 = false;
        while ((!exitg1) && (k <= 5)) {
          short b_ic[6];
          b_ic[0] = static_cast<short>(ic_idx_0 - 1);
          b_ic[3] = static_cast<short>(ic_idx_0 + 2);
          b_ic[1] = static_cast<short>(ic_idx_1 - 1);
          b_ic[4] = static_cast<short>(ic_idx_1 + 2);
          b_ic[2] = static_cast<short>(ic_idx_2 - 1);
          b_ic[5] = static_cast<short>(ic_idx_2 + 2);
          if (icf[b_ic[k]]) {
            y = true;
            exitg1 = true;
          } else {
            k++;
          }
        }
        if (y) {
          double yk_idx_0;
          double yk_idx_1;
          double yk_idx_2;
          //  define outputs that are controller or constrained '
          //  by the controller
          // ltr=-2/(m*g*tw)*(cFL*X(10)+kFL*X(7));
          // Vg=sqrt(X(4)^2+X(5)^2+X(6)^2);
          //  alat = yaw rate * forward speed; U(1)*Vg^2; %lateral accelration
          //  from curv command
          // out=[X(1); X(2); ltr; Vg; a_lat]; % NMPC will apply constraints
          // using these points
          yk_idx_0 = X[b_i + 1];
          yk_idx_1 = X[b_i + 9];
          yk_idx_2 = X[b_i + 25] * X[b_i + 89];
          //  NMPC will apply constraints using these points
          c[ic_idx_0 - 1] = (obj.next.next.next.next.next.next.next.value
                                 .workspace.runtimedata.OutputMin[b_i] -
                             e) -
                            yk_idx_0;
          c[ic_idx_1 - 1] = (d - e) - yk_idx_1;
          c[ic_idx_2 - 1] = (d1 - e) - yk_idx_2;
          c[ic_idx_0 + 2] =
              (yk_idx_0 - obj.next.next.next.next.next.next.next.value.workspace
                              .runtimedata.OutputMax[b_i]) -
              e;
          c[ic_idx_1 + 2] = (yk_idx_1 - d2) - e;
          c[ic_idx_2 + 2] = (yk_idx_2 - d3) - e;
        }
        ic_idx_0 = static_cast<signed char>(ic_idx_0 + 6);
        ic_idx_1 = static_cast<signed char>(ic_idx_1 + 6);
        ic_idx_2 = static_cast<signed char>(ic_idx_2 + 6);
      }
      n = 0;
      for (int b_i{0}; b_i < 42; b_i++) {
        if (icf[b_i]) {
          n++;
        }
      }
      r.set_size(n);
      n = 0;
      for (int b_i{0}; b_i < 42; b_i++) {
        if (icf[b_i]) {
          r[n] = static_cast<signed char>(b_i);
          n++;
        }
      }
      d_c.set_size(r.size(0));
      n = r.size(0);
      for (int i{0}; i < n; i++) {
        d_c[i] = c[r[i]];
      }
      b_c.set_size(r.size(0), 1);
      n = r.size(0);
      for (int i{0}; i < n; i++) {
        b_c[i] = d_c[i];
      }
    }
    y = ((b_c.size(0) != 0) && (b_c.size(1) != 0));
    ic_idx_0 = static_cast<signed char>(b_c.size(0));
    c_c.set_size(static_cast<int>(ic_idx_0), static_cast<int>(y));
    yk = y;
    for (int i{0}; i < yk; i++) {
      n = ic_idx_0;
      for (k = 0; k < n; k++) {
        c_c[k] = b_c[k];
      }
    }
    n = ineqRange.size(1);
    for (int i{0}; i < n; i++) {
      Cineq_workspace[ineqRange[i] - 1] = c_c[i];
    }
    std::copy(&b_x[0], &b_x[98], &Ceq_workspace[eq0 + -1]);
    status = internal::checkVectorNonFinite(obj.next.next.next.next.next.value,
                                            Cineq_workspace, ineq0);
    if (status == 1) {
      status = internal::checkVectorNonFinite(Ceq_workspace, eq0);
    }
  }
  return fval;
}

} // namespace ObjNonlinEvaluator
} // namespace utils
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (evalObjAndConstr.cpp)
