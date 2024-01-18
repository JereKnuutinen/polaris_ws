//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// nlmpcmoveCodeGeneration.cpp
//
// Code generation for function 'nlmpcmoveCodeGeneration'
//

// Include files
#include "nlmpcmoveCodeGeneration.h"
#include "NMPC_Node_data.h"
#include "all.h"
#include "dynamic_model_nmpc.h"
#include "rt_nonfinite.h"
#include "znlmpc_computeJacobianState.h"
#include "coder_array.h"
#include <algorithm>
#include <cmath>
#include <cstring>

// Function Definitions
namespace coder {
void nlmpcmoveCodeGeneration_anonFcn2(
    const double runtimedata_x[14], const double runtimedata_OutputMin[21],
    const double runtimedata_OutputMax[21], const double z[113],
    ::coder::array<double, 2U> &varargout_1, double varargout_2[98],
    ::coder::array<double, 2U> &varargout_3, double varargout_4[11074])
{
  array<double, 3U> c_Jx;
  array<double, 2U> Jc;
  array<double, 2U> c_tmp;
  array<double, 2U> varargin_1;
  array<double, 1U> b_c;
  array<signed char, 2U> b_Je;
  array<signed char, 1U> r;
  double Jx[9604];
  double b_Jx[4116];
  double Jmv[1372];
  double b_Jmv[1372];
  double X[112];
  double b_X[112];
  double b_z[98];
  double U[16];
  double Umv[16];
  double c_X[14];
  double fk[14];
  double ic[14];
  double d;
  double e;
  int Umv_tmp;
  int fk1_tmp;
  signed char ic_idx_0;
  signed char ic_idx_1;
  bool bv[21];
  bool x[3];
  bool exitg1;
  bool guard1{false};
  bool y;
  std::memset(&X[0], 0, 112U * sizeof(double));
  std::memset(&Umv[0], 0, 16U * sizeof(double));
  for (int i{0}; i < 14; i++) {
    d = 0.0;
    for (int b_i{0}; b_i < 14; b_i++) {
      d += static_cast<double>(iv[i + 14 * b_i]) * z[b_i + 98];
    }
    fk[i] = d;
  }
  for (int i{0}; i < 2; i++) {
    for (int b_i{0}; b_i < 7; b_i++) {
      Umv[b_i + (i << 3)] = fk[i + (b_i << 1)];
    }
  }
  e = z[112];
  std::copy(&z[0], &z[98], &b_z[0]);
  for (int i{0}; i < 14; i++) {
    for (int b_i{0}; b_i < 7; b_i++) {
      X[(b_i + (i << 3)) + 1] = b_z[i + 14 * b_i];
    }
    X[i << 3] = runtimedata_x[i];
  }
  for (int b_i{0}; b_i < 2; b_i++) {
    Umv_tmp = b_i << 3;
    Umv[Umv_tmp + 7] = Umv[Umv_tmp + 6];
    std::copy(&Umv[Umv_tmp],
              &Umv[static_cast<int>(static_cast<unsigned int>(Umv_tmp) + 8U)],
              &U[Umv_tmp]);
  }
  std::memset(&Jx[0], 0, 9604U * sizeof(double));
  std::memset(&Jmv[0], 0, 1372U * sizeof(double));
  std::memset(&varargout_2[0], 0, 98U * sizeof(double));
  for (int i{0}; i < 14; i++) {
    ic[i] = static_cast<double>(i) + 1.0;
  }
  for (int i{0}; i < 8; i++) {
    Umv_tmp = i << 1;
    Umv[Umv_tmp] = U[i];
    Umv[Umv_tmp + 1] = U[i + 8];
  }
  for (int i{0}; i < 8; i++) {
    for (int b_i{0}; b_i < 14; b_i++) {
      b_X[b_i + 14 * i] = X[i + (b_i << 3)];
    }
  }
  for (int b_i{0}; b_i < 7; b_i++) {
    double Ak[196];
    double Ak1[196];
    double Bk1[28];
    double val[28];
    double fk1[14];
    double b_Umv[2];
    Umv_tmp = b_i << 1;
    dynamic_model_nmpc(&b_X[14 * b_i], &Umv[Umv_tmp], fk);
    std::copy(&(*(double(*)[14]) & b_X[14 * b_i])[0],
              &(*(double(*)[14]) & b_X[14 * b_i])[14], &c_X[0]);
    b_Umv[0] = Umv[Umv_tmp];
    b_Umv[1] = Umv[1 + Umv_tmp];
    znlmpc_computeJacobianState(fk, c_X, b_Umv, Ak, val);
    fk1_tmp = 14 * (b_i + 1);
    dynamic_model_nmpc(&b_X[fk1_tmp], &Umv[Umv_tmp], fk1);
    std::copy(&(*(double(*)[14]) & b_X[fk1_tmp])[0],
              &(*(double(*)[14]) & b_X[fk1_tmp])[14], &c_X[0]);
    b_Umv[0] = Umv[Umv_tmp];
    b_Umv[1] = Umv[1 + Umv_tmp];
    znlmpc_computeJacobianState(fk1, c_X, b_Umv, Ak1, Bk1);
    for (int i{0}; i < 14; i++) {
      varargout_2[static_cast<int>(ic[i]) - 1] =
          (b_X[i + 14 * b_i] + 0.05 * (fk[i] + fk1[i])) - b_X[i + fk1_tmp];
    }
    if (b_i + 1 > 1) {
      for (fk1_tmp = 0; fk1_tmp < 14; fk1_tmp++) {
        for (int i{0}; i < 14; i++) {
          Jx[((static_cast<int>(ic[i]) + 98 * fk1_tmp) + 1372 * (b_i - 1)) -
             1] = 0.05 * Ak[i + 14 * fk1_tmp];
        }
        Umv_tmp = ((static_cast<signed char>(ic[fk1_tmp]) + 98 * fk1_tmp) +
                   1372 * (b_i - 1)) -
                  1;
        Jx[Umv_tmp]++;
      }
    }
    for (fk1_tmp = 0; fk1_tmp < 14; fk1_tmp++) {
      for (int i{0}; i < 14; i++) {
        Jx[((static_cast<int>(ic[i]) + 98 * fk1_tmp) + 1372 * b_i) - 1] =
            0.05 * Ak1[i + 14 * fk1_tmp];
      }
      Umv_tmp = ((static_cast<signed char>(ic[fk1_tmp]) + 98 * fk1_tmp) +
                 1372 * b_i) -
                1;
      Jx[Umv_tmp]--;
    }
    for (int i{0}; i < 28; i++) {
      val[i] = 0.05 * (val[i] + Bk1[i]);
    }
    for (fk1_tmp = 0; fk1_tmp < 2; fk1_tmp++) {
      for (int i{0}; i < 14; i++) {
        Jmv[((static_cast<int>(ic[i]) + 98 * fk1_tmp) + 196 * b_i) - 1] =
            val[i + 14 * fk1_tmp];
      }
    }
    for (int i{0}; i < 14; i++) {
      ic[i] += 14.0;
    }
  }
  for (int i{0}; i < 21; i++) {
    bv[i] = std::isinf(runtimedata_OutputMin[i]);
  }
  all(bv, x);
  y = true;
  fk1_tmp = 0;
  exitg1 = false;
  while ((!exitg1) && (fk1_tmp <= 2)) {
    if (!x[fk1_tmp]) {
      y = false;
      exitg1 = true;
    } else {
      fk1_tmp++;
    }
  }
  guard1 = false;
  if (y) {
    for (int i{0}; i < 21; i++) {
      bv[i] = std::isinf(runtimedata_OutputMax[i]);
    }
    all(bv, x);
    y = true;
    fk1_tmp = 0;
    exitg1 = false;
    while ((!exitg1) && (fk1_tmp <= 2)) {
      if (!x[fk1_tmp]) {
        y = false;
        exitg1 = true;
      } else {
        fk1_tmp++;
      }
    }
    if (y) {
      c_tmp.set_size(0, 0);
      Jc.set_size(0, 0);
    } else {
      guard1 = true;
    }
  } else {
    guard1 = true;
  }
  if (guard1) {
    double c[42];
    signed char Je[42];
    signed char ic_idx_2;
    bool icf[42];
    for (int b_i{0}; b_i < 42; b_i++) {
      c[b_i] = 0.0;
      icf[b_i] = true;
    }
    std::memset(&b_Jx[0], 0, 4116U * sizeof(double));
    for (int b_i{0}; b_i < 42; b_i++) {
      Je[b_i] = 0;
    }
    ic_idx_0 = 1;
    ic_idx_1 = 2;
    ic_idx_2 = 3;
    for (int b_i{0}; b_i < 7; b_i++) {
      double d1;
      double d2;
      double d3;
      d = runtimedata_OutputMin[b_i];
      icf[ic_idx_0 - 1] = ((!std::isinf(d)) && (!std::isnan(d)));
      d = runtimedata_OutputMin[b_i + 7];
      icf[ic_idx_1 - 1] = ((!std::isinf(d)) && (!std::isnan(d)));
      d1 = runtimedata_OutputMin[b_i + 14];
      icf[ic_idx_2 - 1] = ((!std::isinf(d1)) && (!std::isnan(d1)));
      d2 = runtimedata_OutputMax[b_i];
      icf[static_cast<signed char>(ic_idx_0 + 3) - 1] =
          ((!std::isinf(d2)) && (!std::isnan(d2)));
      d2 = runtimedata_OutputMax[b_i + 7];
      icf[static_cast<signed char>(ic_idx_1 + 3) - 1] =
          ((!std::isinf(d2)) && (!std::isnan(d2)));
      d3 = runtimedata_OutputMax[b_i + 14];
      icf[static_cast<signed char>(ic_idx_2 + 3) - 1] =
          ((!std::isinf(d3)) && (!std::isnan(d3)));
      y = false;
      fk1_tmp = 0;
      exitg1 = false;
      while ((!exitg1) && (fk1_tmp <= 5)) {
        short b_ic[6];
        b_ic[0] = static_cast<short>(ic_idx_0 - 1);
        b_ic[3] = static_cast<short>(ic_idx_0 + 2);
        b_ic[1] = static_cast<short>(ic_idx_1 - 1);
        b_ic[4] = static_cast<short>(ic_idx_1 + 2);
        b_ic[2] = static_cast<short>(ic_idx_2 - 1);
        b_ic[5] = static_cast<short>(ic_idx_2 + 2);
        if (icf[b_ic[fk1_tmp]]) {
          y = true;
          exitg1 = true;
        } else {
          fk1_tmp++;
        }
      }
      if (y) {
        double Ck[42];
        double b_val[42];
        double dx;
        double yk_idx_0;
        double yk_idx_1;
        double yk_idx_2;
        //  define outputs that are controller or constrained '
        //  by the controller
        // ltr=-2/(m*g*tw)*(cFL*X(10)+kFL*X(7));
        // Vg=sqrt(X(4)^2+X(5)^2+X(6)^2);
        //  alat = yaw rate * forward speed; U(1)*Vg^2; %lateral accelration
        //  from curv command
        // out=[X(1); X(2); ltr; Vg; a_lat]; % NMPC will apply constraints using
        // these points
        yk_idx_0 = X[b_i + 1];
        yk_idx_1 = X[b_i + 9];
        yk_idx_2 = X[b_i + 25] * X[b_i + 89];
        //  NMPC will apply constraints using these points
        for (fk1_tmp = 0; fk1_tmp < 14; fk1_tmp++) {
          dx = X[(b_i + (fk1_tmp << 3)) + 1];
          ic[fk1_tmp] = dx;
          dx = std::abs(dx);
          fk[fk1_tmp] = dx;
          if (dx < 1.0) {
            fk[fk1_tmp] = 1.0;
          }
        }
        for (Umv_tmp = 0; Umv_tmp < 14; Umv_tmp++) {
          double f_idx_0;
          double f_idx_1;
          double f_idx_2;
          dx = 1.0E-6 * fk[Umv_tmp];
          ic[Umv_tmp] += dx;
          //  define outputs that are controller or constrained '
          //  by the controller
          // ltr=-2/(m*g*tw)*(cFL*X(10)+kFL*X(7));
          // Vg=sqrt(X(4)^2+X(5)^2+X(6)^2);
          //  alat = yaw rate * forward speed; U(1)*Vg^2; %lateral accelration
          //  from curv command
          // out=[X(1); X(2); ltr; Vg; a_lat]; % NMPC will apply constraints
          // using these points
          f_idx_0 = ic[0];
          f_idx_1 = ic[1];
          f_idx_2 = ic[3] * ic[11];
          //  NMPC will apply constraints using these points
          ic[Umv_tmp] -= dx;
          Ck[3 * Umv_tmp] = (f_idx_0 - yk_idx_0) / dx;
          Ck[3 * Umv_tmp + 1] = (f_idx_1 - yk_idx_1) / dx;
          Ck[3 * Umv_tmp + 2] = (f_idx_2 - yk_idx_2) / dx;
        }
        c[ic_idx_0 - 1] = (runtimedata_OutputMin[b_i] - e) - yk_idx_0;
        c[ic_idx_1 - 1] = (d - e) - yk_idx_1;
        c[ic_idx_2 - 1] = (d1 - e) - yk_idx_2;
        c[ic_idx_0 + 2] = (yk_idx_0 - runtimedata_OutputMax[b_i]) - e;
        c[ic_idx_1 + 2] = (yk_idx_1 - d2) - e;
        c[ic_idx_2 + 2] = (yk_idx_2 - d3) - e;
        for (int i{0}; i < 42; i++) {
          b_val[i] = -Ck[i];
        }
        for (fk1_tmp = 0; fk1_tmp < 14; fk1_tmp++) {
          b_Jx[((ic_idx_0 + 42 * fk1_tmp) + 588 * b_i) - 1] =
              b_val[3 * fk1_tmp];
          b_Jx[((ic_idx_1 + 42 * fk1_tmp) + 588 * b_i) - 1] =
              b_val[3 * fk1_tmp + 1];
          b_Jx[((ic_idx_2 + 42 * fk1_tmp) + 588 * b_i) - 1] =
              b_val[3 * fk1_tmp + 2];
        }
        for (fk1_tmp = 0; fk1_tmp < 14; fk1_tmp++) {
          b_Jx[((ic_idx_0 + 42 * fk1_tmp) + 588 * b_i) + 2] = Ck[3 * fk1_tmp];
          b_Jx[((ic_idx_1 + 42 * fk1_tmp) + 588 * b_i) + 2] =
              Ck[3 * fk1_tmp + 1];
          b_Jx[((ic_idx_2 + 42 * fk1_tmp) + 588 * b_i) + 2] =
              Ck[3 * fk1_tmp + 2];
        }
        Je[ic_idx_0 - 1] = -1;
        Je[ic_idx_1 - 1] = -1;
        Je[ic_idx_2 - 1] = -1;
        Je[static_cast<signed char>(ic_idx_0 + 3) - 1] = -1;
        Je[static_cast<signed char>(ic_idx_1 + 3) - 1] = -1;
        Je[static_cast<signed char>(ic_idx_2 + 3) - 1] = -1;
      }
      ic_idx_0 = static_cast<signed char>(ic_idx_0 + 6);
      ic_idx_1 = static_cast<signed char>(ic_idx_1 + 6);
      ic_idx_2 = static_cast<signed char>(ic_idx_2 + 6);
    }
    Umv_tmp = 0;
    for (int b_i{0}; b_i < 42; b_i++) {
      if (icf[b_i]) {
        Umv_tmp++;
      }
    }
    r.set_size(Umv_tmp);
    Umv_tmp = 0;
    for (int b_i{0}; b_i < 42; b_i++) {
      if (icf[b_i]) {
        r[Umv_tmp] = static_cast<signed char>(b_i);
        Umv_tmp++;
      }
    }
    b_c.set_size(r.size(0));
    fk1_tmp = r.size(0);
    for (int i{0}; i < fk1_tmp; i++) {
      b_c[i] = c[r[i]];
    }
    c_tmp.set_size(r.size(0), 1);
    fk1_tmp = r.size(0);
    for (int i{0}; i < fk1_tmp; i++) {
      c_tmp[i] = b_c[i];
    }
    if (r.size(0) == 0) {
      Jc.set_size(0, 0);
    } else {
      c_Jx.set_size(r.size(0), 14, 7);
      fk1_tmp = r.size(0);
      for (int i{0}; i < 7; i++) {
        for (int b_i{0}; b_i < 14; b_i++) {
          for (Umv_tmp = 0; Umv_tmp < fk1_tmp; Umv_tmp++) {
            c_Jx[(Umv_tmp + c_Jx.size(0) * b_i) + c_Jx.size(0) * 14 * i] =
                b_Jx[(r[Umv_tmp] + 42 * b_i) + 588 * i];
          }
        }
      }
      Umv_tmp = r.size(0);
      varargin_1.set_size(98, r.size(0));
      fk1_tmp = r.size(0);
      for (int i{0}; i < fk1_tmp; i++) {
        for (int b_i{0}; b_i < 98; b_i++) {
          varargin_1[b_i + 98 * i] = c_Jx[i + Umv_tmp * b_i];
        }
      }
      Umv_tmp = varargin_1.size(1);
      b_Je.set_size(1, r.size(0));
      fk1_tmp = r.size(0);
      for (int i{0}; i < fk1_tmp; i++) {
        b_Je[i] = Je[r[i]];
      }
      Jc.set_size(113, varargin_1.size(1));
      for (int i{0}; i < Umv_tmp; i++) {
        for (int b_i{0}; b_i < 98; b_i++) {
          Jc[b_i + Jc.size(0) * i] = varargin_1[b_i + 98 * i];
        }
        for (int b_i{0}; b_i < 14; b_i++) {
          Jc[(b_i + Jc.size(0) * i) + 98] = 0.0;
        }
        Jc[Jc.size(0) * i + 112] = b_Je[i];
      }
    }
  }
  y = ((c_tmp.size(0) != 0) && (c_tmp.size(1) != 0));
  ic_idx_0 = static_cast<signed char>(c_tmp.size(0));
  varargout_1.set_size(static_cast<int>(ic_idx_0), static_cast<int>(y));
  Umv_tmp = y;
  for (int i{0}; i < Umv_tmp; i++) {
    fk1_tmp = ic_idx_0;
    for (int b_i{0}; b_i < fk1_tmp; b_i++) {
      varargout_1[b_i] = c_tmp[b_i];
    }
  }
  y = ((Jc.size(0) != 0) && (Jc.size(1) != 0));
  if (y) {
    ic_idx_0 = static_cast<signed char>(Jc.size(0));
  } else {
    ic_idx_0 = 0;
  }
  if ((ic_idx_0 == 0) || y) {
    ic_idx_1 = static_cast<signed char>(Jc.size(1));
  } else {
    ic_idx_1 = 0;
  }
  varargout_3.set_size(static_cast<int>(ic_idx_0), static_cast<int>(ic_idx_1));
  fk1_tmp = ic_idx_1;
  for (int i{0}; i < fk1_tmp; i++) {
    Umv_tmp = ic_idx_0;
    for (int b_i{0}; b_i < Umv_tmp; b_i++) {
      varargout_3[b_i + varargout_3.size(0) * i] = Jc[b_i + ic_idx_0 * i];
    }
  }
  for (int i{0}; i < 14; i++) {
    for (int b_i{0}; b_i < 98; b_i++) {
      d = 0.0;
      for (Umv_tmp = 0; Umv_tmp < 14; Umv_tmp++) {
        d +=
            Jmv[b_i + 98 * Umv_tmp] * static_cast<double>(iv[Umv_tmp + 14 * i]);
      }
      b_Jmv[i + 14 * b_i] = d;
    }
  }
  for (int i{0}; i < 98; i++) {
    for (int b_i{0}; b_i < 98; b_i++) {
      varargout_4[b_i + 113 * i] = Jx[i + 98 * b_i];
    }
    std::copy(&b_Jmv[i * 14],
              &b_Jmv[static_cast<int>(static_cast<unsigned int>(i * 14) + 14U)],
              &varargout_4[i * 113 + 98]);
    varargout_4[113 * i + 112] = 0.0;
  }
}

} // namespace coder

// End of code generation (nlmpcmoveCodeGeneration.cpp)
