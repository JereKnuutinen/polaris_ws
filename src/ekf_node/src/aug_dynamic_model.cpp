//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// aug_dynamic_model.cpp
//
// Code generation for function 'aug_dynamic_model'
//

// Include files
#include "aug_dynamic_model.h"
#include "EKF_Node_data.h"
#include "EKF_Node_rtwutil.h"
#include "enu2lla.h"
#include "rt_nonfinite.h"
#include "tand.h"
#include "/home/mpc/Desktop/mba/mba/mba.hpp"
#include "intpol_codegen.h"
#include "myInterpolator2.h"
#include <cmath>

// Function Definitions
void aug_dynamic_model_anonFcn1(const double uk[3], const double xk[14],
                                double varargout_1[14])
{
  static const double Dx_mat[6]{0.15, 0.0, 0.0, -0.15, 0.0, 0.0};
  static const double Dy_mat[6]{0.0, 0.25, 0.0, 0.0, -0.25, 0.0};
  static const signed char b_iv[3]{0, 0, 1};
  double C[9];
  double C_tmp[9];
  double PR[9];
  double PR_tmp[9];
  double RP[9];
  double RTemp[9];
  double b_C_tmp[9];
  double b_PR_tmp[9];
  double b_dv[9];
  double c_C_tmp[9];
  double dv1[9];
  double P_FL_dx[6];
  double P_FL_dy[6];
  double P_FR[6];
  double P_FR_dx[6];
  double P_FR_dy[6];
  double P_RL_dx[6];
  double P_RR_dx[6];
  double P_RR_dy[6];
  double FwFL[3];
  double FwFR[3];
  double FwRL[3];
  double FwRR[3];
  double G_vec[3];
  double P_CG[3];
  double P_FL[3];
  double P_FL_LLA[3];
  double P_FR_LLA[3];
  double P_RL[3];
  double P_RL_LLA[3];
  double P_RR[3];
  double P_RR_LLA[3];
  double RpFL[3];
  double RpFR[3];
  double RpRL[3];
  double RpRR[3];
  double VgFL[3];
  double VgFR[3];
  double VgRL[3];
  double VgRR[3];
  double VuFL_tmp[3];
  double VuFR_tmp[3];
  double VuRL_tmp[3];
  double VuRR_tmp[3];
  double b_P_FR[3];
  double b_xk[3];
  double b_y[3];
  double c_y[3];
  double d_y[3];
  double dotENU[3];
  double grad_FL[3];
  double grad_FR[3];
  double grad_RL[3];
  double grad_RR[3];
  double y[3];
  double FcFL;
  double FcFR;
  double FcRL;
  double FcRR;
  double FrFL;
  double FrFR;
  double FrRL;
  double FrRR;
  double FztFL;
  double FztFL_tmp;
  double FztFR;
  double FztRL;
  double FztRL_tmp;
  double FztRR;
  double FztRR_tmp;
  double b_varargout_1_tmp_tmp;
  double c_varargout_1_tmp_tmp;
  double d;
  double d1;
  double d2;
  double d3;
  double d4;
  double d5;
  double d6;
  double d7;
  double d8;
  double d_varargout_1_tmp_tmp;
  double dpitch;
  double droll;
  double dyaw;
  double e_varargout_1_tmp_tmp;
  double fFL;
  double fFR;
  double fRL;
  double fRR;
  double f_varargout_1_tmp_tmp;
  double gamma_FL;
  double gamma_FR;
  double gamma_RL;
  double gamma_RR;
  double mur;
  double stgL;
  double stgR;
  double varargout_1_tmp;
  double varargout_1_tmp_tmp;
  int i;
  int i1;
  int k;
  //  Full Car Model
  //  df_{F,R}/ds
  //  extract inputs
  //  extract the state vector components
  //  position of CG
  //  CG velocities in body frame
  //  Euler angles
  //  body rate about CG
  //  extract curvature state
  //  add TGC interaction coefficient as a state
  //  Cr_state = xk(15);
  //  rotation matrix around X-axis and its derivatives
  //  Rotation matrix around Y-axis and its derivatives
  //  Rotation matrix around Z-axis and its derivatives
  //  pre-compute joint rotation matrices
  varargout_1_tmp_tmp = std::sin(xk[7]);
  b_varargout_1_tmp_tmp = std::cos(xk[7]);
  PR_tmp[0] = b_varargout_1_tmp_tmp;
  PR_tmp[3] = 0.0;
  PR_tmp[6] = varargout_1_tmp_tmp;
  PR_tmp[2] = -varargout_1_tmp_tmp;
  PR_tmp[5] = 0.0;
  PR_tmp[8] = b_varargout_1_tmp_tmp;
  c_varargout_1_tmp_tmp = std::sin(xk[6]);
  d_varargout_1_tmp_tmp = std::cos(xk[6]);
  PR_tmp[1] = 0.0;
  b_PR_tmp[0] = 1.0;
  PR_tmp[4] = 1.0;
  b_PR_tmp[3] = 0.0;
  PR_tmp[7] = 0.0;
  b_PR_tmp[6] = 0.0;
  b_PR_tmp[1] = 0.0;
  b_PR_tmp[4] = d_varargout_1_tmp_tmp;
  b_PR_tmp[7] = -c_varargout_1_tmp_tmp;
  b_PR_tmp[2] = 0.0;
  b_PR_tmp[5] = c_varargout_1_tmp_tmp;
  b_PR_tmp[8] = d_varargout_1_tmp_tmp;
  //  CG-fixed to TGC-patch frame
  //  TGC-patch to CG-fixed frame
  //  body to global (CG) transformation
  e_varargout_1_tmp_tmp = std::sin(xk[8]);
  f_varargout_1_tmp_tmp = std::cos(xk[8]);
  C_tmp[0] = f_varargout_1_tmp_tmp;
  C_tmp[3] = -e_varargout_1_tmp_tmp;
  C_tmp[6] = 0.0;
  C_tmp[1] = e_varargout_1_tmp_tmp;
  C_tmp[4] = f_varargout_1_tmp_tmp;
  C_tmp[7] = 0.0;
  for (i = 0; i < 3; i++) {
    d = PR_tmp[i];
    i1 = static_cast<int>(PR_tmp[i + 3]);
    d1 = PR_tmp[i + 6];
    for (k = 0; k < 3; k++) {
      d2 = (d * b_PR_tmp[3 * k] +
            static_cast<double>(i1) * b_PR_tmp[3 * k + 1]) +
           d1 * b_PR_tmp[3 * k + 2];
      PR[i + 3 * k] = d2;
      RP[k + 3 * i] = d2;
    }
    C_tmp[3 * i + 2] = b_iv[i];
  }
  //  compute velocities in inertial frame
  b_xk[0] = xk[3];
  b_xk[1] = xk[4];
  b_xk[2] = xk[5];
  //  compute traction force in TGC-patch frame
  // Ry(pitch)*Rx(roll)*[m*acmd; 0; 0];
  //  moment arms in global coordinates
  //  compute road profiles from map at each time instant
  P_CG[0] = xk[0];
  P_CG[1] = xk[1];
  P_CG[2] = xk[2];
  //  convert the wheel locations in LLA
  //  interpolate the height corresponding to wheel position
  //  in LLA map first argument is long, and second is lat
  // fFR = SIM_MAP_LLA(P_FR_LLA(2), P_FR_LLA(1)); fFL = SIM_MAP_LLA(P_FL_LLA(2),
  // P_FL_LLA(1)); fRR = SIM_MAP_LLA(P_RR_LLA(2), P_RR_LLA(1)); fRL =
  // SIM_MAP_LLA(P_RL_LLA(2), P_RL_LLA(1));
  //  compute the derivative of height using current position data
  //  this values must be equal or smaller than map resolution
  //  compute the positions in LLA +/- dx distance from the wheel positions
  //  Dx_mat = [dx -dx;0 0;0 0];
  //  P_FR_dx = enu2lla((P_FR + Dx_mat)', lla0, 'flat');
  //  P_RR_dx = enu2lla((P_RR + Dx_mat)', lla0, 'flat');
  //  P_FL_dx = enu2lla((P_FL + Dx_mat)', lla0, 'flat');
  //  P_RL_dx = enu2lla((P_RL + Dx_mat)', lla0, 'flat');
  //
  //  % compute the positions in LLA +/- dy distance from the wheel positions
  //  Dy_mat = [0 0;dy -dy;0 0];
  //  P_FR_dy = enu2lla((P_FR + Dy_mat)', lla0, 'flat');
  //  P_RR_dy = enu2lla((P_RR + Dy_mat)', lla0, 'flat');
  //  P_FL_dy = enu2lla((P_FL + Dy_mat)', lla0, 'flat');
  //  P_RL_dy = enu2lla((P_RL + Dy_mat)', lla0, 'flat');
  //  Intermediate results from code ceval has to be known beforehand
  //  lla calculations
  for (int b_i{0}; b_i < 3; b_i++) {
    d = C_tmp[b_i];
    d1 = C_tmp[b_i + 3];
    i = static_cast<int>(C_tmp[b_i + 6]);
    for (i1 = 0; i1 < 3; i1++) {
      b_C_tmp[b_i + 3 * i1] = (d * PR_tmp[3 * i1] + d1 * PR_tmp[3 * i1 + 1]) +
                              static_cast<double>(i) * PR_tmp[3 * i1 + 2];
    }
    d = b_C_tmp[b_i];
    d1 = b_C_tmp[b_i + 3];
    d2 = b_C_tmp[b_i + 6];
    d3 = 0.0;
    d4 = 0.0;
    d5 = 0.0;
    d6 = 0.0;
    d7 = 0.0;
    for (i = 0; i < 3; i++) {
      d8 = (d * b_PR_tmp[3 * i] + d1 * b_PR_tmp[3 * i + 1]) +
           d2 * b_PR_tmp[3 * i + 2];
      C[b_i + 3 * i] = d8;
      d3 += d8 * b_xk[i];
      d4 += d8 * pFL[i];
      d5 += d8 * pRL[i];
      d6 += d8 * pFR[i];
      d7 += d8 * pRR[i];
    }
    RpRR[b_i] = d7;
    RpFR[b_i] = d6;
    RpRL[b_i] = d5;
    RpFL[b_i] = d4;
    dotENU[b_i] = d3;
    d = P_CG[b_i];
    d1 = d + d6;
    b_P_FR[b_i] = d1;
    P_RR[b_i] = d + d7;
    P_FL[b_i] = d + d4;
    P_RL[b_i] = d + d5;
    k = b_i << 1;
    P_FR[k] = d1 + Dx_mat[b_i];
    P_FR[k + 1] = d1 + Dx_mat[b_i + 3];
  }
  coder::enu2lla(b_P_FR, lla0, P_FR_LLA);
  coder::enu2lla(P_RR, lla0, P_RR_LLA);
  coder::enu2lla(P_FL, lla0, P_FL_LLA);
  coder::enu2lla(P_RL, lla0, P_RL_LLA);
  coder::b_enu2lla(P_FR, lla0, P_FR_dx);
  for (i = 0; i < 3; i++) {
    k = i << 1;
    d = P_RR[i];
    P_FR[k] = d + Dx_mat[i];
    P_FR[k + 1] = d + Dx_mat[i + 3];
  }
  coder::b_enu2lla(P_FR, lla0, P_RR_dx);
  for (i = 0; i < 3; i++) {
    k = i << 1;
    d = P_FL[i];
    P_FR[k] = d + Dx_mat[i];
    P_FR[k + 1] = d + Dx_mat[i + 3];
  }
  coder::b_enu2lla(P_FR, lla0, P_FL_dx);
  for (i = 0; i < 3; i++) {
    k = i << 1;
    d = P_RL[i];
    P_FR[k] = d + Dx_mat[i];
    P_FR[k + 1] = d + Dx_mat[i + 3];
  }
  coder::b_enu2lla(P_FR, lla0, P_RL_dx);
  for (i = 0; i < 3; i++) {
    k = i << 1;
    d = b_P_FR[i];
    P_FR[k] = d + Dy_mat[i];
    P_FR[k + 1] = d + Dy_mat[i + 3];
  }
  coder::b_enu2lla(P_FR, lla0, P_FR_dy);
  for (i = 0; i < 3; i++) {
    k = i << 1;
    d = P_RR[i];
    P_FR[k] = d + Dy_mat[i];
    P_FR[k + 1] = d + Dy_mat[i + 3];
  }
  coder::b_enu2lla(P_FR, lla0, P_RR_dy);
  for (i = 0; i < 3; i++) {
    k = i << 1;
    d = P_FL[i];
    P_FR[k] = d + Dy_mat[i];
    P_FR[k + 1] = d + Dy_mat[i + 3];
  }
  coder::b_enu2lla(P_FR, lla0, P_FL_dy);
  for (i = 0; i < 3; i++) {
    k = i << 1;
    d = P_RL[i];
    P_FR[k] = d + Dy_mat[i];
    P_FR[k + 1] = d + Dy_mat[i + 3];
  }
  double P_RL_dy[6];
  double dfxFL1;
  double dfxFL2;
  double dfxFR1;
  double dfxFR2;
  double dfxRL1;
  double dfxRL2;
  double dfxRR1;
  double dfxRR2;
  double dfyFL1;
  double dfyFL2;
  double dfyFR1;
  double dfyFR2;
  double dfyRL1;
  double dfyRL2;
  double dfyRR1;
  double dfyRR2;
  coder::b_enu2lla(P_FR, lla0, P_RL_dy);
  fFR = int_at(P_FR_LLA[1], P_FR_LLA[0]);
  fRR = int_at(P_RR_LLA[1], P_RR_LLA[0]);
  fFL = int_at(P_FL_LLA[1], P_FL_LLA[0]);
  fRL = int_at(P_RL_LLA[1], P_RL_LLA[0]);
  dfxFR1 = int_at(P_FR_dx[2], P_FR_dx[0]);
  dfxFR2 = int_at(P_FR_dx[3], P_FR_dx[1]);
  dfxFL1 = int_at(P_FL_dx[2], P_FL_dx[0]);
  dfxFL2 = int_at(P_FL_dx[3], P_FL_dx[1]);
  dfxRR1 = int_at(P_RR_dx[2], P_RR_dx[0]);
  dfxRR2 = int_at(P_RR_dx[3], P_RR_dx[1]);
  dfxRL1 = int_at(P_RL_dx[2], P_RL_dx[0]);
  dfxRL2 = int_at(P_RL_dx[3], P_RL_dx[1]);
  // 1/(2*dx)*(coder.ceval("int_at", P_FR_dx(1,2), P_FR_dx(1,1)) -
  // coder.ceval("int_at", P_FR_dx(2,2), P_FR_dx(2,1)));
  // 1/(2*dx)*(coder.ceval("int_at", P_FL_dx(1,2), P_FL_dx(1,1)) -
  // coder.ceval("int_at", P_FL_dx(2,2), P_FL_dx(2,1)));
  // 1/(2*dx)*(coder.ceval("int_at", P_RR_dx(1,2), P_RR_dx(1,1)) -
  // coder.ceval("int_at", P_RR_dx(2,2), P_RR_dx(2,1)));
  dfyFR1 = int_at(P_FR_dy[2], P_FR_dy[0]);
  dfyFR2 = int_at(P_FR_dy[3], P_FR_dy[1]);
  dfyFL1 = int_at(P_FL_dy[2], P_FL_dy[0]);
  dfyFL2 = int_at(P_FL_dy[3], P_FL_dy[1]);
  dfyRR1 = int_at(P_RR_dy[2], P_RR_dy[0]);
  dfyRR2 = int_at(P_RR_dy[3], P_RR_dy[1]);
  dfyRL1 = int_at(P_RL_dy[2], P_RL_dy[0]);
  dfyRL2 = int_at(P_RL_dy[3], P_RL_dy[1]);
  // compute derivative of map(x,y) beneath k-th wheel with respect to y
  // (coder.ceval("int_at", P_FR_dy(1,2), P_FR_dy(1,1)) - coder.ceval("int_at",
  // P_FR_dy(2,2), P_FR_dy(2,1))); (coder.ceval("int_at", P_FL_dy(1,2),
  // P_FL_dy(1,1)) - coder.ceval("int_at", P_FL_dy(2,2), P_FL_dy(2,1)));
  // (coder.ceval("int_at", P_RR_dy(1,2), P_RR_dy(1,1)) - coder.ceval("int_at",
  // P_RR_dy(2,2), P_RR_dy(2,1)));
  // %%% ENU calculations
  //   Dx_mat = [dx -dx;0 0;0 0];
  //  P_FR_dx = P_FR + Dx_mat;
  //  P_RR_dx = P_RR + Dx_mat;
  //  P_FL_dx = P_FL + Dx_mat;
  //  P_RL_dx = P_RL + Dx_mat;
  //
  //  % compute the positions in LLA +/- dy distance from the wheel positions
  //  Dy_mat = [0 0;dy -dy;0 0];
  //  P_FR_dy = P_FR + Dy_mat;
  //  P_RR_dy = P_RR + Dy_mat;
  //  P_FL_dy = P_FL + Dy_mat;
  //  P_RL_dy = P_RL + Dy_mat;
  //  % fFR = coder.ceval("int_at", P_FR_LLA(2), P_FR_LLA(1));
  //  % fRR = coder.ceval("int_at", P_RR_LLA(2), P_RR_LLA(1));
  //  % fFL = coder.ceval("int_at", P_FL_LLA(2), P_FL_LLA(1));
  //  % fRL = coder.ceval("int_at", P_RL_LLA(2), P_RL_LLA(1));
  //  fFR = coder.ceval("int_at", P_FR(1), P_FR(2));
  //  fRR = coder.ceval("int_at", P_RR(1), P_RR(2));
  //  fFL = coder.ceval("int_at", P_FL(1), P_FL(2));
  //  fRL = coder.ceval("int_at", P_RL(1), P_RL(2));
  //
  //    % compute derivative of map(x,y) beneath k-th wheel with respect to x
  //  dfxFR1 = coder.ceval("int_at", P_FR_dx(1,1), P_FR_dx(1,2));
  //  dfxFR2 = coder.ceval("int_at", P_FR_dx(2,1), P_FR_dx(2,2));
  //  dfxFL1 = coder.ceval("int_at", P_FL_dx(1,1), P_FL_dx(1,2));
  //  dfxFL2 = coder.ceval("int_at", P_FL_dx(2,1), P_FL_dx(2,2));
  //  dfxRR1 = coder.ceval("int_at", P_RR_dx(1,1), P_RR_dx(1,2));
  //  dfxRR2 = coder.ceval("int_at", P_RR_dx(2,1), P_RR_dx(2,2));
  //  dfxRL1 = coder.ceval("int_at", P_RL_dx(1,1), P_RL_dx(1,2));
  //  dfxRL2 = coder.ceval("int_at", P_RL_dx(2,1), P_RL_dx(2,2));
  //
  //  dfxFR = 1/(2*dx)*(dfxFR1-dfxFR2);%1/(2*dx)*(coder.ceval("int_at",
  //  P_FR_dx(1,2), P_FR_dx(1,1)) - coder.ceval("int_at", P_FR_dx(2,2),
  //  P_FR_dx(2,1))); dfxFL =
  //  1/(2*dx)*(dfxFL1-dfxFL2);%1/(2*dx)*(coder.ceval("int_at", P_FL_dx(1,2),
  //  P_FL_dx(1,1)) - coder.ceval("int_at", P_FL_dx(2,2), P_FL_dx(2,1))); dfxRR
  //  = 1/(2*dx)*(dfxRR1-dfxRR2);%1/(2*dx)*(coder.ceval("int_at", P_RR_dx(1,2),
  //  P_RR_dx(1,1)) - coder.ceval("int_at", P_RR_dx(2,2), P_RR_dx(2,1))); dfxRL
  //  = 1/(2*dx)*(dfxRL1-dfxRL2);%1/(2*dx)*(coder.ceval("int_at", P_RL_dx(1,2),
  //  P_RL_dx(1,1)) - coder.ceval("int_at", P_RL_dx(2,2), P_RL_dx(2,1)));
  //
  //  dfyFR1 = coder.ceval("int_at", P_FR_dy(1,1), P_FR_dy(1,2));
  //  dfyFR2 = coder.ceval("int_at", P_FR_dy(2,1), P_FR_dy(2,2));
  //  dfyFL1 = coder.ceval("int_at", P_FL_dy(1,1), P_FL_dy(1,2));
  //  dfyFL2 = coder.ceval("int_at", P_FL_dy(2,1), P_FL_dy(2,2));
  //  dfyRR1 = coder.ceval("int_at", P_RR_dy(1,1), P_RR_dy(1,2));
  //  dfyRR2 = coder.ceval("int_at", P_RR_dy(2,1), P_RR_dy(2,2));
  //  dfyRL1 = coder.ceval("int_at", P_RL_dy(1,1), P_RL_dy(1,2));
  //  dfyRL2 = coder.ceval("int_at", P_RL_dy(2,1), P_RL_dy(2,2));
  // compute derivative of map(x,y) beneath k-th wheel with respect to y
  // dfyFR = 1/(2*dy)*(dfyFR1-dfyFR2);%(coder.ceval("int_at", P_FR_dy(1,2),
  // P_FR_dy(1,1)) - coder.ceval("int_at", P_FR_dy(2,2), P_FR_dy(2,1))); dfyFL =
  // 1/(2*dy)*(dfyFL1-dfyFL2);%(coder.ceval("int_at", P_FL_dy(1,2),
  // P_FL_dy(1,1)) - coder.ceval("int_at", P_FL_dy(2,2), P_FL_dy(2,1))); dfyRR =
  // 1/(2*dy)*(dfyRR1-dfyRR2);%(coder.ceval("int_at", P_RR_dy(1,2),
  // P_RR_dy(1,1)) - coder.ceval("int_at", P_RR_dy(2,2), P_RR_dy(2,1))); dfyRL =
  // 1/(2*dy)*(dfyRL1-dfyRL2);%(coder.ceval("int_at", P_RL_dy(1,2),
  // P_RL_dy(1,1)) - coder.ceval("int_at", P_RL_dy(2,2), P_RL_dy(2,1)));
  //  define the spatial gradient of the elevation
  grad_FR[0] = 3.3333333333333335 * (dfxFR1 - dfxFR2);
  grad_FR[1] = 2.0 * (dfyFR1 - dfyFR2);
  grad_FR[2] = 0.0;
  grad_FL[0] = 3.3333333333333335 * (dfxFL1 - dfxFL2);
  grad_FL[1] = 2.0 * (dfyFL1 - dfyFL2);
  grad_FL[2] = 0.0;
  grad_RR[0] = 3.3333333333333335 * (dfxRR1 - dfxRR2);
  grad_RR[1] = 2.0 * (dfyRR1 - dfyRR2);
  grad_RR[2] = 0.0;
  grad_RL[0] = 3.3333333333333335 * (dfxRL1 - dfxRL2);
  grad_RL[1] = 2.0 * (dfyRL1 - dfyRL2);
  grad_RL[2] = 0.0;
  //  implement gimbal equations (don't change the sequence)
  dyaw = (xk[10] * c_varargout_1_tmp_tmp + xk[11] * d_varargout_1_tmp_tmp) /
         b_varargout_1_tmp_tmp;
  dpitch = xk[10] * d_varargout_1_tmp_tmp - xk[11] * c_varargout_1_tmp_tmp;
  droll = dyaw * varargout_1_tmp_tmp + xk[9];
  //  define Euler transformation matrix and
  //  its derivative components
  d = -e_varargout_1_tmp_tmp * dyaw;
  b_dv[0] = d;
  b_dv[3] = -f_varargout_1_tmp_tmp * dyaw;
  b_dv[6] = 0.0 * dyaw;
  b_dv[1] = f_varargout_1_tmp_tmp * dyaw;
  b_dv[4] = d;
  b_dv[7] = 0.0 * dyaw;
  b_dv[2] = 0.0 * dyaw;
  b_dv[5] = 0.0 * dyaw;
  b_dv[8] = 0.0 * dyaw;
  for (i = 0; i < 3; i++) {
    d = b_dv[i];
    d1 = b_dv[i + 3];
    d2 = b_dv[i + 6];
    for (i1 = 0; i1 < 3; i1++) {
      dv1[i + 3 * i1] = (d * PR_tmp[3 * i1] + d1 * PR_tmp[3 * i1 + 1]) +
                        d2 * PR_tmp[3 * i1 + 2];
    }
  }
  d = -varargout_1_tmp_tmp * dpitch;
  b_dv[0] = d;
  b_dv[3] = 0.0 * dpitch;
  b_dv[6] = b_varargout_1_tmp_tmp * dpitch;
  b_dv[1] = 0.0 * dpitch;
  b_dv[4] = 0.0 * dpitch;
  b_dv[7] = 0.0 * dpitch;
  b_dv[2] = -b_varargout_1_tmp_tmp * dpitch;
  b_dv[5] = 0.0 * dpitch;
  b_dv[8] = d;
  for (i = 0; i < 3; i++) {
    d = C_tmp[i];
    d1 = C_tmp[i + 3];
    i1 = static_cast<int>(C_tmp[i + 6]);
    for (k = 0; k < 3; k++) {
      c_C_tmp[i + 3 * k] = (d * b_dv[3 * k] + d1 * b_dv[3 * k + 1]) +
                           static_cast<double>(i1) * b_dv[3 * k + 2];
    }
  }
  for (i = 0; i < 3; i++) {
    d = dv1[i];
    d1 = dv1[i + 3];
    d2 = dv1[i + 6];
    d3 = c_C_tmp[i];
    d4 = c_C_tmp[i + 3];
    d5 = c_C_tmp[i + 6];
    for (i1 = 0; i1 < 3; i1++) {
      d6 = b_PR_tmp[3 * i1];
      d7 = d * d6;
      d8 = d3 * d6;
      d6 = b_PR_tmp[3 * i1 + 1];
      d7 += d1 * d6;
      d8 += d4 * d6;
      d6 = b_PR_tmp[3 * i1 + 2];
      d7 += d2 * d6;
      d8 += d5 * d6;
      k = i + 3 * i1;
      C_tmp[k] = d8;
      b_dv[k] = d7;
    }
  }
  dv1[0] = 0.0 * droll;
  dv1[3] = 0.0 * droll;
  dv1[6] = 0.0 * droll;
  dv1[1] = 0.0 * droll;
  d = -c_varargout_1_tmp_tmp * droll;
  dv1[4] = d;
  dv1[7] = -d_varargout_1_tmp_tmp * droll;
  dv1[2] = 0.0 * droll;
  dv1[5] = d_varargout_1_tmp_tmp * droll;
  dv1[8] = d;
  //  derivatives of moment arms in global coordinates
  //  To compute the deflection of each spring first compute
  //  the HEIGHT of each strut-mount point with respect to
  //  the wheel
  //  the velocity of the strut mount point in ground frame
  //
  //  spring deflections -- output vector
  //  FR side
  // min([delta_x_FR, 0]);%
  //  FL side
  // min([delta_x_FL, 0]);%
  //  RR side
  // min([delta_x_RR, 0]);%
  //  RL side
  // min([delta_x_RL, 0]);%
  //  rate of change of spring deflections with respect to gradient
  //  of the ground profile
  //  ( dfxFR * dE + dfyFR * dN );
  //  ( dfxFL * dE + dfyFL * dN );
  //  ( dfxRR * dE + dfyRR * dN );
  //  ( dfxRL * dE + dfyRL * dN );
  //  compute normal force at each corner
  b_varargout_1_tmp_tmp = 0.0;
  //  + Fcmd(3)/4;
  // max([FztFR, 0]);
  c_varargout_1_tmp_tmp = 0.0;
  for (k = 0; k < 3; k++) {
    d = dotENU[k];
    b_varargout_1_tmp_tmp += grad_FR[k] * d;
    d1 = b_C_tmp[k];
    d2 = b_C_tmp[k + 3];
    d3 = b_C_tmp[k + 6];
    d4 = 0.0;
    for (i = 0; i < 3; i++) {
      i1 = k + 3 * i;
      d5 = (b_dv[i1] + C_tmp[i1]) +
           ((d1 * dv1[3 * i] + d2 * dv1[3 * i + 1]) + d3 * dv1[3 * i + 2]);
      RTemp[i1] = d5;
      d4 += d5 * pFR[i];
    }
    b_xk[k] = d4;
    c_varargout_1_tmp_tmp += grad_FL[k] * d;
  }
  varargout_1_tmp_tmp =
      std::log(std::exp(100.0 * -((xk[2] + RpFR[2]) - fFR)) + 1.0) / 100.0;
  FztFR = -kFR * -varargout_1_tmp_tmp -
          cFR * ((b_xk[2] + xk[5]) - b_varargout_1_tmp_tmp);
  //  + Fcmd(3)/4;
  // max([FztFL, 0]);
  b_varargout_1_tmp_tmp = 0.0;
  d = pFL[0];
  d1 = pFL[1];
  d2 = pFL[2];
  for (k = 0; k < 3; k++) {
    b_xk[k] = (RTemp[k] * d + RTemp[k + 3] * d1) + RTemp[k + 6] * d2;
    b_varargout_1_tmp_tmp += grad_RR[k] * dotENU[k];
  }
  FztFL_tmp =
      std::log(std::exp(100.0 * -((xk[2] + RpFL[2]) - fFL)) + 1.0) / 100.0;
  FztFL = -kFL * -FztFL_tmp - cFL * ((b_xk[2] + xk[5]) - c_varargout_1_tmp_tmp);
  //  + Fcmd(3)/4;
  // max([FztRR, 0]);
  c_varargout_1_tmp_tmp = 0.0;
  d = pRR[0];
  d1 = pRR[1];
  d2 = pRR[2];
  for (k = 0; k < 3; k++) {
    b_xk[k] = (RTemp[k] * d + RTemp[k + 3] * d1) + RTemp[k + 6] * d2;
    c_varargout_1_tmp_tmp += grad_RL[k] * dotENU[k];
  }
  FztRR_tmp =
      std::log(std::exp(100.0 * -((xk[2] + RpRR[2]) - fRR)) + 1.0) / 100.0;
  FztRR = -kRR * -FztRR_tmp - cRR * ((b_xk[2] + xk[5]) - b_varargout_1_tmp_tmp);
  d = pRL[0];
  d1 = pRL[1];
  d2 = pRL[2];
  for (i = 0; i < 3; i++) {
    b_xk[i] = (RTemp[i] * d + RTemp[i + 3] * d1) + RTemp[i + 6] * d2;
  }
  FztRL_tmp =
      std::log(std::exp(100.0 * -((xk[2] + RpRL[2]) - fRL)) + 1.0) / 100.0;
  FztRL = -kRL * -FztRL_tmp - cRL * ((b_xk[2] + xk[5]) - c_varargout_1_tmp_tmp);
  //  + Fcmd(3)/4;
  // max([FztRL, 0]);
  //  Velocities for strut positions in vehicle coordinates
  PR_tmp[1] = 0.0 * xk[9];
  PR_tmp[4] = -0.0 * xk[9];
  PR_tmp[7] = -xk[9];
  PR_tmp[2] = 0.0 * xk[9];
  PR_tmp[5] = xk[9];
  PR_tmp[8] = -0.0 * xk[9];
  b_dv[0] = -0.0 * xk[10];
  b_dv[3] = 0.0 * xk[10];
  b_dv[6] = xk[10];
  b_dv[2] = -xk[10];
  b_dv[5] = 0.0 * xk[10];
  b_dv[8] = -0.0 * xk[10];
  dv1[0] = -0.0 * xk[11];
  dv1[3] = -xk[11];
  dv1[6] = 0.0 * xk[11];
  dv1[1] = xk[11];
  dv1[4] = -0.0 * xk[11];
  dv1[7] = 0.0 * xk[11];
  PR_tmp[0] = 0.0 * xk[9];
  b_dv[1] = 0.0 * xk[10];
  dv1[2] = 0.0 * xk[11];
  PR_tmp[3] = 0.0 * xk[9];
  b_dv[4] = 0.0 * xk[10];
  dv1[5] = 0.0 * xk[11];
  PR_tmp[6] = 0.0 * xk[9];
  b_dv[7] = 0.0 * xk[10];
  dv1[8] = 0.0 * xk[11];
  for (i = 0; i < 9; i++) {
    PR_tmp[i] = (PR_tmp[i] + b_dv[i]) + dv1[i];
  }
  //  velocities of strut points due to additional moment induced by
  //  the instantaneous deflection of the spring above tire radius height
  b_xk[2] = varargout_1_tmp_tmp - h_T;
  d = pFR[0];
  d1 = pFR[1];
  d2 = pFR[2];
  d3 = pFL[0];
  d4 = pFL[1];
  d5 = pFL[2];
  d6 = pRR[0];
  d7 = pRR[1];
  d8 = pRR[2];
  varargout_1_tmp_tmp = pRL[0];
  b_varargout_1_tmp_tmp = pRL[1];
  c_varargout_1_tmp_tmp = pRL[2];
  for (i = 0; i < 3; i++) {
    double d9;
    varargout_1_tmp = PR_tmp[i];
    d_varargout_1_tmp_tmp = varargout_1_tmp * d;
    e_varargout_1_tmp_tmp = varargout_1_tmp * d3;
    f_varargout_1_tmp_tmp = varargout_1_tmp * d6;
    d9 = varargout_1_tmp * varargout_1_tmp_tmp;
    varargout_1_tmp = PR_tmp[i + 3];
    d_varargout_1_tmp_tmp += varargout_1_tmp * d1;
    e_varargout_1_tmp_tmp += varargout_1_tmp * d4;
    f_varargout_1_tmp_tmp += varargout_1_tmp * d7;
    d9 += varargout_1_tmp * b_varargout_1_tmp_tmp;
    varargout_1_tmp = PR_tmp[i + 6];
    d_varargout_1_tmp_tmp += varargout_1_tmp * d2;
    e_varargout_1_tmp_tmp += varargout_1_tmp * d5;
    f_varargout_1_tmp_tmp += varargout_1_tmp * d8;
    d9 += varargout_1_tmp * c_varargout_1_tmp_tmp;
    VuFR_tmp[i] = (RP[i] * 0.0 + RP[i + 3] * 0.0) + RP[i + 6] * b_xk[2];
    d_y[i] = d9;
    c_y[i] = f_varargout_1_tmp_tmp;
    b_y[i] = e_varargout_1_tmp_tmp;
    y[i] = d_varargout_1_tmp_tmp;
  }
  b_xk[2] = FztFL_tmp - h_T;
  for (i = 0; i < 3; i++) {
    VuFL_tmp[i] = (RP[i] * 0.0 + RP[i + 3] * 0.0) + RP[i + 6] * b_xk[2];
  }
  b_xk[2] = FztRR_tmp - h_T;
  for (i = 0; i < 3; i++) {
    VuRR_tmp[i] = (RP[i] * 0.0 + RP[i + 3] * 0.0) + RP[i + 6] * b_xk[2];
  }
  b_xk[2] = FztRL_tmp - h_T;
  for (i = 0; i < 3; i++) {
    VuRL_tmp[i] = (RP[i] * 0.0 + RP[i + 3] * 0.0) + RP[i + 6] * b_xk[2];
  }
  //  Velocities of each strut in tire-ground contact patches
  varargout_1_tmp_tmp =
      (VuFR_tmp[1] * xk[11] - VuFR_tmp[2] * xk[10]) + (y[0] + xk[3]);
  b_varargout_1_tmp_tmp =
      (VuFR_tmp[2] * xk[9] - VuFR_tmp[0] * xk[11]) + (y[1] + xk[4]);
  c_varargout_1_tmp_tmp =
      (VuFR_tmp[0] * xk[10] - VuFR_tmp[1] * xk[9]) + (y[2] + xk[5]);
  b_xk[0] = VuFL_tmp[1] * xk[11] - VuFL_tmp[2] * xk[10];
  b_xk[1] = VuFL_tmp[2] * xk[9] - VuFL_tmp[0] * xk[11];
  b_xk[2] = VuFL_tmp[0] * xk[10] - VuFL_tmp[1] * xk[9];
  y[0] = b_y[0] + xk[3];
  y[1] = b_y[1] + xk[4];
  y[2] = b_y[2] + xk[5];
  for (i = 0; i < 3; i++) {
    VgFR[i] =
        (PR[i] * varargout_1_tmp_tmp + PR[i + 3] * b_varargout_1_tmp_tmp) +
        PR[i + 6] * c_varargout_1_tmp_tmp;
    b_xk[i] += y[i];
  }
  d = b_xk[0];
  d1 = b_xk[1];
  d2 = b_xk[2];
  for (i = 0; i < 3; i++) {
    VgFL[i] = (PR[i] * d + PR[i + 3] * d1) + PR[i + 6] * d2;
  }
  d = (VuRR_tmp[1] * xk[11] - VuRR_tmp[2] * xk[10]) + (c_y[0] + xk[3]);
  d1 = (VuRR_tmp[2] * xk[9] - VuRR_tmp[0] * xk[11]) + (c_y[1] + xk[4]);
  d2 = (VuRR_tmp[0] * xk[10] - VuRR_tmp[1] * xk[9]) + (c_y[2] + xk[5]);
  for (i = 0; i < 3; i++) {
    VgRR[i] = (PR[i] * d + PR[i + 3] * d1) + PR[i + 6] * d2;
  }
  d = (VuRL_tmp[1] * xk[11] - VuRL_tmp[2] * xk[10]) + (d_y[0] + xk[3]);
  d1 = (VuRL_tmp[2] * xk[9] - VuRL_tmp[0] * xk[11]) + (d_y[1] + xk[4]);
  d2 = (VuRL_tmp[0] * xk[10] - VuRL_tmp[1] * xk[9]) + (d_y[2] + xk[5]);
  for (i = 0; i < 3; i++) {
    VgRL[i] = (PR[i] * d + PR[i + 3] * d1) + PR[i + 6] * d2;
  }
  //  compute the wheel-ground contact patch velocities
  //  cornering force (lateral) of each wheel
  //  the problem with this method is the curvature becomes the inverse
  //  function of speed -- so kind of interdependancy of curvature and
  //  speed
  //  compute curvature commands for left and right wheel
  //  compute left and right wheel steering angles
  varargout_1_tmp_tmp = tw / 2.0 * xk[12];
  stgL = std::atan(l * (xk[12] / (1.0 - varargout_1_tmp_tmp)));
  stgR = std::atan(l * (xk[12] / (varargout_1_tmp_tmp + 1.0)));
  //  Slip angles of each wheel
  //  to limit the side-slip angles to linear range for lateral force
  //  versus side-slip angles curve
  // lim_val(AlphaFR, -10.0*D2R, 10.0*D2R);
  // lim_val(AlphaFL, -10.0*D2R, 10.0*D2R);
  // lim_val(AlphaRR, -10.0*D2R, 10.0*D2R);
  // lim_val(AlphaRL, -10.0*D2R, 10.0*D2R);
  //  compute friction forces on each wheel
  mur = m * uk[1] / (((FztRL + FztFL) + FztRR) + FztFR);
  // Fcmd(1)/FzT; % to avoid mu -> Inf -- TBD
  if (std::abs(uk[2]) > 0.0) {
    //  Compute lateral forces as a function of cornering stiffness
    //  coefficient
    varargout_1_tmp_tmp = xk[13] * Ca;
    FcFR = varargout_1_tmp_tmp *
           (-std::log(
                std::exp(-std::log(
                    std::exp(10.0 * ((-rt_atan2d_snf(VgFR[1], VgFR[0]) + stgR) *
                                     R2D)) +
                    3.7200759760208361E-44)) +
                3.7200759760208361E-44) /
            10.0 * D2R);
    FcFL = varargout_1_tmp_tmp *
           (-std::log(
                std::exp(-std::log(
                    std::exp(10.0 * ((-rt_atan2d_snf(VgFL[1], VgFL[0]) + stgL) *
                                     R2D)) +
                    3.7200759760208361E-44)) +
                3.7200759760208361E-44) /
            10.0 * D2R);
    FcRR = varargout_1_tmp_tmp *
           (-std::log(
                std::exp(-std::log(
                    std::exp(10.0 * (-rt_atan2d_snf(VgRR[1], VgRR[0]) * R2D)) +
                    3.7200759760208361E-44)) +
                3.7200759760208361E-44) /
            10.0 * D2R);
    FcRL = varargout_1_tmp_tmp *
           (-std::log(
                std::exp(-std::log(
                    std::exp(10.0 * (-rt_atan2d_snf(VgRL[1], VgRL[0]) * R2D)) +
                    3.7200759760208361E-44)) +
                3.7200759760208361E-44) /
            10.0 * D2R);
    //  compute the rolling resistance force in TGC-patch frame
    //  total rolling resistance force
    //  In reality, it should be distributed on each wheel by mass.
    if (std::isnan(xk[3])) {
      d = rtNaN;
    } else if (xk[3] < 0.0) {
      d = -1.0;
    } else {
      d = (xk[3] > 0.0);
    }
    varargout_1_tmp_tmp = -m * g * Cr * d / 4.0;
    FrFL = varargout_1_tmp_tmp;
    FrRL = varargout_1_tmp_tmp;
    FrFR = varargout_1_tmp_tmp;
    FrRR = varargout_1_tmp_tmp;
    //  compute the road inclination for front and rear wheels in TGC frame
    varargout_1_tmp_tmp = std::sin(-xk[8]);
    b_varargout_1_tmp_tmp = std::cos(-xk[8]);
    PR_tmp[0] = b_varargout_1_tmp_tmp;
    PR_tmp[3] = -varargout_1_tmp_tmp;
    PR_tmp[6] = 0.0;
    PR_tmp[1] = varargout_1_tmp_tmp;
    PR_tmp[4] = b_varargout_1_tmp_tmp;
    PR_tmp[7] = 0.0;
    PR_tmp[2] = 0.0;
    PR_tmp[5] = 0.0;
    PR_tmp[8] = 1.0;
    d = grad_FR[0];
    d1 = grad_FR[1];
    for (i = 0; i < 3; i++) {
      b_xk[i] = PR_tmp[i] * d + PR_tmp[i + 3] * d1;
    }
    gamma_FR = std::atan(b_xk[0]);
    d = grad_FL[0];
    d1 = grad_FL[1];
    for (i = 0; i < 3; i++) {
      b_xk[i] = PR_tmp[i] * d + PR_tmp[i + 3] * d1;
    }
    gamma_FL = std::atan(b_xk[0]);
    d = grad_RR[0];
    d1 = grad_RR[1];
    for (i = 0; i < 3; i++) {
      b_xk[i] = PR_tmp[i] * d + PR_tmp[i + 3] * d1;
    }
    gamma_RR = std::atan(b_xk[0]);
    d = grad_RL[0];
    d1 = grad_RL[1];
    for (i = 0; i < 3; i++) {
      b_xk[i] = PR_tmp[i] * d + PR_tmp[i + 3] * d1;
    }
    gamma_RL = std::atan(b_xk[0]);
  } else {
    //  init = 0;
    //  to stop rolling back when no
    //  acceleration command is present as there
    //  are no brakes in Polaris
    FrFL = 0.0;
    FrRL = 0.0;
    FrFR = 0.0;
    FrRR = 0.0;
    FcFR = 0.0;
    FcFL = 0.0;
    FcRR = 0.0;
    FcRL = 0.0;
    //  road inclination for front and rear wheels -- TBD
    gamma_FR = 0.0;
    gamma_FL = 0.0;
    gamma_RR = 0.0;
    gamma_RL = 0.0;
  }
  //  compute wheel forces in body frame
  varargout_1_tmp_tmp = std::sin(stgR);
  b_varargout_1_tmp_tmp = std::cos(stgR);
  varargout_1_tmp = std::sin(-gamma_FR);
  c_varargout_1_tmp_tmp = std::cos(-gamma_FR);
  PR_tmp[0] = b_varargout_1_tmp_tmp;
  PR_tmp[3] = -varargout_1_tmp_tmp;
  PR_tmp[6] = 0.0;
  PR_tmp[1] = varargout_1_tmp_tmp;
  PR_tmp[4] = b_varargout_1_tmp_tmp;
  PR_tmp[7] = 0.0;
  PR_tmp[2] = 0.0;
  PR_tmp[5] = 0.0;
  PR_tmp[8] = 1.0;
  for (i = 0; i < 3; i++) {
    d = RP[i];
    d1 = RP[i + 3];
    d2 = RP[i + 6];
    for (i1 = 0; i1 < 3; i1++) {
      c_C_tmp[i + 3 * i1] = (d * PR_tmp[3 * i1] + d1 * PR_tmp[3 * i1 + 1]) +
                            d2 * PR_tmp[3 * i1 + 2];
    }
  }
  PR_tmp[0] = c_varargout_1_tmp_tmp;
  PR_tmp[3] = 0.0;
  PR_tmp[6] = varargout_1_tmp;
  PR_tmp[1] = 0.0;
  PR_tmp[4] = 1.0;
  PR_tmp[7] = 0.0;
  PR_tmp[2] = -varargout_1_tmp;
  PR_tmp[5] = 0.0;
  PR_tmp[8] = c_varargout_1_tmp_tmp;
  b_xk[0] = mur * FztFR + FrFR;
  b_xk[1] = FcFR;
  b_xk[2] = FztFR;
  for (i = 0; i < 3; i++) {
    d = 0.0;
    d1 = c_C_tmp[i];
    d2 = c_C_tmp[i + 3];
    d3 = c_C_tmp[i + 6];
    for (i1 = 0; i1 < 3; i1++) {
      d += ((d1 * PR_tmp[3 * i1] + d2 * PR_tmp[3 * i1 + 1]) +
            d3 * PR_tmp[3 * i1 + 2]) *
           b_xk[i1];
    }
    FwFR[i] = d;
  }
  varargout_1_tmp_tmp = std::sin(stgL);
  b_varargout_1_tmp_tmp = std::cos(stgL);
  varargout_1_tmp = std::sin(-gamma_FL);
  c_varargout_1_tmp_tmp = std::cos(-gamma_FL);
  PR_tmp[0] = b_varargout_1_tmp_tmp;
  PR_tmp[3] = -varargout_1_tmp_tmp;
  PR_tmp[6] = 0.0;
  PR_tmp[1] = varargout_1_tmp_tmp;
  PR_tmp[4] = b_varargout_1_tmp_tmp;
  PR_tmp[7] = 0.0;
  PR_tmp[2] = 0.0;
  PR_tmp[5] = 0.0;
  PR_tmp[8] = 1.0;
  for (i = 0; i < 3; i++) {
    d = RP[i];
    d1 = RP[i + 3];
    d2 = RP[i + 6];
    for (i1 = 0; i1 < 3; i1++) {
      c_C_tmp[i + 3 * i1] = (d * PR_tmp[3 * i1] + d1 * PR_tmp[3 * i1 + 1]) +
                            d2 * PR_tmp[3 * i1 + 2];
    }
  }
  PR_tmp[0] = c_varargout_1_tmp_tmp;
  PR_tmp[3] = 0.0;
  PR_tmp[6] = varargout_1_tmp;
  PR_tmp[1] = 0.0;
  PR_tmp[4] = 1.0;
  PR_tmp[7] = 0.0;
  PR_tmp[2] = -varargout_1_tmp;
  PR_tmp[5] = 0.0;
  PR_tmp[8] = c_varargout_1_tmp_tmp;
  b_xk[0] = mur * FztFL + FrFL;
  b_xk[1] = FcFL;
  b_xk[2] = FztFL;
  for (i = 0; i < 3; i++) {
    d = 0.0;
    d1 = c_C_tmp[i];
    d2 = c_C_tmp[i + 3];
    d3 = c_C_tmp[i + 6];
    for (i1 = 0; i1 < 3; i1++) {
      d += ((d1 * PR_tmp[3 * i1] + d2 * PR_tmp[3 * i1 + 1]) +
            d3 * PR_tmp[3 * i1 + 2]) *
           b_xk[i1];
    }
    FwFL[i] = d;
  }
  varargout_1_tmp_tmp = std::sin(-gamma_RR);
  b_varargout_1_tmp_tmp = std::cos(-gamma_RR);
  PR_tmp[0] = b_varargout_1_tmp_tmp;
  PR_tmp[3] = 0.0;
  PR_tmp[6] = varargout_1_tmp_tmp;
  PR_tmp[1] = 0.0;
  PR_tmp[4] = 1.0;
  PR_tmp[7] = 0.0;
  PR_tmp[2] = -varargout_1_tmp_tmp;
  PR_tmp[5] = 0.0;
  PR_tmp[8] = b_varargout_1_tmp_tmp;
  b_xk[0] = mur * FztRR + FrRR;
  b_xk[1] = FcRR;
  b_xk[2] = FztRR;
  for (i = 0; i < 3; i++) {
    d = 0.0;
    d1 = RP[i];
    d2 = RP[i + 3];
    d3 = RP[i + 6];
    for (i1 = 0; i1 < 3; i1++) {
      d += ((d1 * PR_tmp[3 * i1] + d2 * PR_tmp[3 * i1 + 1]) +
            d3 * PR_tmp[3 * i1 + 2]) *
           b_xk[i1];
    }
    FwRR[i] = d;
  }
  varargout_1_tmp_tmp = std::sin(-gamma_RL);
  b_varargout_1_tmp_tmp = std::cos(-gamma_RL);
  PR_tmp[0] = b_varargout_1_tmp_tmp;
  PR_tmp[3] = 0.0;
  PR_tmp[6] = varargout_1_tmp_tmp;
  PR_tmp[1] = 0.0;
  PR_tmp[4] = 1.0;
  PR_tmp[7] = 0.0;
  PR_tmp[2] = -varargout_1_tmp_tmp;
  PR_tmp[5] = 0.0;
  PR_tmp[8] = b_varargout_1_tmp_tmp;
  b_xk[0] = mur * FztRL + FrRL;
  b_xk[1] = FcRL;
  b_xk[2] = FztRL;
  //  compute moments in body frame
  //
  for (i = 0; i < 3; i++) {
    d = 0.0;
    d1 = RP[i];
    d2 = RP[i + 3];
    d3 = RP[i + 6];
    for (i1 = 0; i1 < 3; i1++) {
      d += ((d1 * PR_tmp[3 * i1] + d2 * PR_tmp[3 * i1 + 1]) +
            d3 * PR_tmp[3 * i1 + 2]) *
           b_xk[i1];
    }
    FwRL[i] = d;
    VuFR_tmp[i] += pFR[i];
    VuFL_tmp[i] += pFL[i];
    VuRR_tmp[i] += pRR[i];
    VuRL_tmp[i] += pRL[i];
  }
  //
  //  Extract net forces for each wheel in vehicle frame
  //  Extract net moments
  //  total rolling moment
  //  total pitching moment
  //  total yawing moment
  //  G vector in inertial frame as
  for (i = 0; i < 3; i++) {
    G_vec[i] = (C[3 * i] * 0.0 + C[3 * i + 1] * 0.0) + C[3 * i + 2] * -g;
  }
  //  body linear accelerations
  //  forward acceleration
  //  lateral acceleration
  //  upwards acceleration
  //  body angular accelerations
  //  rolling acceelration
  //  pitch acceleration
  //  yaw acceleration
  //  equation of curvature dynamics
  //  extract steering/command from curvature state/command
  //  Update state vector
  d = str_a * (57.295779513082323 * std::atan(l * xk[12])) +
      str_b * (57.295779513082323 * std::atan(l * uk[0]));
  coder::b_tand(d);
  varargout_1[0] = dotENU[0];
  varargout_1[1] = dotENU[1];
  varargout_1[2] = dotENU[2];
  varargout_1[3] =
      (((((FwFR[0] + FwFL[0]) + FwRR[0]) + FwRL[0]) / m + G_vec[0]) -
       xk[5] * xk[10]) +
      xk[4] * xk[11];
  varargout_1[4] =
      (((((FwFR[1] + FwFL[1]) + FwRR[1]) + FwRL[1]) / m + G_vec[1]) +
       xk[5] * xk[9]) -
      xk[3] * xk[11];
  varargout_1[5] =
      (((((FwFR[2] + FwFL[2]) + FwRR[2]) + FwRL[2]) / m + G_vec[2]) -
       xk[4] * xk[9]) +
      xk[3] * xk[10];
  varargout_1[6] = droll;
  varargout_1[7] = dpitch;
  varargout_1[8] = dyaw;
  varargout_1[9] = (((((VuRR_tmp[1] * FwRR[2] - FwRR[1] * VuRR_tmp[2]) +
                       (VuRL_tmp[1] * FwRL[2] - FwRL[1] * VuRL_tmp[2])) +
                      (VuFR_tmp[1] * FwFR[2] - FwFR[1] * VuFR_tmp[2])) +
                     (VuFL_tmp[1] * FwFL[2] - FwFL[1] * VuFL_tmp[2])) -
                    xk[10] * xk[11] * (Izz - Iyy)) /
                   Ixx;
  varargout_1[10] = (((((FwRR[0] * VuRR_tmp[2] - VuRR_tmp[0] * FwRR[2]) +
                        (FwRL[0] * VuRL_tmp[2] - VuRL_tmp[0] * FwRL[2])) +
                       (FwFR[0] * VuFR_tmp[2] - VuFR_tmp[0] * FwFR[2])) +
                      (FwFL[0] * VuFL_tmp[2] - VuFL_tmp[0] * FwFL[2])) -
                     xk[9] * xk[11] * (Ixx - Izz)) /
                    Iyy;
  varargout_1[11] = (((((VuRR_tmp[0] * FwRR[1] - FwRR[0] * VuRR_tmp[1]) +
                        (VuRL_tmp[0] * FwRL[1] - FwRL[0] * VuRL_tmp[1])) +
                       (VuFR_tmp[0] * FwFR[1] - FwFR[0] * VuFR_tmp[1])) +
                      (VuFL_tmp[0] * FwFL[1] - FwFL[0] * VuFL_tmp[1])) -
                     xk[9] * xk[10] * (Iyy - Ixx)) /
                    Izz;
  varargout_1[12] = d / l;
  varargout_1[13] = 0.0;
  //  for curvature/steering dynamics
  //  for CCa/TGC
  //  0.0; % for Cr_state
  //  draw the vehicle
  //  FrontLoad = FzFR + FzFL;
  //  RearLoad = FzRR + FzRL;
  //  Vg_B = sqrt(u^2 + v^2 + w^2);
  //  y1 = xk(1:14);
  //  drawVehicle(y1,FwFR,FwFL,FwRR,FwRL,...
  //         Vg_B, Fx, Fz, m*G_vec, m*acmd, gamma_FR, ...
  //         Fcmd, mur, AlphaFR, AlphaRR, delta_x_FL, ...
  //         delta_x_RR, L, M, N, FrontLoad, RearLoad);
}

// End of code generation (aug_dynamic_model.cpp)
