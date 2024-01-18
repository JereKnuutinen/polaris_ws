//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// dynamic_model_nmpc.cpp
//
// Code generation for function 'dynamic_model_nmpc'
//

// Include files
#include "dynamic_model_nmpc.h"
#include "NMPC_Node_data.h"
#include "rt_nonfinite.h"
#include "rt_defines.h"
#include <cfloat>
#include <cmath>
#include <cstring>

// Function Declarations
static double rt_atan2d_snf(double u0, double u1);

static double rt_remd_snf(double u0, double u1);

// Function Definitions
static double rt_atan2d_snf(double u0, double u1)
{
  double y;
  if (std::isnan(u0) || std::isnan(u1)) {
    y = rtNaN;
  } else if (std::isinf(u0) && std::isinf(u1)) {
    int i;
    int i1;
    if (u0 > 0.0) {
      i = 1;
    } else {
      i = -1;
    }
    if (u1 > 0.0) {
      i1 = 1;
    } else {
      i1 = -1;
    }
    y = std::atan2(static_cast<double>(i), static_cast<double>(i1));
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = std::atan2(u0, u1);
  }
  return y;
}

static double rt_remd_snf(double u0, double u1)
{
  double y;
  if (std::isnan(u0) || std::isnan(u1) || std::isinf(u0)) {
    y = rtNaN;
  } else if (std::isinf(u1)) {
    y = u0;
  } else if ((u1 != 0.0) && (u1 != std::trunc(u1))) {
    double q;
    q = std::abs(u0 / u1);
    if (!(std::abs(q - std::floor(q + 0.5)) > DBL_EPSILON * q)) {
      y = 0.0 * u0;
    } else {
      y = std::fmod(u0, u1);
    }
  } else {
    y = std::fmod(u0, u1);
  }
  return y;
}

void dynamic_model_nmpc(const double xk[14], const double uk[2], double dxk[14])
{
  static const signed char b[9]{1, 0, 0, 0, 1, 0, 0, 0, 1};
  static const signed char iv1[9]{0, 0, 0, 0, 0, 1, 0, -1, 0};
  static const signed char iv2[9]{0, 0, -1, 0, 0, 0, 1, 0, 0};
  static const signed char iv3[9]{0, 1, 0, -1, 0, 0, 0, 0, 0};
  static const signed char b_iv[3]{0, 0, 1};
  double C[9];
  double C_tmp[9];
  double PR[9];
  double PR_tmp[9];
  double RP[9];
  double RTemp[9];
  double b_C_tmp[9];
  double b_PR_tmp[9];
  double dv[9];
  double dv1[9];
  double Fcmd[3];
  double FwFL[3];
  double FwFR[3];
  double MFR[3];
  double VgFL[3];
  double VgFR[3];
  double VgRL[3];
  double VgRR[3];
  double VuFL_tmp[3];
  double VuRL_tmp[3];
  double VuRR_tmp[3];
  double b_xk[3];
  double dotENU[3];
  double Fr;
  double FztFL;
  double FztFL_tmp;
  double FztFR;
  double FztRL;
  double FztRL_tmp;
  double FztRR;
  double FztRR_tmp;
  double absx;
  double b_varargout_1_tmp_tmp;
  double d;
  double d1;
  double d2;
  double d3;
  double d4;
  double d5;
  double d6;
  double d7;
  double d8;
  double dpitch;
  double droll;
  double dyaw;
  double mur;
  double stgL;
  double stgR;
  double varargout_1_tmp_tmp;
  double y;
  int C_tmp_tmp;
  int i1;
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
  //  rotation matrix around X-axis and its derivatives
  //  Rotation matrix around Y-axis and its derivatives
  //  Rotation matrix around Z-axis and its derivatives
  //  pre-compute joint rotation matrices
  absx = std::sin(xk[7]);
  varargout_1_tmp_tmp = std::cos(xk[7]);
  PR_tmp[0] = varargout_1_tmp_tmp;
  PR_tmp[3] = 0.0;
  PR_tmp[6] = absx;
  PR_tmp[2] = -absx;
  PR_tmp[5] = 0.0;
  PR_tmp[8] = varargout_1_tmp_tmp;
  Fr = std::sin(xk[6]);
  b_varargout_1_tmp_tmp = std::cos(xk[6]);
  PR_tmp[1] = 0.0;
  b_PR_tmp[0] = 1.0;
  PR_tmp[4] = 1.0;
  b_PR_tmp[3] = 0.0;
  PR_tmp[7] = 0.0;
  b_PR_tmp[6] = 0.0;
  b_PR_tmp[1] = 0.0;
  b_PR_tmp[4] = b_varargout_1_tmp_tmp;
  b_PR_tmp[7] = -Fr;
  b_PR_tmp[2] = 0.0;
  b_PR_tmp[5] = Fr;
  b_PR_tmp[8] = b_varargout_1_tmp_tmp;
  //  CG-fixed to TGC-patch frame
  //  TGC-patch to CG-fixed frame
  //  body to global (CG) transformation
  stgR = std::sin(xk[8]);
  mur = std::cos(xk[8]);
  C_tmp[0] = mur;
  C_tmp[3] = -stgR;
  C_tmp[6] = 0.0;
  C_tmp[1] = stgR;
  C_tmp[4] = mur;
  C_tmp[7] = 0.0;
  for (int i{0}; i < 3; i++) {
    d = PR_tmp[i];
    i1 = static_cast<int>(PR_tmp[i + 3]);
    d1 = PR_tmp[i + 6];
    for (C_tmp_tmp = 0; C_tmp_tmp < 3; C_tmp_tmp++) {
      d2 = (d * b_PR_tmp[3 * C_tmp_tmp] +
            static_cast<double>(i1) * b_PR_tmp[3 * C_tmp_tmp + 1]) +
           d1 * b_PR_tmp[3 * C_tmp_tmp + 2];
      PR[i + 3 * C_tmp_tmp] = d2;
      RP[C_tmp_tmp + 3 * i] = d2;
    }
    C_tmp[3 * i + 2] = b_iv[i];
  }
  //  compute velocities in inertial frame
  b_xk[0] = xk[3];
  b_xk[1] = xk[4];
  b_xk[2] = xk[5];
  for (int i{0}; i < 3; i++) {
    d = C_tmp[i];
    d1 = C_tmp[i + 3];
    i1 = static_cast<int>(C_tmp[i + 6]);
    for (C_tmp_tmp = 0; C_tmp_tmp < 3; C_tmp_tmp++) {
      b_C_tmp[i + 3 * C_tmp_tmp] =
          (d * PR_tmp[3 * C_tmp_tmp] + d1 * PR_tmp[3 * C_tmp_tmp + 1]) +
          static_cast<double>(i1) * PR_tmp[3 * C_tmp_tmp + 2];
    }
    d = b_C_tmp[i];
    d1 = b_C_tmp[i + 3];
    d2 = b_C_tmp[i + 6];
    d3 = 0.0;
    for (i1 = 0; i1 < 3; i1++) {
      d4 = (d * b_PR_tmp[3 * i1] + d1 * b_PR_tmp[3 * i1 + 1]) +
           d2 * b_PR_tmp[3 * i1 + 2];
      C[i + 3 * i1] = d4;
      d3 += d4 * b_xk[i1];
    }
    dotENU[i] = d3;
  }
  //  compute traction force in TGC-patch frame
  b_xk[0] = m * uk[1];
  //  moment arms in global coordinates
  //  if 0
  //      % for debugging
  //  compute derivative of profile beneath k-th wheel with respect to x
  //  else
  //  % compute road profiles from map at each time instant
  //  P_CG = [xg; yg; zg];
  //  P_FR = P_CG + RpFR; P_RR = P_CG + RpRR;
  //  P_FL = P_CG + RpFL; P_RL = P_CG + RpRL;
  //  % convert the wheel locations in LLA
  //  P_FR_LLA = enu2lla(P_FR', lla0, 'flat');
  //  P_RR_LLA = enu2lla(P_RR', lla0, 'flat');
  //  P_FL_LLA = enu2lla(P_FL', lla0, 'flat');
  //  P_RL_LLA = enu2lla(P_RL', lla0, 'flat');
  //
  //  % interpolate the height corresponding to wheel position
  //  % in LLA map first argument is long, and second is lat
  //  fFR = SIM_MAP_LLA(P_FR_LLA(2), P_FR_LLA(1)); fFL =
  //  SIM_MAP_LLA(P_FL_LLA(2), P_FL_LLA(1)); fRR = SIM_MAP_LLA(P_RR_LLA(2),
  //  P_RR_LLA(1)); fRL = SIM_MAP_LLA(P_RL_LLA(2), P_RL_LLA(1));
  //
  //  % compute the derivative of height using current position data
  //  dx = 0.1; % this values must be equal or smaller than map resolution
  //  dy = 0.1; % this values must be equal or smaller than map resolution
  //
  //  % compute the positions in LLA +/- dx distance from the wheel positions
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
  //
  //  % compute derivative of map(x,y) beneath k-th wheel with respect to x
  //  dfxFR = 1/(2*dx)*(SIM_MAP_LLA(P_FR_dx(1,2), P_FR_dx(1,1)) -
  //  SIM_MAP_LLA(P_FR_dx(2,2), P_FR_dx(2,1))); dfxFL =
  //  1/(2*dx)*(SIM_MAP_LLA(P_FL_dx(1,2), P_FL_dx(1,1)) -
  //  SIM_MAP_LLA(P_FL_dx(2,2), P_FL_dx(2,1))); dfxRR =
  //  1/(2*dx)*(SIM_MAP_LLA(P_RR_dx(1,2), P_RR_dx(1,1)) -
  //  SIM_MAP_LLA(P_RR_dx(2,2), P_RR_dx(2,1))); dfxRL =
  //  1/(2*dx)*(SIM_MAP_LLA(P_RL_dx(1,2), P_RL_dx(1,1)) -
  //  SIM_MAP_LLA(P_RL_dx(2,2), P_RL_dx(2,1)));
  //
  //  % compute derivative of map(x,y) beneath k-th wheel with respect to y
  //  dfyFR = 1/(2*dy)*(SIM_MAP_LLA(P_FR_dy(1,2), P_FR_dy(1,1)) -
  //  SIM_MAP_LLA(P_FR_dy(2,2), P_FR_dy(2,1))); dfyFL =
  //  1/(2*dy)*(SIM_MAP_LLA(P_FL_dy(1,2), P_FL_dy(1,1)) -
  //  SIM_MAP_LLA(P_FL_dy(2,2), P_FL_dy(2,1))); dfyRR =
  //  1/(2*dy)*(SIM_MAP_LLA(P_RR_dy(1,2), P_RR_dy(1,1)) -
  //  SIM_MAP_LLA(P_RR_dy(2,2), P_RR_dy(2,1))); dfyRL =
  //  1/(2*dy)*(SIM_MAP_LLA(P_RL_dy(1,2), P_RL_dy(1,1)) -
  //  SIM_MAP_LLA(P_RL_dy(2,2), P_RL_dy(2,1)));
  // end
  //  implement gimbal equations (don't change the sequence)
  dyaw = (xk[10] * Fr + xk[11] * b_varargout_1_tmp_tmp) / varargout_1_tmp_tmp;
  dpitch = xk[10] * b_varargout_1_tmp_tmp - xk[11] * Fr;
  droll = dyaw * absx + xk[9];
  //  define Euler transformation matrix and
  //  its derivative components
  d = -stgR * dyaw;
  dv[0] = d;
  dv[3] = -mur * dyaw;
  dv[6] = 0.0 * dyaw;
  dv[1] = mur * dyaw;
  dv[4] = d;
  dv[7] = 0.0 * dyaw;
  for (int i{0}; i < 3; i++) {
    Fcmd[i] = (PR[i] * b_xk[0] + PR[i + 3] * 0.0) + PR[i + 6] * 0.0;
    dv[3 * i + 2] = 0.0 * dyaw;
  }
  for (int i{0}; i < 3; i++) {
    d = dv[i];
    d1 = dv[i + 3];
    d2 = dv[i + 6];
    for (i1 = 0; i1 < 3; i1++) {
      dv1[i + 3 * i1] = (d * PR_tmp[3 * i1] + d1 * PR_tmp[3 * i1 + 1]) +
                        d2 * PR_tmp[3 * i1 + 2];
    }
  }
  d = -absx * dpitch;
  dv[0] = d;
  dv[3] = 0.0 * dpitch;
  dv[6] = varargout_1_tmp_tmp * dpitch;
  dv[1] = 0.0 * dpitch;
  dv[4] = 0.0 * dpitch;
  dv[7] = 0.0 * dpitch;
  dv[2] = -varargout_1_tmp_tmp * dpitch;
  dv[5] = 0.0 * dpitch;
  dv[8] = d;
  for (int i{0}; i < 3; i++) {
    d = C_tmp[i];
    d1 = C_tmp[i + 3];
    i1 = static_cast<int>(C_tmp[i + 6]);
    for (C_tmp_tmp = 0; C_tmp_tmp < 3; C_tmp_tmp++) {
      RTemp[i + 3 * C_tmp_tmp] =
          (d * dv[3 * C_tmp_tmp] + d1 * dv[3 * C_tmp_tmp + 1]) +
          static_cast<double>(i1) * dv[3 * C_tmp_tmp + 2];
    }
  }
  for (int i{0}; i < 3; i++) {
    d = dv1[i];
    d1 = dv1[i + 3];
    d2 = dv1[i + 6];
    d3 = RTemp[i];
    d4 = RTemp[i + 3];
    d5 = RTemp[i + 6];
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
      C_tmp_tmp = i + 3 * i1;
      C_tmp[C_tmp_tmp] = d8;
      dv[C_tmp_tmp] = d7;
    }
  }
  dv1[0] = 0.0 * droll;
  dv1[3] = 0.0 * droll;
  dv1[6] = 0.0 * droll;
  dv1[1] = 0.0 * droll;
  d = -Fr * droll;
  dv1[4] = d;
  dv1[7] = -b_varargout_1_tmp_tmp * droll;
  dv1[2] = 0.0 * droll;
  dv1[5] = b_varargout_1_tmp_tmp * droll;
  dv1[8] = d;
  //  derivatives of moment arms in global coordinates
  //  To compute the deflection of each spring first compute
  //  the HEIGHT of each strut-mount point with respect to
  //  the wheel
  //  the velocity of the strut mount point in ground frame
  //
  //  spring deflections -- output vector
  //  FR side
  for (int i{0}; i < 3; i++) {
    d = b_C_tmp[i];
    d1 = b_C_tmp[i + 3];
    d2 = b_C_tmp[i + 6];
    d3 = 0.0;
    for (i1 = 0; i1 < 3; i1++) {
      C_tmp_tmp = i + 3 * i1;
      RTemp[C_tmp_tmp] =
          (dv[C_tmp_tmp] + C_tmp[C_tmp_tmp]) +
          ((d * dv1[3 * i1] + d1 * dv1[3 * i1 + 1]) + d2 * dv1[3 * i1 + 2]);
      d3 += C[C_tmp_tmp] * pFR[i1];
    }
    b_xk[i] = d3;
  }
  y = std::log(std::exp(100.0 * -(xk[2] + b_xk[2])) + 1.0) / 100.0;
  //  FL side
  d = pFL[0];
  d1 = pFL[1];
  d2 = pFL[2];
  for (int i{0}; i < 3; i++) {
    b_xk[i] = (C[i] * d + C[i + 3] * d1) + C[i + 6] * d2;
  }
  varargout_1_tmp_tmp = std::exp(100.0 * -(xk[2] + b_xk[2]));
  //  RR side
  d = pRR[0];
  d1 = pRR[1];
  d2 = pRR[2];
  for (int i{0}; i < 3; i++) {
    b_xk[i] = (C[i] * d + C[i + 3] * d1) + C[i + 6] * d2;
  }
  absx = std::exp(100.0 * -(xk[2] + b_xk[2]));
  //  RL side
  d = pRL[0];
  d1 = pRL[1];
  d2 = pRL[2];
  for (int i{0}; i < 3; i++) {
    b_xk[i] = (C[i] * d + C[i + 3] * d1) + C[i + 6] * d2;
  }
  stgR = std::exp(100.0 * -(xk[2] + b_xk[2]));
  //  rate of change of spring deflections with respect to gradient
  //  of the ground profile
  //  compute normal force at each corner
  d = pFR[0];
  d1 = pFR[1];
  d2 = pFR[2];
  for (int i{0}; i < 3; i++) {
    b_xk[i] = (RTemp[i] * d + RTemp[i + 3] * d1) + RTemp[i + 6] * d2;
  }
  mur = 0.0 * dotENU[0] + 0.0 * dotENU[1];
  FztFR = (-kFR * -y - cFR * ((b_xk[2] + xk[5]) - mur)) + Fcmd[2] / 4.0;
  // max([FztFR, 0]);
  d = pFL[0];
  d1 = pFL[1];
  d2 = pFL[2];
  for (int i{0}; i < 3; i++) {
    b_xk[i] = (RTemp[i] * d + RTemp[i + 3] * d1) + RTemp[i + 6] * d2;
  }
  FztFL_tmp = -(std::log(varargout_1_tmp_tmp + 1.0) / 100.0);
  FztFL = (-kFL * FztFL_tmp - cFL * ((b_xk[2] + xk[5]) - mur)) + Fcmd[2] / 4.0;
  // max([FztFL, 0]);
  d = pRR[0];
  d1 = pRR[1];
  d2 = pRR[2];
  for (int i{0}; i < 3; i++) {
    b_xk[i] = (RTemp[i] * d + RTemp[i + 3] * d1) + RTemp[i + 6] * d2;
  }
  FztRR_tmp = -(std::log(absx + 1.0) / 100.0);
  FztRR = (-kRR * FztRR_tmp - cRR * ((b_xk[2] + xk[5]) - mur)) + Fcmd[2] / 4.0;
  // max([FztRR, 0]);
  d = pRL[0];
  d1 = pRL[1];
  d2 = pRL[2];
  for (int i{0}; i < 3; i++) {
    b_xk[i] = (RTemp[i] * d + RTemp[i + 3] * d1) + RTemp[i + 6] * d2;
  }
  FztRL_tmp = -(std::log(stgR + 1.0) / 100.0);
  FztRL = (-kRL * FztRL_tmp - cRL * ((b_xk[2] + xk[5]) - mur)) + Fcmd[2] / 4.0;
  // max([FztRL, 0]);
  //  Velocities for strut positions in vehicle coordinates
  d = xk[9];
  d1 = xk[10];
  d2 = xk[11];
  for (int i{0}; i < 9; i++) {
    PR_tmp[i] =
        (static_cast<double>(iv1[i]) * d + static_cast<double>(iv2[i]) * d1) +
        static_cast<double>(iv3[i]) * d2;
  }
  //  velocities of strut points due to additional moment induced by
  //  the instantaneous deflection of the spring above tire radius height
  //  Velocities of each strut in tire-ground contact patches
  d = pFR[0];
  d1 = pFR[1];
  d2 = pFR[2];
  d3 = pFL[0];
  d4 = pFL[1];
  d5 = pFL[2];
  d6 = pRR[0];
  d7 = pRR[1];
  d8 = pRR[2];
  absx = pRL[0];
  varargout_1_tmp_tmp = pRL[1];
  b_varargout_1_tmp_tmp = pRL[2];
  for (int i{0}; i < 3; i++) {
    double VuFL_tmp_tmp;
    double d9;
    stgR = PR_tmp[i];
    mur = stgR * d;
    Fr = stgR * d3;
    stgL = stgR * d6;
    d9 = stgR * absx;
    stgR = PR_tmp[i + 3];
    mur += stgR * d1;
    Fr += stgR * d4;
    stgL += stgR * d7;
    d9 += stgR * varargout_1_tmp_tmp;
    stgR = PR_tmp[i + 6];
    mur += stgR * d2;
    Fr += stgR * d5;
    stgL += stgR * d8;
    d9 += stgR * b_varargout_1_tmp_tmp;
    stgR = RP[i + 6];
    VuFL_tmp_tmp = RP[i] * 0.0 + RP[i + 3] * 0.0;
    VuFL_tmp[i] = VuFL_tmp_tmp + stgR * FztFL_tmp;
    MFR[i] = d9;
    VgRL[i] = stgL;
    VgRR[i] = Fr;
    VgFL[i] = mur;
    VuRR_tmp[i] = VuFL_tmp_tmp + stgR * FztRR_tmp;
    VuRL_tmp[i] = VuFL_tmp_tmp + stgR * FztRL_tmp;
    b_xk[i] = VuFL_tmp_tmp + stgR * -y;
  }
  FwFR[0] = b_xk[1] * xk[11] - b_xk[2] * xk[10];
  FwFR[1] = b_xk[2] * xk[9] - b_xk[0] * xk[11];
  FwFR[2] = b_xk[0] * xk[10] - b_xk[1] * xk[9];
  FwFR[0] += VgFL[0] + xk[3];
  FwFR[1] += VgFL[1] + xk[4];
  FwFR[2] += VgFL[2] + xk[5];
  d = FwFR[0];
  d1 = FwFR[1];
  d2 = FwFR[2];
  for (int i{0}; i < 3; i++) {
    VgFR[i] = (PR[i] * d + PR[i + 3] * d1) + PR[i + 6] * d2;
  }
  d = (VuFL_tmp[1] * xk[11] - VuFL_tmp[2] * xk[10]) + (VgRR[0] + xk[3]);
  d1 = (VuFL_tmp[2] * xk[9] - VuFL_tmp[0] * xk[11]) + (VgRR[1] + xk[4]);
  d2 = (VuFL_tmp[0] * xk[10] - VuFL_tmp[1] * xk[9]) + (VgRR[2] + xk[5]);
  for (int i{0}; i < 3; i++) {
    VgFL[i] = (PR[i] * d + PR[i + 3] * d1) + PR[i + 6] * d2;
  }
  d = (VuRR_tmp[1] * xk[11] - VuRR_tmp[2] * xk[10]) + (VgRL[0] + xk[3]);
  d1 = (VuRR_tmp[2] * xk[9] - VuRR_tmp[0] * xk[11]) + (VgRL[1] + xk[4]);
  d2 = (VuRR_tmp[0] * xk[10] - VuRR_tmp[1] * xk[9]) + (VgRL[2] + xk[5]);
  for (int i{0}; i < 3; i++) {
    VgRR[i] = (PR[i] * d + PR[i + 3] * d1) + PR[i + 6] * d2;
  }
  //  compute the wheel-ground contact patch velocities
  //  cornering force (lateral) of each wheel
  //  the problem with this method is the curvature becomes the inverse
  //  function of speed -- so kind of interdependancy of curvature and
  //  speed
  //  compute curvature commands for left and right wheel
  //  compute left and right wheel steering angles
  absx = tw / 2.0 * xk[12];
  stgL = std::atan(l * (xk[12] / (1.0 - absx)));
  stgR = std::atan(l * (xk[12] / (absx + 1.0)));
  //  Slip angles of each wheel
  //  to limit the side-slip angles to linear range for lateral force
  //  versus side-slip angles curve
  // lim_val(AlphaFR, -10.0*D2R, 10.0*D2R);
  // lim_val(AlphaFL, -10.0*D2R, 10.0*D2R);
  // lim_val(AlphaRR, -10.0*D2R, 10.0*D2R);
  // lim_val(AlphaRL, -10.0*D2R, 10.0*D2R);
  //  compute friction forces on each wheel
  mur = Fcmd[0] / (((FztRL + FztFL) + FztRR) + FztFR);
  //  to avoid mu -> Inf -- TBD
  //  if abs(acmd) > 0.0
  //  road inclination for front and rear wheels
  //  Compute lateral forces as a function of cornering stiffness
  //  coefficient
  //  compute the rolling resistance force in TGC-patch frame
  if (std::isnan(xk[3])) {
    d = rtNaN;
  } else if (xk[3] < 0.0) {
    d = -1.0;
  } else {
    d = (xk[3] > 0.0);
  }
  Fr = -m * g * Cr * d;
  //  total rolling resistance force
  //  should be distributed on each wheel by mass
  //  else
  //      % road inclination for front and rear wheels -- TBD
  //      gamma_FR = 0.0; gamma_FL = 0.0;
  //      gamma_RR = 0.0; gamma_RL = 0.0;
  //      FcFR = 0.0; FcFL = 0.0;
  //      FcRR = 0.0; FcRL = 0.0;
  //      FrFL = 0.0; FrRL = 0.0;
  //      FrFR = 0.0; FrRR = 0.0;
  //  end
  //  compute wheel forces in body frame
  absx = std::sin(-stgR);
  varargout_1_tmp_tmp = std::cos(-stgR);
  RTemp[0] = varargout_1_tmp_tmp;
  RTemp[3] = -absx;
  RTemp[6] = 0.0;
  RTemp[1] = absx;
  RTemp[4] = varargout_1_tmp_tmp;
  RTemp[7] = 0.0;
  d = (VuRL_tmp[1] * xk[11] - VuRL_tmp[2] * xk[10]) + (MFR[0] + xk[3]);
  d1 = (VuRL_tmp[2] * xk[9] - VuRL_tmp[0] * xk[11]) + (MFR[1] + xk[4]);
  d2 = (VuRL_tmp[0] * xk[10] - VuRL_tmp[1] * xk[9]) + (MFR[2] + xk[5]);
  for (int i{0}; i < 3; i++) {
    VgRL[i] = (PR[i] * d + PR[i + 3] * d1) + PR[i + 6] * d2;
    RTemp[3 * i + 2] = b_iv[i];
  }
  for (int i{0}; i < 3; i++) {
    d = RP[i];
    d1 = RP[i + 3];
    d2 = RP[i + 6];
    for (i1 = 0; i1 < 3; i1++) {
      PR_tmp[i + 3 * i1] =
          (d * RTemp[3 * i1] + d1 * RTemp[3 * i1 + 1]) + d2 * RTemp[3 * i1 + 2];
    }
  }
  b_xk[0] = mur * FztFR + Fr / 4.0;
  b_varargout_1_tmp_tmp = -xk[13] * Ca;
  b_xk[1] =
      b_varargout_1_tmp_tmp *
      (-std::log(std::exp(-std::log(
                     std::exp(10.0 * ((rt_atan2d_snf(VgFR[1], VgFR[0]) - stgR) *
                                      R2D)) +
                     3.7200759760208361E-44)) +
                 3.7200759760208361E-44) /
       10.0 * D2R);
  b_xk[2] = FztFR;
  for (int i{0}; i < 3; i++) {
    d = 0.0;
    d1 = PR_tmp[i];
    d2 = PR_tmp[i + 3];
    d3 = PR_tmp[i + 6];
    for (i1 = 0; i1 < 3; i1++) {
      d += ((d1 * static_cast<double>(b[3 * i1]) +
             d2 * static_cast<double>(b[3 * i1 + 1])) +
            d3 * static_cast<double>(b[3 * i1 + 2])) *
           b_xk[i1];
    }
    FwFR[i] = d;
  }
  absx = std::sin(-stgL);
  varargout_1_tmp_tmp = std::cos(-stgL);
  RTemp[0] = varargout_1_tmp_tmp;
  RTemp[3] = -absx;
  RTemp[6] = 0.0;
  RTemp[1] = absx;
  RTemp[4] = varargout_1_tmp_tmp;
  RTemp[7] = 0.0;
  RTemp[2] = 0.0;
  RTemp[5] = 0.0;
  RTemp[8] = 1.0;
  for (int i{0}; i < 3; i++) {
    d = RP[i];
    d1 = RP[i + 3];
    d2 = RP[i + 6];
    for (i1 = 0; i1 < 3; i1++) {
      PR_tmp[i + 3 * i1] =
          (d * RTemp[3 * i1] + d1 * RTemp[3 * i1 + 1]) + d2 * RTemp[3 * i1 + 2];
    }
  }
  b_xk[0] = mur * FztFL + Fr / 4.0;
  b_xk[1] =
      b_varargout_1_tmp_tmp *
      (-std::log(std::exp(-std::log(
                     std::exp(10.0 * ((rt_atan2d_snf(VgFL[1], VgFL[0]) - stgL) *
                                      R2D)) +
                     3.7200759760208361E-44)) +
                 3.7200759760208361E-44) /
       10.0 * D2R);
  b_xk[2] = FztFL;
  for (int i{0}; i < 3; i++) {
    d = 0.0;
    d1 = PR_tmp[i];
    d2 = PR_tmp[i + 3];
    d3 = PR_tmp[i + 6];
    for (i1 = 0; i1 < 3; i1++) {
      d += ((d1 * static_cast<double>(b[3 * i1]) +
             d2 * static_cast<double>(b[3 * i1 + 1])) +
            d3 * static_cast<double>(b[3 * i1 + 2])) *
           b_xk[i1];
    }
    FwFL[i] = d;
    d = RP[i];
    d1 = RP[i + 3];
    d2 = RP[i + 6];
    for (i1 = 0; i1 < 3; i1++) {
      PR_tmp[i + 3 * i1] = (d * static_cast<double>(b[3 * i1]) +
                            d1 * static_cast<double>(b[3 * i1 + 1])) +
                           d2 * static_cast<double>(b[3 * i1 + 2]);
    }
  }
  d = mur * FztRR + Fr / 4.0;
  d1 = b_varargout_1_tmp_tmp *
       (-std::log(std::exp(-std::log(
                      std::exp(10.0 * (rt_atan2d_snf(VgRR[1], VgRR[0]) * R2D)) +
                      3.7200759760208361E-44)) +
                  3.7200759760208361E-44) /
        10.0 * D2R);
  for (int i{0}; i < 3; i++) {
    VgFR[i] = (PR_tmp[i] * d + PR_tmp[i + 3] * d1) + PR_tmp[i + 6] * FztRR;
  }
  d = mur * FztRL + Fr / 4.0;
  d1 = b_varargout_1_tmp_tmp *
       (-std::log(std::exp(-std::log(
                      std::exp(10.0 * (rt_atan2d_snf(VgRL[1], VgRL[0]) * R2D)) +
                      3.7200759760208361E-44)) +
                  3.7200759760208361E-44) /
        10.0 * D2R);
  for (int i{0}; i < 3; i++) {
    Fcmd[i] = (PR_tmp[i] * d + PR_tmp[i + 3] * d1) + PR_tmp[i + 6] * FztRL;
  }
  //  compute moments in body frame
  //
  for (int i{0}; i < 3; i++) {
    b_xk[i] = pFR[i] + ((RP[i] * 0.0 + RP[i + 3] * 0.0) + RP[i + 6] * -y);
    VuFL_tmp[i] += pFL[i];
    VuRR_tmp[i] += pRR[i];
    VuRL_tmp[i] += pRL[i];
  }
  MFR[0] = b_xk[1] * FwFR[2] - FwFR[1] * b_xk[2];
  MFR[1] = FwFR[0] * b_xk[2] - b_xk[0] * FwFR[2];
  MFR[2] = b_xk[0] * FwFR[1] - FwFR[0] * b_xk[1];
  //
  //  Extract net forces for each wheel in vehicle frame
  //  Extract net moments
  //  total rolling moment
  //  total pitching moment
  //  total yawing moment
  //  G vector in inertial frame as
  for (int i{0}; i < 3; i++) {
    VgFL[i] = (C[3 * i] * 0.0 + C[3 * i + 1] * 0.0) + C[3 * i + 2] * -g;
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
  varargout_1_tmp_tmp = str_a * (57.295779513082323 * std::atan(l * xk[12])) +
                        str_b * (57.295779513082323 * std::atan(l * uk[0]));
  if (std::isinf(varargout_1_tmp_tmp) || std::isnan(varargout_1_tmp_tmp)) {
    varargout_1_tmp_tmp = rtNaN;
  } else {
    signed char n;
    varargout_1_tmp_tmp = rt_remd_snf(varargout_1_tmp_tmp, 360.0);
    absx = std::abs(varargout_1_tmp_tmp);
    if (absx > 180.0) {
      if (varargout_1_tmp_tmp > 0.0) {
        varargout_1_tmp_tmp -= 360.0;
      } else {
        varargout_1_tmp_tmp += 360.0;
      }
      absx = std::abs(varargout_1_tmp_tmp);
    }
    if (absx <= 45.0) {
      varargout_1_tmp_tmp *= 0.017453292519943295;
      n = 0;
    } else if (absx <= 135.0) {
      if (varargout_1_tmp_tmp > 0.0) {
        varargout_1_tmp_tmp =
            0.017453292519943295 * (varargout_1_tmp_tmp - 90.0);
        n = 1;
      } else {
        varargout_1_tmp_tmp =
            0.017453292519943295 * (varargout_1_tmp_tmp + 90.0);
        n = -1;
      }
    } else if (varargout_1_tmp_tmp > 0.0) {
      varargout_1_tmp_tmp =
          0.017453292519943295 * (varargout_1_tmp_tmp - 180.0);
      n = 2;
    } else {
      varargout_1_tmp_tmp =
          0.017453292519943295 * (varargout_1_tmp_tmp + 180.0);
      n = -2;
    }
    varargout_1_tmp_tmp = std::tan(varargout_1_tmp_tmp);
    if ((n == 1) || (n == -1)) {
      absx = 1.0 / varargout_1_tmp_tmp;
      varargout_1_tmp_tmp = -(1.0 / varargout_1_tmp_tmp);
      if (std::isinf(varargout_1_tmp_tmp) && (n == 1)) {
        varargout_1_tmp_tmp = absx;
      }
    }
  }
  dxk[0] = dotENU[0];
  dxk[1] = dotENU[1];
  dxk[2] = dotENU[2];
  dxk[3] = (((((FwFR[0] + FwFL[0]) + VgFR[0]) + Fcmd[0]) / m + VgFL[0]) -
            xk[5] * xk[10]) +
           xk[4] * xk[11];
  dxk[4] = (((((FwFR[1] + FwFL[1]) + VgFR[1]) + Fcmd[1]) / m + VgFL[1]) +
            xk[5] * xk[9]) -
           xk[3] * xk[11];
  dxk[5] = (((((FwFR[2] + FwFL[2]) + VgFR[2]) + Fcmd[2]) / m + VgFL[2]) -
            xk[4] * xk[9]) +
           xk[3] * xk[10];
  dxk[6] = droll;
  dxk[7] = dpitch;
  dxk[8] = dyaw;
  dxk[9] = (((((VuRR_tmp[1] * VgFR[2] - VgFR[1] * VuRR_tmp[2]) +
               (VuRL_tmp[1] * Fcmd[2] - Fcmd[1] * VuRL_tmp[2])) +
              MFR[0]) +
             (VuFL_tmp[1] * FwFL[2] - FwFL[1] * VuFL_tmp[2])) -
            xk[10] * xk[11] * (Izz - Iyy)) /
           Ixx;
  dxk[10] = (((((VgFR[0] * VuRR_tmp[2] - VuRR_tmp[0] * VgFR[2]) +
                (Fcmd[0] * VuRL_tmp[2] - VuRL_tmp[0] * Fcmd[2])) +
               MFR[1]) +
              (FwFL[0] * VuFL_tmp[2] - VuFL_tmp[0] * FwFL[2])) -
             xk[9] * xk[11] * (Ixx - Izz)) /
            Iyy;
  dxk[11] = (((((VuRR_tmp[0] * VgFR[1] - VgFR[0] * VuRR_tmp[1]) +
                (VuRL_tmp[0] * Fcmd[1] - Fcmd[0] * VuRL_tmp[1])) +
               MFR[2]) +
              (VuFL_tmp[0] * FwFL[1] - FwFL[0] * VuFL_tmp[1])) -
             xk[9] * xk[10] * (Iyy - Ixx)) /
            Izz;
  dxk[12] = varargout_1_tmp_tmp / l;
  dxk[13] = 0.0;
  //  for curvature/steering dynamics
  //  for CCa/TGC
  //  Euler integration step -- with augmented states
  // xkp1 = [xk(1:neq) + dt2*dxk;
  //     xk(1:ns-neq)];
  //  % % draw the vehicle
  //  FrontLoad = FzFR + FzFL;
  //  RearLoad = FzRR + FzRL;
  //  Vg_B = sqrt(u^2 + v^2 + w^2);
  //  drawVehicle(xk,FwFR,FwFL,FwRR,FwRL,...
  //      Vg_B, Fx, Fz, m*G_vec, m*acmd, gamma_FR, ...
  //      Fcmd, mur, AlphaFR, AlphaRR, delta_x_FL, ...
  //      delta_x_RR, L, M, N, FrontLoad, RearLoad, vFR, vRR, uFR, uRR);
}

// End of code generation (dynamic_model_nmpc.cpp)
