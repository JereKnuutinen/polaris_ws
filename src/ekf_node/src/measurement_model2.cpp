//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// measurement_model2.cpp
//
// Code generation for function 'measurement_model2'
//

// Include files
#include "measurement_model2.h"
#include "EKF_Node_data.h"
#include "enu2lla.h"
#include "rt_nonfinite.h"
#include "/home/mpc/Desktop/mba/mba/mba.hpp"
#include "intpol_codegen.h"
#include "myInterpolator2.h"
#include <cmath>

// Function Definitions
void measurement_model2(const double x[70], double Y[14])
{
  static const signed char b_iv[3]{0, 0, 1};
  double C_epec[9];
  double e_varargout_1_tmp[9];
  double f_varargout_1_tmp[9];
  double g_varargout_1_tmp[9];
  double P_CG[3];
  double P_FL[3];
  double P_FR[3];
  double P_RL[3];
  double P_RR[3];
  double dotENU[3];
  double u_nov[3];
  double b_varargout_1_tmp;
  double c_varargout_1_tmp;
  double d;
  double d1;
  double d2;
  double d_varargout_1_tmp;
  double fFL;
  double fRL;
  double fRR;
  double pitch;
  double pitch_nov;
  double roll;
  double roll_nov;
  double roll_nov_tmp;
  double roll_tmp;
  double varargout_1_tmp;
  double yaw;
  double yaw_nov;
  int i1;
  //  the measurement vector contains:
  //  1. spring deflections
  //  2. 2D position of cg
  //  3. orientation
  //  4. acquired odometry speed
  //  5. curvature
  //  define global variables
  //  extract state vector from augmented state vector for computation of
  //  spring deflections
  //  position of CG
  //  CG velocities in body frame
  roll_tmp = neq * D_Epec;
  roll = x[static_cast<int>(roll_tmp + 7.0) - 1];
  pitch = x[static_cast<int>(roll_tmp + 8.0) - 1];
  yaw = x[static_cast<int>(roll_tmp + 9.0) - 1];
  //  Euler angles
  //  body rate about CG
  //  Cr_epec = x(14 + neq*D_Epec);
  //  extract state vector from augmented state vector for computation of
  //  novatel values
  //  position of CG
  //  CG velocities in body frame
  roll_nov_tmp = neq * D_Novatel;
  roll_nov = x[static_cast<int>(roll_nov_tmp + 7.0) - 1];
  pitch_nov = x[static_cast<int>(roll_nov_tmp + 8.0) - 1];
  yaw_nov = x[static_cast<int>(roll_nov_tmp + 9.0) - 1];
  //  Euler angles
  //  body rate about CG
  //  Cr_nov = x(14 + neq*D_Novatel);
  //  rotation matrix around X-axis and its derivatives
  //  Rotation matrix around Y-axis and its derivatives
  //  Rotation matrix around Z-axis and its derivatives
  //  body to global (CG) transformation
  varargout_1_tmp = std::sin(yaw);
  yaw = std::cos(yaw);
  b_varargout_1_tmp = std::sin(pitch);
  c_varargout_1_tmp = std::cos(pitch);
  d_varargout_1_tmp = std::sin(roll);
  pitch = std::cos(roll);
  e_varargout_1_tmp[0] = yaw;
  e_varargout_1_tmp[3] = -varargout_1_tmp;
  e_varargout_1_tmp[6] = 0.0;
  e_varargout_1_tmp[1] = varargout_1_tmp;
  e_varargout_1_tmp[4] = yaw;
  e_varargout_1_tmp[7] = 0.0;
  f_varargout_1_tmp[0] = c_varargout_1_tmp;
  f_varargout_1_tmp[3] = 0.0;
  f_varargout_1_tmp[6] = b_varargout_1_tmp;
  e_varargout_1_tmp[2] = 0.0;
  f_varargout_1_tmp[1] = 0.0;
  e_varargout_1_tmp[5] = 0.0;
  f_varargout_1_tmp[4] = 1.0;
  e_varargout_1_tmp[8] = 1.0;
  f_varargout_1_tmp[7] = 0.0;
  f_varargout_1_tmp[2] = -b_varargout_1_tmp;
  f_varargout_1_tmp[5] = 0.0;
  f_varargout_1_tmp[8] = c_varargout_1_tmp;
  for (int i{0}; i < 3; i++) {
    d = e_varargout_1_tmp[i];
    d1 = e_varargout_1_tmp[i + 3];
    i1 = static_cast<int>(e_varargout_1_tmp[i + 6]);
    for (int i2{0}; i2 < 3; i2++) {
      g_varargout_1_tmp[i + 3 * i2] =
          (d * f_varargout_1_tmp[3 * i2] + d1 * f_varargout_1_tmp[3 * i2 + 1]) +
          static_cast<double>(i1) * f_varargout_1_tmp[3 * i2 + 2];
    }
  }
  f_varargout_1_tmp[0] = 1.0;
  f_varargout_1_tmp[3] = 0.0;
  f_varargout_1_tmp[6] = 0.0;
  f_varargout_1_tmp[1] = 0.0;
  f_varargout_1_tmp[4] = pitch;
  f_varargout_1_tmp[7] = -d_varargout_1_tmp;
  f_varargout_1_tmp[2] = 0.0;
  f_varargout_1_tmp[5] = d_varargout_1_tmp;
  f_varargout_1_tmp[8] = pitch;
  //  compute velocities in inertial frame
  varargout_1_tmp = std::sin(yaw_nov);
  yaw = std::cos(yaw_nov);
  b_varargout_1_tmp = std::sin(pitch_nov);
  c_varargout_1_tmp = std::cos(pitch_nov);
  d_varargout_1_tmp = std::sin(roll_nov);
  pitch = std::cos(roll_nov);
  e_varargout_1_tmp[0] = yaw;
  e_varargout_1_tmp[3] = -varargout_1_tmp;
  e_varargout_1_tmp[6] = 0.0;
  e_varargout_1_tmp[1] = varargout_1_tmp;
  e_varargout_1_tmp[4] = yaw;
  e_varargout_1_tmp[7] = 0.0;
  for (int i{0}; i < 3; i++) {
    d = g_varargout_1_tmp[i];
    d1 = g_varargout_1_tmp[i + 3];
    d2 = g_varargout_1_tmp[i + 6];
    for (i1 = 0; i1 < 3; i1++) {
      C_epec[i + 3 * i1] =
          (d * f_varargout_1_tmp[3 * i1] + d1 * f_varargout_1_tmp[3 * i1 + 1]) +
          d2 * f_varargout_1_tmp[3 * i1 + 2];
    }
    e_varargout_1_tmp[3 * i + 2] = b_iv[i];
  }
  f_varargout_1_tmp[0] = c_varargout_1_tmp;
  f_varargout_1_tmp[3] = 0.0;
  f_varargout_1_tmp[6] = b_varargout_1_tmp;
  f_varargout_1_tmp[1] = 0.0;
  f_varargout_1_tmp[4] = 1.0;
  f_varargout_1_tmp[7] = 0.0;
  f_varargout_1_tmp[2] = -b_varargout_1_tmp;
  f_varargout_1_tmp[5] = 0.0;
  f_varargout_1_tmp[8] = c_varargout_1_tmp;
  for (int i{0}; i < 3; i++) {
    d = e_varargout_1_tmp[i];
    d1 = e_varargout_1_tmp[i + 3];
    i1 = static_cast<int>(e_varargout_1_tmp[i + 6]);
    for (int i2{0}; i2 < 3; i2++) {
      g_varargout_1_tmp[i + 3 * i2] =
          (d * f_varargout_1_tmp[3 * i2] + d1 * f_varargout_1_tmp[3 * i2 + 1]) +
          static_cast<double>(i1) * f_varargout_1_tmp[3 * i2 + 2];
    }
  }
  f_varargout_1_tmp[0] = 1.0;
  f_varargout_1_tmp[3] = 0.0;
  f_varargout_1_tmp[6] = 0.0;
  f_varargout_1_tmp[1] = 0.0;
  f_varargout_1_tmp[4] = pitch;
  f_varargout_1_tmp[7] = -d_varargout_1_tmp;
  f_varargout_1_tmp[2] = 0.0;
  f_varargout_1_tmp[5] = d_varargout_1_tmp;
  f_varargout_1_tmp[8] = pitch;
  u_nov[0] = x[static_cast<int>(roll_nov_tmp + 4.0) - 1];
  u_nov[1] = x[static_cast<int>(roll_nov_tmp + 5.0) - 1];
  u_nov[2] = x[static_cast<int>(roll_nov_tmp + 6.0) - 1];
  //  moment arms in global coordinates
  //  compute 2D positions of the wheels
  P_CG[0] = x[static_cast<int>(roll_tmp + 1.0) - 1];
  P_CG[1] = x[static_cast<int>(roll_tmp + 2.0) - 1];
  P_CG[2] = x[static_cast<int>(roll_tmp + 3.0) - 1];
  for (int i{0}; i < 3; i++) {
    d = 0.0;
    d1 = 0.0;
    d2 = 0.0;
    yaw = 0.0;
    pitch = 0.0;
    roll = g_varargout_1_tmp[i];
    varargout_1_tmp = g_varargout_1_tmp[i + 3];
    c_varargout_1_tmp = g_varargout_1_tmp[i + 6];
    for (i1 = 0; i1 < 3; i1++) {
      pitch += ((roll * f_varargout_1_tmp[3 * i1] +
                 varargout_1_tmp * f_varargout_1_tmp[3 * i1 + 1]) +
                c_varargout_1_tmp * f_varargout_1_tmp[3 * i1 + 2]) *
               u_nov[i1];
      d_varargout_1_tmp = C_epec[i + 3 * i1];
      d += d_varargout_1_tmp * pFR[i1];
      d1 += d_varargout_1_tmp * pRR[i1];
      d2 += d_varargout_1_tmp * pFL[i1];
      yaw += d_varargout_1_tmp * pRL[i1];
    }
    dotENU[i] = pitch;
    pitch = P_CG[i];
    P_FR[i] = pitch + d;
    P_RR[i] = pitch + d1;
    P_FL[i] = pitch + d2;
    P_RL[i] = pitch + yaw;
  }
  double P_FL_LLA[3];
  double P_FR_LLA[3];
  double P_RL_LLA[3];
  double P_RR_LLA[3];
  double fFR;
  //  convert the wheel locations in LLA
  coder::enu2lla(P_FR, lla0, P_FR_LLA);
  coder::enu2lla(P_RR, lla0, P_RR_LLA);
  coder::enu2lla(P_FL, lla0, P_FL_LLA);
  coder::enu2lla(P_RL, lla0, P_RL_LLA);
  //  interpolate the height corresponding to wheel position
  //  in LLA map first argument is long, and second is lat
  // fFR = SIM_MAP_LLA(P_FR_LLA(2), P_FR_LLA(1)); fFL = SIM_MAP_LLA(P_FL_LLA(2),
  // P_FL_LLA(1)); fRR = SIM_MAP_LLA(P_RR_LLA(2), P_RR_LLA(1)); fRL =
  // SIM_MAP_LLA(P_RL_LLA(2), P_RL_LLA(1));
  fFR = int_at(P_FR_LLA[1], P_FR_LLA[0]);
  fRR = int_at(P_RR_LLA[1], P_RR_LLA[0]);
  fFL = int_at(P_FL_LLA[1], P_FL_LLA[0]);
  fRL = int_at(P_RL_LLA[1], P_RL_LLA[0]);
  //  fFR = coder.ceval("int_at", P_FR(1), P_FR(2));
  //  fRR = coder.ceval("int_at", P_RR(1), P_RR(2));
  //  fFL = coder.ceval("int_at", P_FL(1), P_FL(2));
  //  fRL = coder.ceval("int_at", P_RL(1), P_RL(2));
  //  --------------------------------------------------
  //  TB: DO NOT CHANGE THE MIN to SMOOTH_MAX_ZERO here
  //  --------------------------------------------------
  //  spring deflections -- output vector
  //  delta_x_FR = P_FR_LLA(3) - fFR; % FR side
  //  delta_x_FR = min([delta_x_FR, 0]);%-smooth_max_zero(-delta_x_FR); %
  //  delta_x_FL = P_FL_LLA(3) - fFL; % FL side
  //  delta_x_FL = min([delta_x_FL, 0]);%-smooth_max_zero(-delta_x_FL); %
  //  delta_x_RR = P_RR_LLA(3) - fRR; % RR side
  //  delta_x_RR = min([delta_x_RR, 0]);%-smooth_max_zero(-delta_x_RR);%
  //  delta_x_RL = P_RL_LLA(3) - fRL; % RL side
  //  delta_x_RL = min([delta_x_RL, 0]);%-smooth_max_zero(-delta_x_RL); %
  // P_FR
  // P_FR_LLA
  //  FR side
  // -smooth_max_zero(-delta_x_FR); %
  //  FL side
  // -smooth_max_zero(-delta_x_FL); %
  //  RR side
  // -smooth_max_zero(-delta_x_RR);%
  //  RL side
  // -smooth_max_zero(-delta_x_RL); %
  //  corresponding measurement for speed data from epec
  // sign(u_epec)*sqrt((u_epec*cos(pitch_epec))^2 +
  // (w_epec*sin(pitch_epec))^2);% + v_epec^2);
  //  to generate Jacobian using MATLAB, do not pre-allocate the output vector,
  //  and it is recommended to declare the output vector as follows:
  roll = P_FR[2] - fFR;
  if ((roll > 0.0) || std::isnan(roll)) {
    varargout_1_tmp = 0.0;
  } else {
    varargout_1_tmp = roll;
  }
  roll = P_FL[2] - fFL;
  if ((roll > 0.0) || std::isnan(roll)) {
    pitch = 0.0;
  } else {
    pitch = roll;
  }
  roll = P_RR[2] - fRR;
  if ((roll > 0.0) || std::isnan(roll)) {
    yaw = 0.0;
  } else {
    yaw = roll;
  }
  roll = P_RL[2] - fRL;
  if ((roll > 0.0) || std::isnan(roll)) {
    roll = 0.0;
  }
  Y[0] = varargout_1_tmp;
  Y[1] = pitch;
  Y[2] = yaw;
  Y[3] = roll;
  Y[4] = x[static_cast<int>(roll_nov_tmp + 1.0) - 1];
  Y[5] = x[static_cast<int>(roll_nov_tmp + 2.0) - 1];
  Y[6] = roll_nov;
  Y[7] = pitch_nov;
  Y[8] = yaw_nov;
  Y[9] = dotENU[0];
  Y[10] = dotENU[1];
  Y[11] = dotENU[2];
  Y[12] = x[static_cast<int>(roll_tmp + 4.0) - 1];
  Y[13] = x[static_cast<int>(roll_tmp + 13.0) - 1];
  //  ground speed from epec
  //  curvature state from steering model
}

// End of code generation (measurement_model2.cpp)
