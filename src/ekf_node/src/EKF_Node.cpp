//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// EKF_Node.cpp
//
// Code generation for function 'EKF_Node'
//

// Include files
#include "EKF_Node.h"
#include "EKF_Node_data.h"
#include "EKF_Node_initialize.h"
#include "EKF_Node_types.h"
#include "Publisher.h"
#include "Rate.h"
#include "Subscriber.h"
#include "diag.h"
#include "enu2lla.h"
#include "eul2quat.h"
#include "extendedKalmanFilter.h"
#include "lla2enu.h"
#include "mtimes.h"
#include "rt_nonfinite.h"
#include "tand.h"
#include "tic.h"
#include "toc.h"
#include "validate_print_arguments.h"
#include "/home/mpc/Desktop/mba/mba/mba.hpp"
#include "coder_array.h"
#include "coder_posix_time.h"
#include "intpol_codegen.h"
#include "mlroscpp_pub.h"
#include "mlroscpp_rate.h"
#include "myInterpolator2.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

// Function Declarations
static void EKF_Node_anonFcn1(double roll, double varargout_1[9]);

static void EKF_Node_anonFcn2(double pitch, double varargout_1[9]);

static void EKF_Node_anonFcn3(double yaw, double varargout_1[9]);

// Function Definitions
static void EKF_Node_anonFcn1(double roll, double varargout_1[9])
{
  double b_varargout_1_tmp;
  double varargout_1_tmp;
  varargout_1_tmp = std::sin(roll);
  b_varargout_1_tmp = std::cos(roll);
  varargout_1[0] = 1.0;
  varargout_1[3] = 0.0;
  varargout_1[6] = 0.0;
  varargout_1[1] = 0.0;
  varargout_1[4] = b_varargout_1_tmp;
  varargout_1[7] = -varargout_1_tmp;
  varargout_1[2] = 0.0;
  varargout_1[5] = varargout_1_tmp;
  varargout_1[8] = b_varargout_1_tmp;
}

static void EKF_Node_anonFcn2(double pitch, double varargout_1[9])
{
  double b_varargout_1_tmp;
  double varargout_1_tmp;
  varargout_1_tmp = std::sin(pitch);
  b_varargout_1_tmp = std::cos(pitch);
  varargout_1[0] = b_varargout_1_tmp;
  varargout_1[3] = 0.0;
  varargout_1[6] = varargout_1_tmp;
  varargout_1[1] = 0.0;
  varargout_1[4] = 1.0;
  varargout_1[7] = 0.0;
  varargout_1[2] = -varargout_1_tmp;
  varargout_1[5] = 0.0;
  varargout_1[8] = b_varargout_1_tmp;
}

static void EKF_Node_anonFcn3(double yaw, double varargout_1[9])
{
  double b_varargout_1_tmp;
  double varargout_1_tmp;
  varargout_1_tmp = std::sin(yaw);
  b_varargout_1_tmp = std::cos(yaw);
  varargout_1[0] = b_varargout_1_tmp;
  varargout_1[3] = -varargout_1_tmp;
  varargout_1[6] = 0.0;
  varargout_1[1] = varargout_1_tmp;
  varargout_1[4] = b_varargout_1_tmp;
  varargout_1[7] = 0.0;
  varargout_1[2] = 0.0;
  varargout_1[5] = 0.0;
  varargout_1[8] = 1.0;
}

void EKF_Node()
{
  static coder::extendedKalmanFilter myFilter;
  static double dv6[4900];
  coder::ros::Publisher EKF_out;
  coder::ros::Rate rr;
  coder::ros::Subscriber novatel_odom;
  coder::ros::b_Subscriber novatel_gps;
  coder::ros::c_Subscriber atv_wheels;
  coder::ros::d_Subscriber atv_steering;
  coder::ros::e_Subscriber atv_odom;
  coder::ros::f_Subscriber atv_cmd;
  coder::array<double, 2U> Fa;
  coder::array<double, 2U> b_result;
  coder::array<double, 2U> result;
  coder::array<double, 1U> P_est;
  nav_msgs_OdometryStruct_T EKF_msg;
  double dv1[169];
  double b_value[70];
  double ym[14];
  double b_dv[13];
  double C[9];
  double dv2[9];
  double dv3[9];
  double dv4[9];
  double dv5[9];
  double validatedHoleFilling[3];
  double K_avg;
  double b_pFR_tmp;
  double d;
  double mFL_tmp;
  double pFR_tmp;
  double pRR_tmp;
  int b_input_sizes_idx_1;
  int input_sizes_idx_1;
  int u0;
  bool empty_non_axis_sizes;
  if (!isInitialized_EKF_Node) {
    EKF_Node_initialize();
  }
  // clear all;
  //  persistent myFilter
  //  global variables to be used in while(1)
  //  rotation matrix around X-axis and its derivatives
  //  Rotation matrix around Y-axis and its derivatives
  //  Rotation matrix around Z-axis and its derivatives
  //  initialize the global variables
  delta_x_FR = 0.0;
  delta_x_FL = 0.0;
  delta_x_RR = 0.0;
  delta_x_RL = 0.0;
  phi_m = 0.0;
  theta_m = 0.0;
  psi_m = 0.0;
  dotE_m = 0.0;
  dotN_m = 0.0;
  dotU_m = 0.0;
  pos_gnss_lla[0] = 0.0;
  pos_gnss_lla[1] = 0.0;
  pos_gnss_lla[2] = 0.0;
  //  fixed GCP point in Vakola
  stg = 0.0;
  curv = 0.0;
  stg_cmd = 0.0;
  V_cmd = 0.0;
  acmd = 0.0;
  Kcmd = 0.0;
  TGC = 0.01;
  // 0.2;
  //  initialize the global constants
  Ca = 10419.0;
  //  cornering stiffness
  g = 9.8;
  //  acceleration due to gravity -- m/s^2
  Cr = 0.0397;
  //  rolling resistance coefficient
  D2R = 0.017453292519943295;
  //  degrees to radians
  R2D = 1.0 / D2R;
  //  radians to degrees
  //  coefficient for force acceleration model
  // acc_a = -0.515; acc_b = -acc_a; acc_c = g*Cr;
  acc_a = -1.0;
  acc_b = -acc_a;
  acc_c = g * Cr;
  //  coefficient for steering dynamics model
  // str_a = -1.427; str_b = -str_a;
  str_a = -2.0;
  str_b = -str_a;
  //  mass at each corner
  mFL = 190.0;
  mFR = 190.0;
  mRL = 260.0;
  mRR = 260.0;
  //  mass of two drivers
  m = (((mFL + mFR) + mRL) + mRR) + 180.0;
  //  the moment arms from tests
  l = 1.83;
  tw = 1.16;
  h = 1.7534;
  //  tire radius
  h_T = 0.3175;
  //  extended lengths of each side are used to compute inertia matrix
  //  depth -- m
  //  width -- m
  //  height -- m
  //  compute inertia matrix values by using the full lengths of polaris
  Ixx = m / 12.0 * 5.4961;
  //  inertia around X
  Iyy = m / 12.0 * 10.930100000000001;
  //  inertia around Y
  Izz = m / 12.0 * 9.5812;
  //  inertia around Z
  //  even mass on each side
  mFL_tmp = m / 4.0;
  mFL = mFL_tmp;
  mFR = mFL_tmp;
  mRL = mFL_tmp;
  mRR = mFL_tmp;
  //  spring coefficients from calibration test
  kFL = 15791.0;
  kFR = 13099.0;
  kRL = 17327.0;
  kRR = 16467.0;
  K_avg = (((kFL + kRL) + kFR) + kRR) / 4.0;
  kFL = K_avg;
  kFR = K_avg;
  kRL = K_avg;
  kRR = K_avg;
  //  define offsets of CG from VC
  xoff = 0.0;
  //  front side is lighter than rear
  yoff = 0.0;
  //  load is evenly distributed to left and right sides
  zoff = 0.0;
  //  below H_VC
  //  high damping with high settling time -- TBD
  cFL = 2.0 * std::sqrt(kFL * mFL);
  cFR = 2.0 * std::sqrt(kFR * mFR);
  cRL = 2.0 * std::sqrt(kRL * mRL);
  cRR = 2.0 * std::sqrt(kRR * mRR);
  //  initialize the moment arms here, so the same should be used in dynamic
  //  model and for data post-processing.
  mFL_tmp = l / 2.0 - xoff;
  pFR[0] = mFL_tmp;
  pFR_tmp = -tw / 2.0 - yoff;
  pFR[1] = pFR_tmp;
  b_pFR_tmp = -h / 2.0 - zoff;
  pFR[2] = b_pFR_tmp;
  pRR_tmp = -l / 2.0 - xoff;
  pRR[0] = pRR_tmp;
  pRR[1] = pFR_tmp;
  pRR[2] = b_pFR_tmp;
  pFL[0] = mFL_tmp;
  mFL_tmp = tw / 2.0 - yoff;
  pFL[1] = mFL_tmp;
  pFL[2] = b_pFR_tmp;
  pRL[0] = pRR_tmp;
  pRL[1] = mFL_tmp;
  pRL[2] = b_pFR_tmp;
  //  x, y, and z antenna to vehicle CM offsets
  roll_bias = 0.00017611;
  pitch_bias = -8.5281E-5;
  VEC_OFF[0] = 0.4724;
  VEC_OFF[1] = -0.2546;
  VEC_OFF[2] = -(h / 2.0);
  dt2 = 0.05;
  //  delay variables
  mFL_tmp = 0.2 / dt2;
  D_Epec = mFL_tmp;
  D_Epec = std::ceil(D_Epec);
  D_Novatel = mFL_tmp;
  D_Novatel = std::ceil(D_Novatel);
  //  no of times state is augmented
  na = std::fmax(D_Epec, D_Novatel) + 1.0;
  //  initialize variables for the number of equations in EKF
  nd = 13.0;
  //  no of equation corresponding to dynamic model
  np = 1.0;
  //  no of parameters
  ny = 14.0;
  //  no of measurements
  nu = 3.0;
  //  number of inputs
  neq = nd + np;
  //  total no of system equations
  ns = neq * na;
  //  total no of augmented states
  //  first noted position as lla0 -- don't delete it
  //  lla0=[60.449414203022314 24.35479561532673 0.0];
  //  initialize the process noise covariance matrix
  //  xg
  //  yg
  //  zg
  //  u
  //  v
  //  w
  //  roll
  //  pitch
  //  yaw
  //  p
  //  q
  //  r
  //  curvature
  //  compute the joint covariance matrix for state and parameters
  // Fa = Fw;
  if ((static_cast<int>(nd) != 0) && (static_cast<int>(np) != 0)) {
    input_sizes_idx_1 = static_cast<int>(np);
  } else {
    input_sizes_idx_1 = 0;
  }
  b_dv[0] = 1.0E-6;
  b_dv[1] = 1.0E-6;
  b_dv[2] = 1.0E-8;
  b_dv[3] = 1.0E-6;
  b_dv[4] = 1.0E-8;
  b_dv[5] = 1.0E-8;
  b_dv[6] = 1.0E-8 * D2R;
  b_dv[7] = 1.0E-8 * D2R;
  b_dv[8] = 1.0E-8 * D2R;
  b_dv[9] = 1.0E-8 * D2R;
  b_dv[10] = 1.0E-8 * D2R;
  b_dv[11] = 1.0E-8 * D2R;
  b_dv[12] = 1.0E-6;
  coder::diag(b_dv, dv1);
  result.set_size(13, input_sizes_idx_1 + 13);
  for (int i{0}; i < 13; i++) {
    for (int i1{0}; i1 < 13; i1++) {
      result[i1 + 13 * i] = dv1[i1 + 13 * i];
    }
  }
  for (int i{0}; i < input_sizes_idx_1; i++) {
    for (int i1{0}; i1 < 13; i1++) {
      result[i1 + 13 * (i + 13)] = 0.0;
    }
  }
  if ((static_cast<int>(np) != 0) && (static_cast<int>(nd) != 0)) {
    u0 = static_cast<int>(np);
  } else if (static_cast<int>(np) != 0) {
    u0 = static_cast<int>(np);
  } else {
    u0 = static_cast<int>(np);
    if (u0 < 0) {
      u0 = 0;
    }
    if (static_cast<int>(np) > u0) {
      u0 = static_cast<int>(np);
    }
  }
  empty_non_axis_sizes = (u0 == 0);
  if (empty_non_axis_sizes ||
      ((static_cast<int>(np) != 0) && (static_cast<int>(nd) != 0))) {
    b_input_sizes_idx_1 = static_cast<int>(nd);
  } else {
    b_input_sizes_idx_1 = 0;
  }
  if (empty_non_axis_sizes || (static_cast<int>(np) != 0)) {
    input_sizes_idx_1 = static_cast<int>(np);
  } else {
    input_sizes_idx_1 = 0;
  }
  b_result.set_size(u0, b_input_sizes_idx_1 + input_sizes_idx_1);
  for (int i{0}; i < b_input_sizes_idx_1; i++) {
    for (int i1{0}; i1 < u0; i1++) {
      b_result[i1 + b_result.size(0) * i] = 0.0;
    }
  }
  for (int i{0}; i < input_sizes_idx_1; i++) {
    for (int i1{0}; i1 < u0; i1++) {
      b_result[i1 + b_result.size(0) * (i + b_input_sizes_idx_1)] = 0.0;
    }
  }
  if ((b_result.size(0) != 0) && (b_result.size(1) != 0)) {
    input_sizes_idx_1 = b_result.size(0);
  } else {
    input_sizes_idx_1 = 0;
  }
  Fa.set_size(input_sizes_idx_1 + 13, result.size(1));
  u0 = result.size(1);
  for (int i{0}; i < u0; i++) {
    for (int i1{0}; i1 < 13; i1++) {
      Fa[i1 + Fa.size(0) * i] = result[i1 + 13 * i];
    }
    for (int i1{0}; i1 < input_sizes_idx_1; i1++) {
      Fa[(i1 + Fa.size(0) * i) + 13] = 0.0;
    }
  }
  //  Initialize the process covariance for AS-EKF
  std::memset(&Q_t[0], 0, 4900U * sizeof(double));
  coder::internal::blas::mtimes(Fa, Fa, b_result);
  for (int i{0}; i < 14; i++) {
    for (int i1{0}; i1 < 14; i1++) {
      Q_t[i1 + 70 * i] = b_result[i1 + 14 * i];
    }
  }
  //  measurement covariance matrix
  //  xg  - Novatel
  //  yg  - Novatel
  //  psi - Novatel
  //  Vt  - epec
  //  compute diagonal matrix
  std::copy(&dv[0], &dv[196], &R_t[0]);
  //  initialize the state covariance matrix
  coder::diag(Pkm1);
  Pkm1[0] = 0.01;
  //  xg
  Pkm1[71] = 0.01;
  //  yg
  Pkm1[142] = 0.1;
  //  zg
  Pkm1[213] = 0.1;
  //  u
  Pkm1[284] = 0.1;
  //  v
  Pkm1[355] = 0.1;
  //  w
  Pkm1[426] = 0.1 * D2R;
  //  phi
  Pkm1[497] = 0.1 * D2R;
  //  theta
  Pkm1[568] = 0.01 * D2R;
  //  psi
  Pkm1[639] = 0.5 * D2R;
  //  p
  Pkm1[710] = 0.5 * D2R;
  //  q
  Pkm1[781] = 0.5 * D2R;
  //  r
  Pkm1[852] = 0.015;
  //  curvature
  Pkm1[923] = 0.001;
  //  TGC
  //  Own scatter interpolate
  //  res = double(1);
  //  coder.cinclude("/home/mpc/Desktop/mba/mba/mba.hpp");
  //  coder.cinclude("myInterpolator.h");
  //  coder.cinclude("myInterpolatorwrapper.h");
  //  coder.updateBuildInfo("addSourceFiles", "myInterpolator.cpp");
  //  coder.updateBuildInfo("addSourceFiles", "myInterpolatorwrapper.cpp");
  //
  //  %global myinterpolatorptr
  //  if coder.target('MATLAB')
  //      const = [0];
  //  else
  //      myinterpolatorptr =
  //      coder.opaque('interpolatortype','HeaderFile','"myInterpolatorwrapper.h"');
  //      myinterpolatorptr = coder.ceval('interpolator_constructor');
  //      res = coder.ceval('interpolateat_wrapper',myinterpolatorptr, 354489.0,
  //      6704413.0); const = [myinterpolatorptr];
  //
  //  end
  //  res = coder.ceval("int_at", 354489, 6704413);
  //  res1 = coder.ceval("int_at", 354489, 6704413);
  //  fprintf('map value: %f.\n',res);
  //  fprintf('map value: %f.\n',res1);
  //  initialize the EKF Object
  myFilter.init();
  myFilter.set_ProcessNoise(Q_t);
  myFilter.set_MeasurementNoise(R_t);
  myFilter.set_StateCovariance(Pkm1);
  //  Initiliaze subscribers
  //  from novatel:
  novatel_odom.init();
  novatel_gps.init();
  // from can_node:
  atv_wheels.init();
  atv_steering.init();
  atv_odom.init();
  atv_cmd.init();
  // initialize publisher
  EKF_out.init();
  // CMD_out =
  // rospublisher("/cmd_out",'geometry_msgs/Accel','DataFormat','struct');
  //  two more publishers are needed
  coder::ros::Publisher::rosmessage(EKF_msg);
  // CMD_msg=rosmessage(CMD_out);
  //  wait for at least one data pack from each measurement message
  novatel_odom.receive();
  novatel_gps.receive();
  atv_wheels.receive();
  atv_steering.receive();
  atv_odom.receive();
  // atv_cmd_msg=receive(atv_cmd,100);
  //  disp('SatFix received: Initializing EKF Filter Object');
  coder::internal::validate_print_arguments(
      pos_gnss_lla[0], pos_gnss_lla[1], pos_gnss_lla[2], validatedHoleFilling);
  std::printf("Initial Position: %f, %f, %f.\n", validatedHoleFilling[0],
              validatedHoleFilling[1], validatedHoleFilling[2]);
  std::fflush(stdout);
  //  How to resolve this
  lla0[0] = pos_gnss_lla[0];
  lla0[1] = pos_gnss_lla[1];
  lla0[2] = 0.0;
  coder::lla2enu(pos_gnss_lla, lla0, pos_enu_gnss);
  // pos_enu_gnss = [ll2utm(pos_gnss_lla(1), pos_gnss_lla(2)), pos_gnss_lla(3)];
  coder::internal::validate_print_arguments(
      pos_enu_gnss[0], pos_enu_gnss[1], pos_enu_gnss[2], validatedHoleFilling);
  std::printf("Initial Position: %f, %f, %f.\n", validatedHoleFilling[0],
              validatedHoleFilling[1], validatedHoleFilling[2]);
  std::fflush(stdout);
  EKF_Node_anonFcn3(psi_m, dv2);
  EKF_Node_anonFcn2(theta_m, dv3);
  EKF_Node_anonFcn1(phi_m, dv4);
  for (int i{0}; i < 3; i++) {
    mFL_tmp = dv2[i];
    pFR_tmp = dv2[i + 3];
    b_pFR_tmp = dv2[i + 6];
    for (int i1{0}; i1 < 3; i1++) {
      dv5[i + 3 * i1] = (mFL_tmp * dv3[3 * i1] + pFR_tmp * dv3[3 * i1 + 1]) +
                        b_pFR_tmp * dv3[3 * i1 + 2];
    }
    mFL_tmp = 0.0;
    pFR_tmp = dv5[i];
    b_pFR_tmp = dv5[i + 3];
    pRR_tmp = dv5[i + 6];
    for (int i1{0}; i1 < 3; i1++) {
      d = (pFR_tmp * dv4[3 * i1] + b_pFR_tmp * dv4[3 * i1 + 1]) +
          pRR_tmp * dv4[3 * i1 + 2];
      C[i + 3 * i1] = d;
      mFL_tmp += d * VEC_OFF[i1];
    }
    pos_enu_m[i] = mFL_tmp + pos_enu_gnss[i];
  }
  double dotUVW[3];
  double lla0_vc[3];
  //  position of VC in local ENU frame
  coder::internal::validate_print_arguments(pos_enu_m[0], pos_enu_m[1],
                                            pos_enu_m[2], validatedHoleFilling);
  std::printf("Initial Position: %f, %f, %f.\n", validatedHoleFilling[0],
              validatedHoleFilling[1], validatedHoleFilling[2]);
  std::fflush(stdout);
  validatedHoleFilling[0] = pos_enu_m[0];
  validatedHoleFilling[1] = pos_enu_m[1];
  validatedHoleFilling[2] = pos_enu_m[2];
  coder::enu2lla(validatedHoleFilling, lla0, lla0_vc);
  coder::internal::validate_print_arguments(lla0_vc[0], lla0_vc[1], lla0_vc[2],
                                            validatedHoleFilling);
  std::printf("Initial Position: %f, %f, %f.\n", validatedHoleFilling[0],
              validatedHoleFilling[1], validatedHoleFilling[2]);
  std::fflush(stdout);
  // initilize EKF for C-Code generation
  validatedHoleFilling[0] = dotE_m;
  validatedHoleFilling[1] = dotN_m;
  validatedHoleFilling[2] = dotU_m;
  coder::internal::blas::mtimes(C, validatedHoleFilling, dotUVW);
  // X
  // Y
  // Z
  // odom_msg.Twist.Twist.Linear.X;   %u
  // odom_msg.Twist.Twist.Linear.Y;   %v
  // odom_msg.Twist.Twist.Linear.Z;   %w
  // imu_msg.Orientation.X;      %roll rate
  // imu_msg.Orientation.Y;      %pitch rate
  // imu_msg.Orientation.Z;     %yaw rate
  //  initialize the state vector
  std::memset(&xkm1[0], 0, 70U * sizeof(double));
  xkm1[0] = pos_enu_m[0];
  xkm1[1] = pos_enu_m[1];
  xkm1[2] = pos_enu_m[2];
  xkm1[3] = dotUVW[0];
  xkm1[4] = dotUVW[1];
  xkm1[5] = dotUVW[2];
  xkm1[6] = phi_m;
  xkm1[7] = theta_m;
  xkm1[8] = psi_m;
  xkm1[9] = 0.0;
  xkm1[10] = 0.0;
  xkm1[11] = 0.0;
  xkm1[12] = curv;
  xkm1[static_cast<int>(nd + 1.0) - 1] = TGC;
  //  include TGC coefficient as parameter in EKF
  //  first time call to EKF_Implement is here.
  std::copy(&xkm1[0], &xkm1[70], &myFilter.pState[0]);
  //  zero initial commands
  Kcmd = 0.0;
  acmd = 0.0;
  //  % important flag for EKF object initialization
  //  init = 1;
  //  define a fixed cycle rate for EKF node
  rr.init();
  EKF_msg.Twist.Covariance[1] = lla0_vc[0];
  EKF_msg.Twist.Covariance[2] = lla0_vc[1];
  EKF_msg.Twist.Covariance[3] = lla0_vc[2];
  while (1) {
    double vec_u[3];
    double psi_modulo;
    int exitg1;
    coder::lla2enu(pos_gnss_lla, lla0, pos_enu_gnss);
    // pos_enu_gnss = [ll2utm(pos_gnss_lla(1), pos_gnss_lla(2)),
    // pos_gnss_lla(3)];
    EKF_Node_anonFcn3(psi_m, dv2);
    EKF_Node_anonFcn2(theta_m, dv3);
    EKF_Node_anonFcn1(phi_m, dv4);
    for (int i{0}; i < 3; i++) {
      mFL_tmp = dv2[i];
      pFR_tmp = dv2[i + 3];
      b_pFR_tmp = dv2[i + 6];
      for (int i1{0}; i1 < 3; i1++) {
        dv5[i + 3 * i1] = (mFL_tmp * dv3[3 * i1] + pFR_tmp * dv3[3 * i1 + 1]) +
                          b_pFR_tmp * dv3[3 * i1 + 2];
      }
      mFL_tmp = 0.0;
      pFR_tmp = dv5[i];
      b_pFR_tmp = dv5[i + 3];
      pRR_tmp = dv5[i + 6];
      for (int i1{0}; i1 < 3; i1++) {
        d = (pFR_tmp * dv4[3 * i1] + b_pFR_tmp * dv4[3 * i1 + 1]) +
            pRR_tmp * dv4[3 * i1 + 2];
        dv2[i + 3 * i1] = d;
        mFL_tmp += d * VEC_OFF[i1];
      }
      pos_enu_m[i] = mFL_tmp + pos_enu_gnss[i];
    }
    //  position of VC in local ENU frame
    //  pack the measurements vector
    mFL_tmp = stg;
    coder::b_tand(mFL_tmp);
    ym[0] = delta_x_FR;
    ym[1] = delta_x_FL;
    ym[2] = delta_x_RR;
    ym[3] = delta_x_RL;
    ym[4] = pos_enu_m[0];
    ym[5] = pos_enu_m[1];
    ym[6] = phi_m;
    ym[7] = theta_m;
    ym[9] = dotE_m;
    ym[10] = dotN_m;
    ym[11] = dotU_m;
    ym[12] = V_ODOM;
    ym[13] = mFL_tmp / l;
    //  x
    //  y
    //  roll
    //  pitch
    //  psi_modulo;  % heading
    //  East velocity
    //  North velocity
    //  Up velocity
    //  ground velocity -- use EPEC data
    //  measured curvature from steering angle measurements -- use EPEC data
    //  fix heading jumps
    psi_modulo = psi_m;
    do {
      exitg1 = 0;
      std::copy(&myFilter.pState[0], &myFilter.pState[70], &b_value[0]);
      if (psi_modulo - b_value[8] > 3.1415926535897931) {
        psi_modulo -= 6.2831853071795862;
      } else {
        exitg1 = 1;
      }
    } while (exitg1 == 0);
    do {
      exitg1 = 0;
      std::copy(&myFilter.pState[0], &myFilter.pState[70], &b_value[0]);
      if (psi_modulo - b_value[8] < -3.1415926535897931) {
        psi_modulo += 6.2831853071795862;
      } else {
        exitg1 = 1;
      }
    } while (exitg1 == 0);
    ym[8] = psi_modulo;
    //  compute the curvature command
    mFL_tmp = stg_cmd;
    coder::b_tand(mFL_tmp);
    Kcmd = mFL_tmp / l;
    //  % use the measured speed from EPEC and command from ROS to compute
    //  % the force acceleration command
    std::copy(&myFilter.pState[0], &myFilter.pState[70], &b_value[0]);
    //  speed command to acceleration command model
    //  acmd = a*v + b*vcmd, where we have
    //  a = -0.515; b = -1.0*a; c = g*Cr;
    //  xk is the state(4), uk is V_CMD
    if (std::isnan(b_value[3])) {
      d = rtNaN;
    } else if (b_value[3] < 0.0) {
      d = -1.0;
    } else {
      d = (b_value[3] > 0.0);
    }
    acc = (acc_a * b_value[3] + acc_b * V_cmd) + acc_c * d;
    // acc_model(0, V_ODOM, V_cmd);
    if (acc * V_cmd <= 0.0) {
      acc = 0.0;
    }
    acmd = acc;
    //  pack the inputs
    vec_u[0] = Kcmd;
    vec_u[1] = acmd;
    vec_u[2] = V_cmd;
    myFilter.correct(ym);
    std::copy(&myFilter.pState[0], &myFilter.pState[70], &b_value[0]);
    if (neq < 1.0) {
      u0 = 0;
    } else {
      u0 = static_cast<int>(neq);
    }
    if (neq < 1.0) {
      input_sizes_idx_1 = 0;
    } else {
      input_sizes_idx_1 = static_cast<int>(neq);
    }
    myFilter.get_StateCovariance(dv6);
    b_result.set_size(u0, input_sizes_idx_1);
    for (int i{0}; i < input_sizes_idx_1; i++) {
      for (int i1{0}; i1 < u0; i1++) {
        b_result[i1 + b_result.size(0) * i] = dv6[i1 + 70 * i];
      }
    }
    double Quarternion[4];
    coder::diag(b_result, P_est);
    myFilter.predict(vec_u);
    //  X_est = myFilter.State(1:neq);
    EKF_msg.Pose.Pose.Position.X = b_value[0];
    //  X -- local ENU
    EKF_msg.Pose.Pose.Position.Y = b_value[1];
    //  Y
    EKF_msg.Pose.Pose.Position.Z = b_value[2];
    //  VC Height
    EKF_msg.Twist.Twist.Linear.X = b_value[3];
    //  u -- forward velocity
    EKF_msg.Twist.Twist.Linear.Y = b_value[4];
    //  v -- sideways velocity
    EKF_msg.Twist.Twist.Linear.Z = b_value[5];
    //  w -- up velocity
    EKF_msg.Twist.Twist.Angular.X = b_value[9];
    //  p -- roll rate
    EKF_msg.Twist.Twist.Angular.Y = b_value[10];
    //  q -- pitch rate
    EKF_msg.Twist.Twist.Angular.Z = b_value[11];
    //  r -- yaw rate
    validatedHoleFilling[0] = b_value[8];
    validatedHoleFilling[1] = b_value[7];
    validatedHoleFilling[2] = b_value[6];
    coder::eul2quat(validatedHoleFilling, Quarternion);
    EKF_msg.Pose.Pose.Orientation.X = Quarternion[1];
    EKF_msg.Pose.Pose.Orientation.Y = Quarternion[2];
    EKF_msg.Pose.Pose.Orientation.Z = Quarternion[3];
    EKF_msg.Pose.Pose.Orientation.W = Quarternion[0];
    // temp = [ym; P_est];
    //  put the measurement vector in covariance field -- debugging
    for (input_sizes_idx_1 = 0; input_sizes_idx_1 < 14; input_sizes_idx_1++) {
      EKF_msg.Pose.Covariance[input_sizes_idx_1] = ym[input_sizes_idx_1];
      EKF_msg.Pose.Covariance[input_sizes_idx_1 + 14] =
          P_est[input_sizes_idx_1];
    }
    //  store the diagonal covariance values
    EKF_msg.Twist.Covariance[0] = b_value[13];
    //  put the TGC here
    EKF_msg.Twist.Covariance[4] = b_value[6];
    //  roll
    EKF_msg.Twist.Covariance[5] = b_value[7];
    //  pitch
    EKF_msg.Twist.Covariance[6] = b_value[8];
    //  heading
    EKF_msg.Twist.Covariance[7] = b_value[12];
    //  curvature state
    //  Kcmd acmd V_cmd
    EKF_msg.Twist.Covariance[8] = Kcmd;
    //  curvature command
    EKF_msg.Twist.Covariance[9] = acmd;
    //  Acceleration command
    EKF_msg.Twist.Covariance[10] = V_cmd;
    //  Velocity command
    //  Original lon lat
    EKF_msg.Twist.Covariance[11] = pos_gnss_lla[0];
    //  lat
    EKF_msg.Twist.Covariance[12] = pos_gnss_lla[1];
    //  lon
    EKF_msg.Twist.Covariance[13] = lla0[0];
    //  lat
    EKF_msg.Twist.Covariance[14] = lla0[1];
    //  lon
    EKF_msg.Twist.Covariance[15] = lla0[2];
    //  lon
    //  put the steering/cruvature commands in angular part and acceleration
    //  command in the linear part of the geomtery_msgs/Accel type message
    // CMD_msg.Linear.X = acmd;
    // CMD_msg.Linear.Y = V_cmd;
    // CMD_msg.Linear.Z = sqrt(X_est(4)*X_est(4) +...
    //     X_est(5)*X_est(5) + X_est(6)*X_est(6)); % measured speed from state
    //     vector
    // CMD_msg.Angular.X = stg_cmd;
    // CMD_msg.Angular.Y = Kcmd;
    // CMD_msg.Angular.Z = X_est(13); % curvature from state
    MATLABPUBLISHER_publish(EKF_out.PublisherHelper, &EKF_msg);
    // send(CMD_out,CMD_msg);
    MATLABRate_sleep(rr.RateHelper);
    coder::toc(rr.PreviousPeriod.tv_sec, rr.PreviousPeriod.tv_nsec);
    rr.PreviousPeriod.tv_sec = coder::tic(rr.PreviousPeriod.tv_nsec);
  }
}

// End of code generation (EKF_Node.cpp)
