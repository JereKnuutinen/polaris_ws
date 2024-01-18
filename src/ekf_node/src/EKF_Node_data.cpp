//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// EKF_Node_data.cpp
//
// Code generation for function 'EKF_Node_data'
//

// Include files
#include "EKF_Node_data.h"
#include "rt_nonfinite.h"

// Variable Definitions
double delta_x_FR;

double delta_x_FL;

double delta_x_RR;

double delta_x_RL;

double phi_m;

double theta_m;

double psi_m;

double dotE_m;

double dotN_m;

double dotU_m;

double pos_gnss_lla[3];

double stg;

double curv;

double stg_cmd;

double V_cmd;

double acmd;

double Kcmd;

double TGC;

double Ca;

double g;

double Cr;

double D2R;

double R2D;

double acc_a;

double acc_b;

double acc_c;

double str_a;

double str_b;

double mFL;

double mFR;

double mRL;

double mRR;

double m;

double l;

double tw;

double h;

double h_T;

double Ixx;

double Iyy;

double Izz;

double kFL;

double kFR;

double kRL;

double kRR;

double xoff;

double yoff;

double zoff;

double cFL;

double cFR;

double cRL;

double cRR;

double pFR[3];

double pRR[3];

double pFL[3];

double pRL[3];

double roll_bias;

double pitch_bias;

double VEC_OFF[3];

double dt2;

double D_Epec;

double D_Novatel;

double na;

double nd;

double np;

double ny;

double nu;

double neq;

double ns;

double Q_t[4900];

double R_t[196];

double Pkm1[4900];

double lla0[3];

double pos_enu_gnss[3];

double pos_enu_m[3];

double xkm1[70];

double V_ODOM;

double acc;

double freq;

bool freq_not_empty;

const double dv[196]{1.0000000000000002E-10,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     1.0000000000000002E-10,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     1.0000000000000002E-10,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     1.0000000000000002E-10,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     1.0E-12,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     1.0E-12,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     4.0E-12,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     4.0E-12,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     3.6000000000000005E-11,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     4.8999999999999992E-9,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     4.9E-11,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     4.9E-11,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     1.0E-12,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     1.0E-12};

const signed char iv[9]{0, 1, 0, 1, 0, 0, 0, 0, -1};

bool isInitialized_EKF_Node{false};

// End of code generation (EKF_Node_data.cpp)
