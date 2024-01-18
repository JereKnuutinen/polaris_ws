//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// EKF_Node_data.h
//
// Code generation for function 'EKF_Node_data'
//

#ifndef EKF_NODE_DATA_H
#define EKF_NODE_DATA_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Variable Declarations
extern double delta_x_FR;
extern double delta_x_FL;
extern double delta_x_RR;
extern double delta_x_RL;
extern double phi_m;
extern double theta_m;
extern double psi_m;
extern double dotE_m;
extern double dotN_m;
extern double dotU_m;
extern double pos_gnss_lla[3];
extern double stg;
extern double curv;
extern double stg_cmd;
extern double V_cmd;
extern double acmd;
extern double Kcmd;
extern double TGC;
extern double Ca;
extern double g;
extern double Cr;
extern double D2R;
extern double R2D;
extern double acc_a;
extern double acc_b;
extern double acc_c;
extern double str_a;
extern double str_b;
extern double mFL;
extern double mFR;
extern double mRL;
extern double mRR;
extern double m;
extern double l;
extern double tw;
extern double h;
extern double h_T;
extern double Ixx;
extern double Iyy;
extern double Izz;
extern double kFL;
extern double kFR;
extern double kRL;
extern double kRR;
extern double xoff;
extern double yoff;
extern double zoff;
extern double cFL;
extern double cFR;
extern double cRL;
extern double cRR;
extern double pFR[3];
extern double pRR[3];
extern double pFL[3];
extern double pRL[3];
extern double roll_bias;
extern double pitch_bias;
extern double VEC_OFF[3];
extern double dt2;
extern double D_Epec;
extern double D_Novatel;
extern double na;
extern double nd;
extern double np;
extern double ny;
extern double nu;
extern double neq;
extern double ns;
extern double Q_t[4900];
extern double R_t[196];
extern double Pkm1[4900];
extern double lla0[3];
extern double pos_enu_gnss[3];
extern double pos_enu_m[3];
extern double xkm1[70];
extern double V_ODOM;
extern double acc;
extern double freq;
extern bool freq_not_empty;
extern const double dv[196];
extern const signed char iv[9];
extern bool isInitialized_EKF_Node;

#endif
// End of code generation (EKF_Node_data.h)
