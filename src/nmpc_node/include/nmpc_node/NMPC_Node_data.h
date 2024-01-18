//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// NMPC_Node_data.h
//
// Code generation for function 'NMPC_Node_data'
//

#ifndef NMPC_NODE_DATA_H
#define NMPC_NODE_DATA_H

// Include files
#include "NMPC_Node_types.h"
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Variable Declarations
extern double X_ekf[14];
extern double ref_test[44000];
extern double Index;
extern c_struct_T onlineData;
extern double dt;
extern double V_ODOM;
extern double freq;
extern bool freq_not_empty;
extern double m;
extern double pFL[3];
extern double pRL[3];
extern double pFR[3];
extern double pRR[3];
extern double kFR;
extern double cFR;
extern double kFL;
extern double cFL;
extern double kRR;
extern double cRR;
extern double kRL;
extern double cRL;
extern double tw;
extern double l;
extern double R2D;
extern double D2R;
extern double Ca;
extern double g;
extern double Cr;
extern double Izz;
extern double Iyy;
extern double Ixx;
extern double str_a;
extern double str_b;
extern const signed char iv[196];
extern bool isInitialized_NMPC_Node;

#endif
// End of code generation (NMPC_Node_data.h)
