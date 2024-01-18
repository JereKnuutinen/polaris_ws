//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// NMPC_Node_data.cpp
//
// Code generation for function 'NMPC_Node_data'
//

// Include files
#include "NMPC_Node_data.h"
#include "rt_nonfinite.h"
#include <cstring>

// Variable Definitions
double X_ekf[14];

double ref_test[44000];

double Index;

c_struct_T onlineData;

double dt;

double V_ODOM;

double freq;

bool freq_not_empty;

double m;

double pFL[3];

double pRL[3];

double pFR[3];

double pRR[3];

double kFR;

double cFR;

double kFL;

double cFL;

double kRR;

double cRR;

double kRL;

double cRL;

double tw;

double l;

double R2D;

double D2R;

double Ca;

double g;

double Cr;

double Izz;

double Iyy;

double Ixx;

double str_a;

double str_b;

const signed char iv[196]{
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1};

bool isInitialized_NMPC_Node{false};

// End of code generation (NMPC_Node_data.cpp)
