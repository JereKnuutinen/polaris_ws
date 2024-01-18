//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// extendedKalmanFilter.cpp
//
// Code generation for function 'extendedKalmanFilter'
//

// Include files
#include "extendedKalmanFilter.h"
#include "EKFCorrectorAdditive.h"
#include "EKF_Node_rtwutil.h"
#include "aug_dynamic_model.h"
#include "cholPSD.h"
#include "measurement_model2.h"
#include "rt_nonfinite.h"
#include "svd.h"
#include "xgemv.h"
#include "xgerc.h"
#include "xnrm2.h"
#include "xpotrf.h"
#include <algorithm>
#include <cmath>
#include <cstring>

// Variable Definitions
static const signed char iv1[4900]{
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1};

// Function Definitions
namespace coder {
void extendedKalmanFilter::correct(const double z[14])
{
  static const signed char b[196]{
      1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1};
  double obj[4900];
  double M[1176];
  double dHdx[980];
  double y[980];
  double R[196];
  double b_R[196];
  double imvec[70];
  double b_z[14];
  double imz[14];
  double work[14];
  double d;
  double epsilon;
  int coffset;
  int i;
  int i1;
  int k;
  int knt;
  if ((!pIsSetStateCovariance) || (pSqrtStateCovarianceScalar != -1.0)) {
    epsilon = pSqrtStateCovarianceScalar;
    for (i = 0; i < 4900; i++) {
      pSqrtStateCovariance[i] = epsilon * static_cast<double>(iv1[i]);
    }
  }
  if (pIsFirstCallCorrect) {
    if (!pIsValidMeasurementFcn) {
      measurement_model2(pState, b_z);
      pIsValidMeasurementFcn = true;
    }
    pIsFirstCallCorrect = false;
  }
  if ((!pIsSetMeasurementNoise) || (pSqrtMeasurementNoiseScalar != -1.0)) {
    epsilon = pSqrtMeasurementNoiseScalar;
    for (i = 0; i < 196; i++) {
      pSqrtMeasurementNoise[i] = epsilon * static_cast<double>(b[i]);
    }
    pIsSetMeasurementNoise = true;
    pSqrtMeasurementNoiseScalar = -1.0;
  }
  measurement_model2(pState, b_z);
  for (int j{0}; j < 70; j++) {
    std::copy(&pState[0], &pState[70], &imvec[0]);
    d = pState[j];
    epsilon =
        std::fmax(1.4901161193847656E-8, 1.4901161193847656E-8 * std::abs(d));
    imvec[j] = d + epsilon;
    measurement_model2(imvec, imz);
    for (i = 0; i < 14; i++) {
      dHdx[i + 14 * j] = (imz[i] - b_z[i]) / epsilon;
    }
  }
  measurement_model2(pState, b_z);
  for (int j{0}; j < 14; j++) {
    coffset = j * 70;
    std::memset(&y[coffset], 0, 70U * sizeof(double));
    for (k = 0; k < 70; k++) {
      epsilon = dHdx[k * 14 + j];
      for (int b_i{0}; b_i < 70; b_i++) {
        knt = coffset + b_i;
        y[knt] += pSqrtStateCovariance[b_i * 70 + k] * epsilon;
      }
    }
  }
  for (int b_i{0}; b_i < 14; b_i++) {
    std::copy(&y[b_i * 70],
              &y[static_cast<int>(static_cast<unsigned int>(b_i * 70) + 70U)],
              &M[b_i * 84]);
    for (i = 0; i < 14; i++) {
      M[(i + 84 * b_i) + 70] = pSqrtMeasurementNoise[b_i + 14 * i];
    }
    imz[b_i] = 0.0;
    work[b_i] = 0.0;
  }
  for (int b_i{0}; b_i < 14; b_i++) {
    double atmp;
    int ii;
    ii = b_i * 84 + b_i;
    atmp = M[ii];
    coffset = ii + 2;
    imz[b_i] = 0.0;
    epsilon = internal::blas::e_xnrm2(83 - b_i, M, ii + 2);
    if (epsilon != 0.0) {
      double beta1;
      d = M[ii];
      beta1 = rt_hypotd_snf(d, epsilon);
      if (d >= 0.0) {
        beta1 = -beta1;
      }
      if (std::abs(beta1) < 1.0020841800044864E-292) {
        knt = 0;
        i = (ii - b_i) + 84;
        do {
          knt++;
          for (k = coffset; k <= i; k++) {
            M[k - 1] *= 9.9792015476736E+291;
          }
          beta1 *= 9.9792015476736E+291;
          atmp *= 9.9792015476736E+291;
        } while ((std::abs(beta1) < 1.0020841800044864E-292) && (knt < 20));
        beta1 =
            rt_hypotd_snf(atmp, internal::blas::e_xnrm2(83 - b_i, M, ii + 2));
        if (atmp >= 0.0) {
          beta1 = -beta1;
        }
        imz[b_i] = (beta1 - atmp) / beta1;
        epsilon = 1.0 / (atmp - beta1);
        for (k = coffset; k <= i; k++) {
          M[k - 1] *= epsilon;
        }
        for (k = 0; k < knt; k++) {
          beta1 *= 1.0020841800044864E-292;
        }
        atmp = beta1;
      } else {
        imz[b_i] = (beta1 - d) / beta1;
        epsilon = 1.0 / (d - beta1);
        i = (ii - b_i) + 84;
        for (k = coffset; k <= i; k++) {
          M[k - 1] *= epsilon;
        }
        atmp = beta1;
      }
    }
    M[ii] = atmp;
    if (b_i + 1 < 14) {
      int lastc;
      int lastv;
      M[ii] = 1.0;
      if (imz[b_i] != 0.0) {
        bool exitg2;
        lastv = 84 - b_i;
        coffset = (ii - b_i) + 83;
        while ((lastv > 0) && (M[coffset] == 0.0)) {
          lastv--;
          coffset--;
        }
        lastc = 12 - b_i;
        exitg2 = false;
        while ((!exitg2) && (lastc + 1 > 0)) {
          int exitg1;
          coffset = (ii + lastc * 84) + 84;
          k = coffset;
          do {
            exitg1 = 0;
            if (k + 1 <= coffset + lastv) {
              if (M[k] != 0.0) {
                exitg1 = 1;
              } else {
                k++;
              }
            } else {
              lastc--;
              exitg1 = 2;
            }
          } while (exitg1 == 0);
          if (exitg1 == 1) {
            exitg2 = true;
          }
        }
      } else {
        lastv = 0;
        lastc = -1;
      }
      if (lastv > 0) {
        knt = ii + 85;
        if (lastc + 1 != 0) {
          std::memset(&work[0], 0,
                      static_cast<unsigned int>(lastc + 1) * sizeof(double));
          i = (ii + 84 * lastc) + 85;
          for (int j{knt}; j <= i; j += 84) {
            epsilon = 0.0;
            i1 = (j + lastv) - 1;
            for (k = j; k <= i1; k++) {
              epsilon += M[k - 1] * M[(ii + k) - j];
            }
            coffset = div_nde_s32_floor((j - ii) - 85, 84);
            work[coffset] += epsilon;
          }
        }
        if (!(-imz[b_i] == 0.0)) {
          coffset = ii;
          for (int j{0}; j <= lastc; j++) {
            d = work[j];
            if (d != 0.0) {
              epsilon = d * -imz[b_i];
              i = coffset + 85;
              i1 = lastv + coffset;
              for (knt = i; knt <= i1 + 84; knt++) {
                M[knt - 1] += M[((ii + knt) - coffset) - 85] * epsilon;
              }
            }
            coffset += 84;
          }
        }
      }
      M[ii] = atmp;
    }
  }
  for (int j{0}; j < 14; j++) {
    for (int b_i{0}; b_i <= j; b_i++) {
      R[b_i + 14 * j] = M[b_i + 84 * j];
    }
    i = j + 2;
    if (i <= 14) {
      std::memset(&R[(j * 14 + i) + -1], 0,
                  static_cast<unsigned int>(-i + 15) * sizeof(double));
    }
  }
  for (int b_i{0}; b_i < 70; b_i++) {
    imvec[b_i] = pState[b_i];
    for (i = 0; i < 70; i++) {
      d = 0.0;
      for (i1 = 0; i1 < 70; i1++) {
        d += pSqrtStateCovariance[b_i + 70 * i1] *
             pSqrtStateCovariance[i + 70 * i1];
      }
      obj[b_i + 70 * i] = d;
    }
  }
  for (i = 0; i < 14; i++) {
    b_z[i] = z[i] - b_z[i];
  }
  for (i = 0; i < 70; i++) {
    for (i1 = 0; i1 < 14; i1++) {
      d = 0.0;
      for (coffset = 0; coffset < 70; coffset++) {
        d += obj[i + 70 * coffset] * dHdx[i1 + 14 * coffset];
      }
      y[i + 70 * i1] = d;
    }
  }
  for (i = 0; i < 14; i++) {
    for (i1 = 0; i1 < 14; i1++) {
      b_R[i1 + 14 * i] = R[i + 14 * i1];
    }
  }
  matlabshared::tracking::internal::EKFCorrectorAdditive::
      correctStateAndSqrtCovariance(imvec, pSqrtStateCovariance, b_z, y, b_R,
                                    dHdx, pSqrtMeasurementNoise);
  std::copy(&imvec[0], &imvec[70], &pState[0]);
  pIsSetStateCovariance = true;
  pSqrtStateCovarianceScalar = -1.0;
}

void extendedKalmanFilter::get_StateCovariance(double b_value[4900])
{
  double a;
  if ((!pIsSetStateCovariance) || (pSqrtStateCovarianceScalar != -1.0)) {
    a = pSqrtStateCovarianceScalar;
    for (int i{0}; i < 4900; i++) {
      pSqrtStateCovariance[i] = a * static_cast<double>(iv1[i]);
    }
    pIsSetStateCovariance = true;
    pSqrtStateCovarianceScalar = -1.0;
  }
  for (int i{0}; i < 70; i++) {
    for (int i1{0}; i1 < 70; i1++) {
      a = 0.0;
      for (int i2{0}; i2 < 70; i2++) {
        a += pSqrtStateCovariance[i + 70 * i2] *
             pSqrtStateCovariance[i1 + 70 * i2];
      }
      b_value[i + 70 * i1] = a;
    }
  }
}

extendedKalmanFilter *extendedKalmanFilter::init()
{
  extendedKalmanFilter *EKF;
  EKF = this;
  EKF->pIsFirstCallPredict = true;
  EKF->pIsFirstCallCorrect = true;
  EKF->pSqrtStateCovarianceScalar = 1.0;
  EKF->pIsValidStateTransitionFcn = false;
  EKF->pIsValidMeasurementFcn = false;
  EKF->pIsValidMeasurementFcn = false;
  EKF->pIsValidStateTransitionFcn = false;
  EKF->pSqrtProcessNoiseScalar = 1.0;
  EKF->pSqrtMeasurementNoiseScalar = 1.0;
  return EKF;
}

void extendedKalmanFilter::predict(const double varargin_1[3])
{
  double A[9800];
  double C[4900];
  double dFdx[4900];
  double imvec[70];
  double z[70];
  double f1[14];
  double f2[14];
  double f3[14];
  double f4[14];
  double obj[14];
  double unusedExpr[14];
  double d;
  double epsilon;
  int coffset;
  int j;
  int knt;
  int lastv;
  if ((!pIsSetStateCovariance) || (pSqrtStateCovarianceScalar != -1.0)) {
    epsilon = pSqrtStateCovarianceScalar;
    for (lastv = 0; lastv < 4900; lastv++) {
      pSqrtStateCovariance[lastv] = epsilon * static_cast<double>(iv1[lastv]);
    }
  }
  if ((!pIsSetProcessNoise) || (pSqrtProcessNoiseScalar != -1.0)) {
    epsilon = pSqrtProcessNoiseScalar;
    for (lastv = 0; lastv < 4900; lastv++) {
      pSqrtProcessNoise[lastv] = epsilon * static_cast<double>(iv1[lastv]);
    }
    pIsSetProcessNoise = true;
    pSqrtProcessNoiseScalar = -1.0;
  }
  if (pIsFirstCallPredict) {
    if (!pIsValidStateTransitionFcn) {
      // global ns neq dt2
      aug_dynamic_model_anonFcn1(varargin_1, &pState[0], f1);
      for (lastv = 0; lastv < 14; lastv++) {
        obj[lastv] = pState[lastv] + 0.025 * f1[lastv];
      }
      aug_dynamic_model_anonFcn1(varargin_1, obj, f2);
      for (lastv = 0; lastv < 14; lastv++) {
        obj[lastv] = pState[lastv] + 0.025 * f2[lastv];
      }
      aug_dynamic_model_anonFcn1(varargin_1, obj, f3);
      for (lastv = 0; lastv < 14; lastv++) {
        obj[lastv] = pState[lastv] + 0.05 * f3[lastv];
      }
      aug_dynamic_model_anonFcn1(varargin_1, obj, unusedExpr);
      pIsValidStateTransitionFcn = true;
    }
    pIsFirstCallPredict = false;
  }
  // global ns neq dt2
  aug_dynamic_model_anonFcn1(varargin_1, &pState[0], f1);
  for (lastv = 0; lastv < 14; lastv++) {
    obj[lastv] = pState[lastv] + 0.025 * f1[lastv];
  }
  aug_dynamic_model_anonFcn1(varargin_1, obj, f2);
  for (lastv = 0; lastv < 14; lastv++) {
    obj[lastv] = pState[lastv] + 0.025 * f2[lastv];
  }
  aug_dynamic_model_anonFcn1(varargin_1, obj, f3);
  for (lastv = 0; lastv < 14; lastv++) {
    obj[lastv] = pState[lastv] + 0.05 * f3[lastv];
  }
  aug_dynamic_model_anonFcn1(varargin_1, obj, f4);
  for (int i{0}; i < 14; i++) {
    z[i] = pState[i] + 0.0083333333333333332 *
                           (((f1[i] + 2.0 * f2[i]) + 2.0 * f3[i]) + f4[i]);
  }
  std::copy(&pState[0], &pState[56], &z[14]);
  for (j = 0; j < 70; j++) {
    std::copy(&pState[0], &pState[70], &imvec[0]);
    d = pState[j];
    epsilon =
        std::fmax(1.4901161193847656E-8, 1.4901161193847656E-8 * std::abs(d));
    imvec[j] = d + epsilon;
    // global ns neq dt2
    aug_dynamic_model_anonFcn1(varargin_1, &imvec[0], f1);
    for (lastv = 0; lastv < 14; lastv++) {
      obj[lastv] = imvec[lastv] + 0.025 * f1[lastv];
    }
    aug_dynamic_model_anonFcn1(varargin_1, obj, f2);
    for (lastv = 0; lastv < 14; lastv++) {
      obj[lastv] = imvec[lastv] + 0.025 * f2[lastv];
    }
    aug_dynamic_model_anonFcn1(varargin_1, obj, f3);
    for (lastv = 0; lastv < 14; lastv++) {
      obj[lastv] = imvec[lastv] + 0.05 * f3[lastv];
    }
    aug_dynamic_model_anonFcn1(varargin_1, obj, f4);
    for (lastv = 0; lastv < 14; lastv++) {
      dFdx[lastv + 70 * j] =
          ((imvec[lastv] +
            0.0083333333333333332 *
                (((f1[lastv] + 2.0 * f2[lastv]) + 2.0 * f3[lastv]) +
                 f4[lastv])) -
           z[lastv]) /
          epsilon;
    }
    for (lastv = 0; lastv < 56; lastv++) {
      dFdx[(lastv + 70 * j) + 14] = (imvec[lastv] - z[lastv + 14]) / epsilon;
    }
  }
  // global ns neq dt2
  aug_dynamic_model_anonFcn1(varargin_1, &pState[0], f1);
  for (lastv = 0; lastv < 14; lastv++) {
    obj[lastv] = pState[lastv] + 0.025 * f1[lastv];
  }
  aug_dynamic_model_anonFcn1(varargin_1, obj, f2);
  for (lastv = 0; lastv < 14; lastv++) {
    obj[lastv] = pState[lastv] + 0.025 * f2[lastv];
  }
  aug_dynamic_model_anonFcn1(varargin_1, obj, f3);
  for (lastv = 0; lastv < 14; lastv++) {
    obj[lastv] = pState[lastv] + 0.05 * f3[lastv];
  }
  aug_dynamic_model_anonFcn1(varargin_1, obj, f4);
  for (j = 0; j < 70; j++) {
    coffset = j * 70;
    std::memset(&C[coffset], 0, 70U * sizeof(double));
    for (int k{0}; k < 70; k++) {
      epsilon = dFdx[k * 70 + j];
      for (int i{0}; i < 70; i++) {
        knt = coffset + i;
        C[knt] += pSqrtStateCovariance[i * 70 + k] * epsilon;
      }
    }
  }
  for (int i{0}; i < 70; i++) {
    for (lastv = 0; lastv < 70; lastv++) {
      coffset = lastv + 140 * i;
      A[coffset] = C[lastv + 70 * i];
      A[coffset + 70] = pSqrtProcessNoise[i + 70 * lastv];
    }
    imvec[i] = 0.0;
  }
  for (int i{0}; i < 70; i++) {
    double atmp;
    int ii;
    ii = i * 140 + i;
    atmp = A[ii];
    coffset = ii + 2;
    z[i] = 0.0;
    epsilon = internal::blas::g_xnrm2(139 - i, A, ii + 2);
    if (epsilon != 0.0) {
      double beta1;
      d = A[ii];
      beta1 = rt_hypotd_snf(d, epsilon);
      if (d >= 0.0) {
        beta1 = -beta1;
      }
      if (std::abs(beta1) < 1.0020841800044864E-292) {
        knt = 0;
        lastv = (ii - i) + 140;
        do {
          knt++;
          for (int k{coffset}; k <= lastv; k++) {
            A[k - 1] *= 9.9792015476736E+291;
          }
          beta1 *= 9.9792015476736E+291;
          atmp *= 9.9792015476736E+291;
        } while ((std::abs(beta1) < 1.0020841800044864E-292) && (knt < 20));
        beta1 =
            rt_hypotd_snf(atmp, internal::blas::g_xnrm2(139 - i, A, ii + 2));
        if (atmp >= 0.0) {
          beta1 = -beta1;
        }
        z[i] = (beta1 - atmp) / beta1;
        epsilon = 1.0 / (atmp - beta1);
        for (int k{coffset}; k <= lastv; k++) {
          A[k - 1] *= epsilon;
        }
        for (int k{0}; k < knt; k++) {
          beta1 *= 1.0020841800044864E-292;
        }
        atmp = beta1;
      } else {
        z[i] = (beta1 - d) / beta1;
        epsilon = 1.0 / (d - beta1);
        lastv = (ii - i) + 140;
        for (int k{coffset}; k <= lastv; k++) {
          A[k - 1] *= epsilon;
        }
        atmp = beta1;
      }
    }
    A[ii] = atmp;
    if (i + 1 < 70) {
      A[ii] = 1.0;
      if (z[i] != 0.0) {
        bool exitg2;
        lastv = 140 - i;
        coffset = (ii - i) + 139;
        while ((lastv > 0) && (A[coffset] == 0.0)) {
          lastv--;
          coffset--;
        }
        coffset = 69 - i;
        exitg2 = false;
        while ((!exitg2) && (coffset > 0)) {
          int exitg1;
          knt = (ii + (coffset - 1) * 140) + 140;
          j = knt;
          do {
            exitg1 = 0;
            if (j + 1 <= knt + lastv) {
              if (A[j] != 0.0) {
                exitg1 = 1;
              } else {
                j++;
              }
            } else {
              coffset--;
              exitg1 = 2;
            }
          } while (exitg1 == 0);
          if (exitg1 == 1) {
            exitg2 = true;
          }
        }
      } else {
        lastv = 0;
        coffset = 0;
      }
      if (lastv > 0) {
        internal::blas::xgemv(lastv, coffset, A, ii + 141, A, ii + 1, imvec);
        internal::blas::xgerc(lastv, coffset, -z[i], ii + 1, imvec, A,
                              ii + 141);
      }
      A[ii] = atmp;
    }
  }
  for (j = 0; j < 70; j++) {
    for (int i{0}; i <= j; i++) {
      dFdx[i + 70 * j] = A[i + 140 * j];
    }
    lastv = j + 2;
    if (lastv <= 70) {
      std::memset(&dFdx[(j * 70 + lastv) + -1], 0,
                  static_cast<unsigned int>(-lastv + 71) * sizeof(double));
    }
  }
  for (lastv = 0; lastv < 14; lastv++) {
    z[lastv] =
        pState[lastv] +
        0.0083333333333333332 *
            (((f1[lastv] + 2.0 * f2[lastv]) + 2.0 * f3[lastv]) + f4[lastv]);
  }
  std::copy(&pState[0], &pState[56], &z[14]);
  for (lastv = 0; lastv < 70; lastv++) {
    pState[lastv] = z[lastv];
    for (coffset = 0; coffset < 70; coffset++) {
      pSqrtStateCovariance[coffset + 70 * lastv] = dFdx[lastv + 70 * coffset];
    }
  }
  pIsSetStateCovariance = true;
  pSqrtStateCovarianceScalar = -1.0;
}

void extendedKalmanFilter::set_MeasurementNoise(const double b_value[196])
{
  double Ss[196];
  int jmax;
  std::copy(&b_value[0], &b_value[196], &Ss[0]);
  jmax = internal::lapack::b_xpotrf(Ss);
  if (jmax == 0) {
    int i;
    std::copy(&b_value[0], &b_value[196], &Ss[0]);
    jmax = internal::lapack::b_xpotrf(Ss);
    if (jmax == 0) {
      jmax = 12;
    } else {
      jmax -= 3;
    }
    for (int j{0}; j <= jmax; j++) {
      i = j + 2;
      if (i <= jmax + 2) {
        std::memset(&Ss[(j * 14 + i) + -1], 0,
                    static_cast<unsigned int>((jmax - i) + 3) * sizeof(double));
      }
    }
    for (i = 0; i < 14; i++) {
      for (jmax = 0; jmax < 14; jmax++) {
        pSqrtMeasurementNoise[jmax + 14 * i] = Ss[i + 14 * jmax];
      }
    }
  } else {
    double V[196];
    double s[14];
    double d;
    bool p;
    p = true;
    for (jmax = 0; jmax < 196; jmax++) {
      if (p) {
        d = b_value[jmax];
        if (std::isinf(d) || std::isnan(d)) {
          p = false;
        }
      } else {
        p = false;
      }
    }
    if (p) {
      internal::b_svd(b_value, Ss, s, V);
    } else {
      for (jmax = 0; jmax < 14; jmax++) {
        s[jmax] = rtNaN;
      }
      for (int i{0}; i < 196; i++) {
        V[i] = rtNaN;
      }
    }
    std::memset(&Ss[0], 0, 196U * sizeof(double));
    for (jmax = 0; jmax < 14; jmax++) {
      Ss[jmax + 14 * jmax] = s[jmax];
    }
    for (jmax = 0; jmax < 196; jmax++) {
      Ss[jmax] = std::sqrt(Ss[jmax]);
    }
    for (int i{0}; i < 14; i++) {
      for (jmax = 0; jmax < 14; jmax++) {
        d = 0.0;
        for (int j{0}; j < 14; j++) {
          d += V[i + 14 * j] * Ss[j + 14 * jmax];
        }
        pSqrtMeasurementNoise[i + 14 * jmax] = d;
      }
    }
  }
  pIsSetMeasurementNoise = true;
  pSqrtMeasurementNoiseScalar = -1.0;
}

void extendedKalmanFilter::set_ProcessNoise(const double b_value[4900])
{
  matlabshared::tracking::internal::cholPSD(b_value, pSqrtProcessNoise);
  pIsSetProcessNoise = true;
  pSqrtProcessNoiseScalar = -1.0;
}

void extendedKalmanFilter::set_StateCovariance(const double b_value[4900])
{
  matlabshared::tracking::internal::cholPSD(b_value, pSqrtStateCovariance);
  pIsSetStateCovariance = true;
  pSqrtStateCovarianceScalar = -1.0;
}

} // namespace coder

// End of code generation (extendedKalmanFilter.cpp)
