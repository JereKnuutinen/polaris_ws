//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// NMPC_Node_internal_types2.h
//
// Code generation for function 'NMPC_Node_internal_types2'
//

#ifndef NMPC_NODE_INTERNAL_TYPES2_H
#define NMPC_NODE_INTERNAL_TYPES2_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Definitions
struct j_struct_T {
  double x[14];
  double lastMV[2];
  double ref[21];
  double OutputWeights[21];
  double MVWeights[14];
  double MVRateWeights[14];
  double ECRWeight;
  double OutputMin[21];
  double OutputMax[21];
  double StateMin[98];
  double StateMax[98];
  double MVMin[14];
  double MVMax[14];
  double MVRateMin[14];
  double MVRateMax[14];
  double MVScaledTarget[14];
  double Parameters[1];
};

struct k_struct_T {
  double Ts;
  double CurrentStates[14];
  double LastMV[2];
  double References[21];
  double MVTarget[14];
  double PredictionHorizon;
  double NumOfStates;
  double NumOfOutputs;
  double NumOfInputs;
  double MVIndex[2];
  double InputPassivityIndex;
  double OutputPassivityIndex;
  bool PassivityUsePredictedX;
};

#endif
// End of code generation (NMPC_Node_internal_types2.h)
