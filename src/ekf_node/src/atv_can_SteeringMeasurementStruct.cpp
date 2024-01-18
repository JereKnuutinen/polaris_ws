//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// atv_can_SteeringMeasurementStruct.cpp
//
// Code generation for function 'atv_can_SteeringMeasurementStruct'
//

// Include files
#include "atv_can_SteeringMeasurementStruct.h"
#include "EKF_Node_types.h"
#include "ros_TimeStruct.h"
#include "rt_nonfinite.h"

// Function Definitions
atv_can_SteeringMeasurementStruct_T atv_can_SteeringMeasurementStruct()
{
  static const char cv[27]{'a', 't', 'v', '_', 'c', 'a', 'n', '/', 'S',
                           't', 'e', 'e', 'r', 'i', 'n', 'g', 'M', 'e',
                           'a', 's', 'u', 'r', 'e', 'm', 'e', 'n', 't'};
  atv_can_SteeringMeasurementStruct_T msg;
  //  Message struct definition for atv_can/SteeringMeasurement
  for (int i{0}; i < 27; i++) {
    msg.MessageType[i] = cv[i];
  }
  msg.EncoderPosition = 0U;
  msg.TimeReceived = ros_TimeStruct();
  //(&msg);
  return msg;
}

// End of code generation (atv_can_SteeringMeasurementStruct.cpp)
