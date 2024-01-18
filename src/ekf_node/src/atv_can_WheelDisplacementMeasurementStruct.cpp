//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// atv_can_WheelDisplacementMeasurementStruct.cpp
//
// Code generation for function 'atv_can_WheelDisplacementMeasurementStruct'
//

// Include files
#include "atv_can_WheelDisplacementMeasurementStruct.h"
#include "EKF_Node_types.h"
#include "ros_TimeStruct.h"
#include "rt_nonfinite.h"

// Function Definitions
atv_can_WheelDisplacementMeasurementStruct_T
atv_can_WheelDisplacementMeasurementStruct()
{
  static const char cv[36]{'a', 't', 'v', '_', 'c', 'a', 'n', '/', 'W',
                           'h', 'e', 'e', 'l', 'D', 'i', 's', 'p', 'l',
                           'a', 'c', 'e', 'm', 'e', 'n', 't', 'M', 'e',
                           'a', 's', 'u', 'r', 'e', 'm', 'e', 'n', 't'};
  atv_can_WheelDisplacementMeasurementStruct_T msg;
  //  Message struct definition for atv_can/WheelDisplacementMeasurement
  for (int i{0}; i < 36; i++) {
    msg.MessageType[i] = cv[i];
  }
  msg.FrontLeftHeight = 0;
  msg.FrontRightHeight = 0;
  msg.RearLeftHeight = 0;
  msg.RearRightHeight = 0;
  msg.TimeReceived = ros_TimeStruct();
  //(&msg);
  return msg;
}

// End of code generation (atv_can_WheelDisplacementMeasurementStruct.cpp)
