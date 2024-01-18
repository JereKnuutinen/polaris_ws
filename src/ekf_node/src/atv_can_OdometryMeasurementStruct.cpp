//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// atv_can_OdometryMeasurementStruct.cpp
//
// Code generation for function 'atv_can_OdometryMeasurementStruct'
//

// Include files
#include "atv_can_OdometryMeasurementStruct.h"
#include "EKF_Node_types.h"
#include "ros_TimeStruct.h"
#include "rt_nonfinite.h"

// Function Definitions
atv_can_OdometryMeasurementStruct_T atv_can_OdometryMeasurementStruct()
{
  static const char cv[27]{'a', 't', 'v', '_', 'c', 'a', 'n', '/', 'O',
                           'd', 'o', 'm', 'e', 't', 'r', 'y', 'M', 'e',
                           'a', 's', 'u', 'r', 'e', 'm', 'e', 'n', 't'};
  atv_can_OdometryMeasurementStruct_T msg;
  //  Message struct definition for atv_can/OdometryMeasurement
  for (int i{0}; i < 27; i++) {
    msg.MessageType[i] = cv[i];
  }
  msg.FrontLeft = 0;
  msg.FrontRight = 0;
  msg.RearLeft = 0;
  msg.RearRight = 0;
  msg.TimeReceived = ros_TimeStruct();
  //(&msg);
  return msg;
}

// End of code generation (atv_can_OdometryMeasurementStruct.cpp)
