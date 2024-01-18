//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// atv_can_DriveServiceRequestStruct.cpp
//
// Code generation for function 'atv_can_DriveServiceRequestStruct'
//

// Include files
#include "atv_can_DriveServiceRequestStruct.h"
#include "NMPC_Node_types.h"
#include "rt_nonfinite.h"
#include <cstring>

// Function Definitions
atv_can_DriveServiceRequestStruct_T atv_can_DriveServiceRequestStruct()
{
  static const atv_can_DriveServiceRequestStruct_T b_msg{
      {'a', 't', 'v', '_', 'c', 'a', 'n', '/', 'D', 'r', 'i', 'v', 'e', 'S',
       'e', 'r', 'v', 'i', 'c', 'e', 'R', 'e', 'q', 'u', 'e', 's', 't'}, // MessageType
      0,     // MotorControl
      0U,    // TurningRadius
      0U,    // GearRatio
      0U,    // AllWheelDrive
      false, // ControlMode
      false  // Direction
  };
  atv_can_DriveServiceRequestStruct_T msg;
  msg = b_msg;
  //  Message struct definition for atv_can/DriveServiceRequest
  //(&b_msg);
  return msg;
}

// End of code generation (atv_can_DriveServiceRequestStruct.cpp)
