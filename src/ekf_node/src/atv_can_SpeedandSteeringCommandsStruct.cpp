//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// atv_can_SpeedandSteeringCommandsStruct.cpp
//
// Code generation for function 'atv_can_SpeedandSteeringCommandsStruct'
//

// Include files
#include "atv_can_SpeedandSteeringCommandsStruct.h"
#include "EKF_Node_types.h"
#include "ros_TimeStruct.h"
#include "rt_nonfinite.h"

// Function Definitions
atv_can_SpeedandSteeringCommandsStruct_T
atv_can_SpeedandSteeringCommandsStruct()
{
  static const char cv[32]{'a', 't', 'v', '_', 'c', 'a', 'n', '/',
                           'S', 'p', 'e', 'e', 'd', 'a', 'n', 'd',
                           'S', 't', 'e', 'e', 'r', 'i', 'n', 'g',
                           'C', 'o', 'm', 'm', 'a', 'n', 'd', 's'};
  atv_can_SpeedandSteeringCommandsStruct_T msg;
  //  Message struct definition for atv_can/SpeedandSteeringCommands
  for (int i{0}; i < 32; i++) {
    msg.MessageType[i] = cv[i];
  }
  msg.MotionControlRef = 0;
  msg.TurningRadiusRef = 0U;
  msg.GearRatio = 0U;
  msg.AllWheelDrive = 0U;
  msg.DirectTorqueFlag = 0U;
  msg.FwdReverse = 0U;
  msg.TimeReceived = ros_TimeStruct();
  //(&msg);
  return msg;
}

// End of code generation (atv_can_SpeedandSteeringCommandsStruct.cpp)
