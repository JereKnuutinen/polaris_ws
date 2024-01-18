//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// ros_TimeStruct.cpp
//
// Code generation for function 'ros_TimeStruct'
//

// Include files
#include "ros_TimeStruct.h"
#include "EKF_Node_types.h"
#include "rt_nonfinite.h"

// Function Definitions
ros_TimeStruct_T ros_TimeStruct()
{
  ros_TimeStruct_T msg;
  //  Message struct definition for ros/Time
  msg.Sec = 0U;
  msg.Nsec = 0U;
  //(&msg);
  return msg;
}

// End of code generation (ros_TimeStruct.cpp)
