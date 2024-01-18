//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// geometry_msgs_QuaternionStruct.cpp
//
// Code generation for function 'geometry_msgs_QuaternionStruct'
//

// Include files
#include "geometry_msgs_QuaternionStruct.h"
#include "EKF_Node_types.h"
#include "rt_nonfinite.h"

// Function Definitions
geometry_msgs_QuaternionStruct_T geometry_msgs_QuaternionStruct()
{
  static const char t1_MessageType[24]{'g', 'e', 'o', 'm', 'e', 't', 'r', 'y',
                                       '_', 'm', 's', 'g', 's', '/', 'Q', 'u',
                                       'a', 't', 'e', 'r', 'n', 'i', 'o', 'n'};
  geometry_msgs_QuaternionStruct_T msg;
  //  Message struct definition for geometry_msgs/Quaternion
  for (int i{0}; i < 24; i++) {
    msg.MessageType[i] = t1_MessageType[i];
  }
  msg.X = 0.0;
  msg.Y = 0.0;
  msg.Z = 0.0;
  msg.W = 0.0;
  //(&msg);
  return msg;
}

// End of code generation (geometry_msgs_QuaternionStruct.cpp)
