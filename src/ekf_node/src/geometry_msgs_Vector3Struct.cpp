//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// geometry_msgs_Vector3Struct.cpp
//
// Code generation for function 'geometry_msgs_Vector3Struct'
//

// Include files
#include "geometry_msgs_Vector3Struct.h"
#include "EKF_Node_types.h"
#include "rt_nonfinite.h"

// Function Definitions
geometry_msgs_Vector3Struct_T geometry_msgs_Vector3Struct()
{
  static const char t0_MessageType[21]{'g', 'e', 'o', 'm', 'e', 't', 'r',
                                       'y', '_', 'm', 's', 'g', 's', '/',
                                       'V', 'e', 'c', 't', 'o', 'r', '3'};
  geometry_msgs_Vector3Struct_T msg;
  //  Message struct definition for geometry_msgs/Vector3
  for (int i{0}; i < 21; i++) {
    msg.MessageType[i] = t0_MessageType[i];
  }
  msg.X = 0.0;
  msg.Y = 0.0;
  msg.Z = 0.0;
  //(&msg);
  return msg;
}

// End of code generation (geometry_msgs_Vector3Struct.cpp)
