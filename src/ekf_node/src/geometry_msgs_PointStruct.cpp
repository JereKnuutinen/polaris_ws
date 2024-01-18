//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// geometry_msgs_PointStruct.cpp
//
// Code generation for function 'geometry_msgs_PointStruct'
//

// Include files
#include "geometry_msgs_PointStruct.h"
#include "EKF_Node_types.h"
#include "rt_nonfinite.h"

// Function Definitions
geometry_msgs_PointStruct_T geometry_msgs_PointStruct()
{
  static const char t2_MessageType[19]{'g', 'e', 'o', 'm', 'e', 't', 'r',
                                       'y', '_', 'm', 's', 'g', 's', '/',
                                       'P', 'o', 'i', 'n', 't'};
  geometry_msgs_PointStruct_T msg;
  //  Message struct definition for geometry_msgs/Point
  for (int i{0}; i < 19; i++) {
    msg.MessageType[i] = t2_MessageType[i];
  }
  msg.X = 0.0;
  msg.Y = 0.0;
  msg.Z = 0.0;
  //(&msg);
  return msg;
}

// End of code generation (geometry_msgs_PointStruct.cpp)
