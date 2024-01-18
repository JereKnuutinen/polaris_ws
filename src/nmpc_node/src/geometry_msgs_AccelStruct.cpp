//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// geometry_msgs_AccelStruct.cpp
//
// Code generation for function 'geometry_msgs_AccelStruct'
//

// Include files
#include "geometry_msgs_AccelStruct.h"
#include "NMPC_Node_types.h"
#include "geometry_msgs_Vector3Struct.h"
#include "rt_nonfinite.h"
#include <cstring>

// Function Definitions
void geometry_msgs_AccelStruct(geometry_msgs_AccelStruct_T &msg)
{
  static const char cv[19]{'g', 'e', 'o', 'm', 'e', 't', 'r', 'y', '_', 'm',
                           's', 'g', 's', '/', 'A', 'c', 'c', 'e', 'l'};
  //  Message struct definition for geometry_msgs/Accel
  for (int i{0}; i < 19; i++) {
    msg.MessageType[i] = cv[i];
  }
  msg.Linear = geometry_msgs_Vector3Struct();
  msg.Angular = geometry_msgs_Vector3Struct();
  //(&msg);
}

// End of code generation (geometry_msgs_AccelStruct.cpp)
