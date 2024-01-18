//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// sensor_msgs_NavSatStatusStruct.cpp
//
// Code generation for function 'sensor_msgs_NavSatStatusStruct'
//

// Include files
#include "sensor_msgs_NavSatStatusStruct.h"
#include "EKF_Node_types.h"
#include "rt_nonfinite.h"

// Function Definitions
sensor_msgs_NavSatStatusStruct_T sensor_msgs_NavSatStatusStruct()
{
  static const char cv[24]{'s', 'e', 'n', 's', 'o', 'r', '_', 'm',
                           's', 'g', 's', '/', 'N', 'a', 'v', 'S',
                           'a', 't', 'S', 't', 'a', 't', 'u', 's'};
  sensor_msgs_NavSatStatusStruct_T msg;
  //  Message struct definition for sensor_msgs/NavSatStatus
  for (int i{0}; i < 24; i++) {
    msg.MessageType[i] = cv[i];
  }
  msg.STATUSNOFIX = -1;
  msg.STATUSFIX = 0;
  msg.STATUSSBASFIX = 1;
  msg.STATUSGBASFIX = 2;
  msg.Status = 0;
  msg.SERVICEGPS = 1U;
  msg.SERVICEGLONASS = 2U;
  msg.SERVICECOMPASS = 4U;
  msg.SERVICEGALILEO = 8U;
  msg.Service = 0U;
  //(&msg);
  return msg;
}

// End of code generation (sensor_msgs_NavSatStatusStruct.cpp)
