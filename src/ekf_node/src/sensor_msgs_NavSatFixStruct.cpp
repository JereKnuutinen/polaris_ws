//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// sensor_msgs_NavSatFixStruct.cpp
//
// Code generation for function 'sensor_msgs_NavSatFixStruct'
//

// Include files
#include "sensor_msgs_NavSatFixStruct.h"
#include "EKF_Node_types.h"
#include "rt_nonfinite.h"
#include "sensor_msgs_NavSatStatusStruct.h"
#include "std_msgs_HeaderStruct.h"
#include <cstring>

// Function Definitions
void sensor_msgs_NavSatFixStruct(sensor_msgs_NavSatFixStruct_T &msg)
{
  static const char cv[21]{'s', 'e', 'n', 's', 'o', 'r', '_',
                           'm', 's', 'g', 's', '/', 'N', 'a',
                           'v', 'S', 'a', 't', 'F', 'i', 'x'};
  //  Message struct definition for sensor_msgs/NavSatFix
  for (int i{0}; i < 21; i++) {
    msg.MessageType[i] = cv[i];
  }
  std_msgs_HeaderStruct(msg.Header);
  msg.Status = sensor_msgs_NavSatStatusStruct();
  msg.Latitude = 0.0;
  msg.Longitude = 0.0;
  msg.Altitude = 0.0;
  std::memset(&msg.PositionCovariance[0], 0, 9U * sizeof(double));
  msg.COVARIANCETYPEUNKNOWN = 0U;
  msg.COVARIANCETYPEAPPROXIMATED = 1U;
  msg.COVARIANCETYPEDIAGONALKNOWN = 2U;
  msg.COVARIANCETYPEKNOWN = 3U;
  msg.PositionCovarianceType = 0U;
  //(&msg);
}

// End of code generation (sensor_msgs_NavSatFixStruct.cpp)
