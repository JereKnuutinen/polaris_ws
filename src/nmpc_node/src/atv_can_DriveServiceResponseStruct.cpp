//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// atv_can_DriveServiceResponseStruct.cpp
//
// Code generation for function 'atv_can_DriveServiceResponseStruct'
//

// Include files
#include "atv_can_DriveServiceResponseStruct.h"
#include "NMPC_Node_types.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <cstring>

// Function Definitions
void atv_can_DriveServiceResponseStruct(
    atv_can_DriveServiceResponseStruct_T &msg)
{
  static const char cv[28]{'a', 't', 'v', '_', 'c', 'a', 'n', '/', 'D', 'r',
                           'i', 'v', 'e', 'S', 'e', 'r', 'v', 'i', 'c', 'e',
                           'R', 'e', 's', 'p', 'o', 'n', 's', 'e'};
  //  Message struct definition for atv_can/DriveServiceResponse
  for (int i{0}; i < 28; i++) {
    msg.MessageType[i] = cv[i];
  }
  msg.Status.set_size(1, 0);
  //(&msg);
}

// End of code generation (atv_can_DriveServiceResponseStruct.cpp)
