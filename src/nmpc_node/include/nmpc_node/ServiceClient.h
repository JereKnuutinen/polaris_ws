//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// ServiceClient.h
//
// Code generation for function 'ServiceClient'
//

#ifndef SERVICECLIENT_H
#define SERVICECLIENT_H

// Include files
#include "NMPC_Node_types.h"
#include "rtwtypes.h"
#include "mlroscpp_svcclient.h"
#include <cstddef>
#include <cstdlib>

// Type Definitions
namespace coder {
namespace ros {
class ServiceClient {
public:
  ServiceClient *init();
  void isServerAvailable() const;
  void waitForServer() const;
  bool b_isServerAvailable() const;
  void call(const char varargin_1_MessageType[27],
            short varargin_1_MotorControl,
            unsigned short varargin_1_TurningRadius);
  char ServiceName[12];
  std::unique_ptr<MATLABSvcClient<
      atv_can::DriveService, atv_can::DriveServiceRequest,
      atv_can::DriveServiceResponse, atv_can_DriveServiceRequestStruct_T,
      atv_can_DriveServiceResponseStruct_T>>
      SvcClientHelperPtr;

private:
  atv_can_DriveServiceRequestStruct_T ReqMsgStruct;
  atv_can_DriveServiceResponseStruct_T RespMsgStruct;
};

} // namespace ros
} // namespace coder

#endif
// End of code generation (ServiceClient.h)
