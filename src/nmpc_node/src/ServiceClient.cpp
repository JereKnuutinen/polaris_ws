//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// ServiceClient.cpp
//
// Code generation for function 'ServiceClient'
//

// Include files
#include "ServiceClient.h"
#include "NMPC_Node_types.h"
#include "atv_can_DriveServiceRequestStruct.h"
#include "atv_can_DriveServiceResponseStruct.h"
#include "rt_nonfinite.h"
#include "mlroscpp_svcclient.h"
#include <cstring>

// Function Definitions
namespace coder {
namespace ros {
void ServiceClient::isServerAvailable() const
{
  //(SvcClientHelperPtr);
  MATLABSvcClient_mlExists(SvcClientHelperPtr);
}

bool ServiceClient::b_isServerAvailable() const
{
  //(SvcClientHelperPtr);
  return MATLABSvcClient_mlExists(SvcClientHelperPtr);
}

void ServiceClient::call(const char varargin_1_MessageType[27],
                         short varargin_1_MotorControl,
                         unsigned short varargin_1_TurningRadius)
{
  atv_can_DriveServiceResponseStruct_T r;
  double x1;
  //(SvcClientHelperPtr);
  atv_can_DriveServiceResponseStruct(r);
  for (int i{0}; i < 27; i++) {
    ReqMsgStruct.MessageType[i] = varargin_1_MessageType[i];
  }
  ReqMsgStruct.MotorControl = varargin_1_MotorControl;
  ReqMsgStruct.TurningRadius = varargin_1_TurningRadius;
  ReqMsgStruct.GearRatio = 0U;
  ReqMsgStruct.AllWheelDrive = 0U;
  ReqMsgStruct.ControlMode = false;
  ReqMsgStruct.Direction = true;
  x1 = 2.0;
  MATLABSvcClient_callService(SvcClientHelperPtr, 1000.0, &x1);
  if (x1 == 0.0) {
    MATLABSvcClient_lock(SvcClientHelperPtr);
    MATLABSvcClient_unlock(SvcClientHelperPtr);
  }
}

ServiceClient *ServiceClient::init()
{
  static const char svcname[12]{'/', 'c', 'a', 'n', '_', 's',
                                'e', 'r', 'v', 'i', 'c', 'e'};
  ServiceClient *obj;
  bool isPersistent;
  obj = this;
  isPersistent = true;
  for (int i{0}; i < 12; i++) {
    obj->ServiceName[i] = svcname[i];
  }
  obj->ReqMsgStruct = atv_can_DriveServiceRequestStruct();
  atv_can_DriveServiceResponseStruct(obj->RespMsgStruct);
  atv_can::DriveService *svcPtr = nullptr;             //();
  atv_can::DriveServiceRequest *reqMsgPtr = nullptr;   //();
  atv_can::DriveServiceResponse *respMsgPtr = nullptr; //();
  auto reqStructPtr = (&obj->ReqMsgStruct);
  auto respStructPtr = (&obj->RespMsgStruct);
  obj->SvcClientHelperPtr = std::unique_ptr<MATLABSvcClient<
      atv_can::DriveService, atv_can::DriveServiceRequest,
      atv_can::DriveServiceResponse, atv_can_DriveServiceRequestStruct_T,
      atv_can_DriveServiceResponseStruct_T>>(
      new MATLABSvcClient<atv_can::DriveService, atv_can::DriveServiceRequest,
                          atv_can::DriveServiceResponse,
                          atv_can_DriveServiceRequestStruct_T,
                          atv_can_DriveServiceResponseStruct_T>(
          reqStructPtr, respStructPtr)); //();
  MATLABSvcClient_createSvcClient(obj->SvcClientHelperPtr, &obj->ServiceName[0],
                                  12.0, &isPersistent);
  obj->isServerAvailable();
  return obj;
}

void ServiceClient::waitForServer() const
{
  bool status;
  //(SvcClientHelperPtr);
  status = false;
  MATLABSvcClient_mlWaitForExistence(SvcClientHelperPtr, 10.0, &status);
}

} // namespace ros
} // namespace coder

// End of code generation (ServiceClient.cpp)
