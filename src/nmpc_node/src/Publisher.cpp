//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// Publisher.cpp
//
// Code generation for function 'Publisher'
//

// Include files
#include "Publisher.h"
#include "NMPC_Node_types.h"
#include "geometry_msgs_AccelStruct.h"
#include "rt_nonfinite.h"
#include "mlroscpp_pub.h"
#include <cstring>

// Function Definitions
namespace coder {
namespace ros {
Publisher *Publisher::init()
{
  static const char topic[9]{'/', 'n', 'm', 'p', 'c', '_', 'o', 'u', 't'};
  Publisher *obj;
  geometry_msgs_AccelStruct_T unusedExpr;
  obj = this;
  for (int i{0}; i < 9; i++) {
    obj->TopicName[i] = topic[i];
  }
  obj->BufferSize = 1.0;
  obj->IsLatching = true;
  geometry_msgs_AccelStruct(unusedExpr);
  obj->PublisherHelper = std::unique_ptr<
      MATLABPublisher<geometry_msgs::Accel, geometry_msgs_AccelStruct_T>>(
      new MATLABPublisher<geometry_msgs::Accel,
                          geometry_msgs_AccelStruct_T>()); //();
  MATLABPUBLISHER_createPublisher(obj->PublisherHelper, &obj->TopicName[0], 9.0,
                                  obj->BufferSize, obj->IsLatching);
  return obj;
}

b_Publisher *b_Publisher::init()
{
  static const char topic[14]{'/', 'r', 'e', 'f', '_', 'p', 'o',
                              'i', 'n', 't', '_', 'o', 'u', 't'};
  b_Publisher *obj;
  geometry_msgs_AccelStruct_T unusedExpr;
  obj = this;
  for (int i{0}; i < 14; i++) {
    obj->TopicName[i] = topic[i];
  }
  obj->BufferSize = 1.0;
  obj->IsLatching = true;
  geometry_msgs_AccelStruct(unusedExpr);
  obj->PublisherHelper = std::unique_ptr<
      MATLABPublisher<geometry_msgs::Accel, geometry_msgs_AccelStruct_T>>(
      new MATLABPublisher<geometry_msgs::Accel,
                          geometry_msgs_AccelStruct_T>()); //();
  MATLABPUBLISHER_createPublisher(obj->PublisherHelper, &obj->TopicName[0],
                                  14.0, obj->BufferSize, obj->IsLatching);
  return obj;
}

void Publisher::rosmessage(geometry_msgs_AccelStruct_T &msgFromPub)
{
  geometry_msgs_AccelStruct(msgFromPub);
}

} // namespace ros
} // namespace coder

// End of code generation (Publisher.cpp)
