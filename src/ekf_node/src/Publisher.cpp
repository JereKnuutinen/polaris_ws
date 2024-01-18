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
#include "EKF_Node_types.h"
#include "nav_msgs_OdometryStruct.h"
#include "rt_nonfinite.h"
#include "mlroscpp_pub.h"

// Function Definitions
namespace coder {
namespace ros {
Publisher *Publisher::init()
{
  static const char topic[8]{'/', 'e', 'k', 'f', '_', 'o', 'u', 't'};
  Publisher *obj;
  nav_msgs_OdometryStruct_T r;
  obj = this;
  for (int i{0}; i < 8; i++) {
    obj->TopicName[i] = topic[i];
  }
  obj->BufferSize = 1.0;
  obj->IsLatching = true;
  nav_msgs_OdometryStruct(r);
  obj->PublisherHelper = std::unique_ptr<
      MATLABPublisher<nav_msgs::Odometry, nav_msgs_OdometryStruct_T>>(
      new MATLABPublisher<nav_msgs::Odometry,
                          nav_msgs_OdometryStruct_T>()); //();
  MATLABPUBLISHER_createPublisher(obj->PublisherHelper, &obj->TopicName[0], 8.0,
                                  obj->BufferSize, obj->IsLatching);
  return obj;
}

void Publisher::rosmessage(nav_msgs_OdometryStruct_T &msgFromPub)
{
  nav_msgs_OdometryStruct(msgFromPub);
}

} // namespace ros
} // namespace coder

// End of code generation (Publisher.cpp)
