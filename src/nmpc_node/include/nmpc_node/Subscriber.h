//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// Subscriber.h
//
// Code generation for function 'Subscriber'
//

#ifndef SUBSCRIBER_H
#define SUBSCRIBER_H

// Include files
#include "NMPC_Node_types.h"
#include "rtwtypes.h"
#include "mlroscpp_sub.h"
#include <cstddef>
#include <cstdlib>

// Type Definitions
namespace coder {
namespace ros {
class Subscriber {
public:
  Subscriber *init();
  void callback();
  double get_MessageCount() const;
  void
  receive(geometry_msgs_TwistWithCovarianceStruct_T &receivedMsg_Twist) const;
  void get_LatestMessage(
      geometry_msgs_TwistWithCovarianceStruct_T &lastSubMsg_Twist) const;
  char TopicName[8];
  double BufferSize;
  double MessageCount;

private:
  std::unique_ptr<
      MATLABSubscriber<nav_msgs::Odometry, nav_msgs_OdometryStruct_T>>
      SubscriberHelper;
  nav_msgs_OdometryStruct_T MsgStruct;
  bool IsInitialized;
};

} // namespace ros
} // namespace coder

#endif
// End of code generation (Subscriber.h)
