//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// Subscriber.cpp
//
// Code generation for function 'Subscriber'
//

// Include files
#include "Subscriber.h"
#include "NMPC_Node_data.h"
#include "NMPC_Node_types.h"
#include "nav_msgs_OdometryStruct.h"
#include "rt_nonfinite.h"
#include "mlroscpp_sub.h"
#include "ros/duration.h"
#include "ros/time.h"
#include <cmath>
#include <cstring>

// Function Definitions
namespace coder {
namespace ros {
void Subscriber::callback()
{
  MessageCount = get_MessageCount() + 1.0;
  if (IsInitialized) {
    double X_ekf_tmp;
    double b_X_ekf_tmp;
    double c_X_ekf_tmp;
    X_ekf[0] = MsgStruct.Pose.Pose.Position.X;
    X_ekf[1] = MsgStruct.Pose.Pose.Position.Y;
    X_ekf[2] = MsgStruct.Pose.Pose.Position.Z;
    X_ekf_tmp = MsgStruct.Twist.Twist.Linear.X;
    X_ekf[3] = X_ekf_tmp;
    b_X_ekf_tmp = MsgStruct.Twist.Twist.Linear.Y;
    X_ekf[4] = b_X_ekf_tmp;
    c_X_ekf_tmp = MsgStruct.Twist.Twist.Linear.Z;
    X_ekf[5] = c_X_ekf_tmp;
    X_ekf[6] = MsgStruct.Twist.Covariance[4];
    X_ekf[7] = MsgStruct.Twist.Covariance[5];
    X_ekf[8] = MsgStruct.Twist.Covariance[6];
    X_ekf[9] = MsgStruct.Twist.Twist.Angular.X;
    X_ekf[10] = MsgStruct.Twist.Twist.Angular.Y;
    X_ekf[11] = MsgStruct.Twist.Twist.Angular.Z;
    X_ekf[12] = MsgStruct.Twist.Covariance[7];
    X_ekf[13] = MsgStruct.Twist.Covariance[0];
    //  roll
    //  pitch
    //  heading
    //  p -- roll rate
    //  q -- pitch rate
    //  r -- yaw rate
    // curvaure
    //  put the TGC here
    V_ODOM = std::sqrt((X_ekf_tmp * X_ekf_tmp + b_X_ekf_tmp * b_X_ekf_tmp) +
                       c_X_ekf_tmp * c_X_ekf_tmp);
  }
}

void Subscriber::get_LatestMessage(
    geometry_msgs_TwistWithCovarianceStruct_T &lastSubMsg_Twist) const
{
  MATLABSUBSCRIBER_lock(SubscriberHelper);
  lastSubMsg_Twist = MsgStruct.Twist;
  MATLABSUBSCRIBER_unlock(SubscriberHelper);
}

double Subscriber::get_MessageCount() const
{
  return MessageCount;
}

Subscriber *Subscriber::init()
{
  static const char topic[8]{'/', 'e', 'k', 'f', '_', 'o', 'u', 't'};
  Subscriber *obj;
  obj = this;
  obj->IsInitialized = false;
  for (int i{0}; i < 8; i++) {
    obj->TopicName[i] = topic[i];
  }
  obj->BufferSize = 1.0;
  obj->MessageCount = 0.0;
  nav_msgs_OdometryStruct(obj->MsgStruct);
  auto structPtr = (&obj->MsgStruct);
  obj->SubscriberHelper = std::unique_ptr<
      MATLABSubscriber<nav_msgs::Odometry, nav_msgs_OdometryStruct_T>>(
      new MATLABSubscriber<nav_msgs::Odometry, nav_msgs_OdometryStruct_T>(
          structPtr, [this] { this->callback(); })); //();
  MATLABSUBSCRIBER_createSubscriber(obj->SubscriberHelper, &obj->TopicName[0],
                                    8.0, obj->BufferSize);
  obj->callback();
  obj->IsInitialized = true;
  return obj;
}

void Subscriber::receive(
    geometry_msgs_TwistWithCovarianceStruct_T &receivedMsg_Twist) const
{
  ::ros::Duration tDur;
  ::ros::Time tStop;
  double nMessages;
  bool exitg1;
  bool status;
  nMessages = get_MessageCount();
  tDur = tDur.fromSec(100.0);
  tStop = ::ros::Time::now() + (tDur);
  status = true;
  exitg1 = false;
  while ((!exitg1) && (get_MessageCount() == nMessages)) {
    ::ros::Time currentTime;
    currentTime = ::ros::Time::now();
    if (currentTime >= tStop) {
      status = false;
      exitg1 = true;
    }
  }
  char statusText[7];
  getStatusText(status, &statusText[0]);
  get_LatestMessage(receivedMsg_Twist);
}

} // namespace ros
} // namespace coder

// End of code generation (Subscriber.cpp)
