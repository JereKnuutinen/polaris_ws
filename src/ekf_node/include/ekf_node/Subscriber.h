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
#include "EKF_Node_types.h"
#include "rtwtypes.h"
#include "mlroscpp_sub.h"
#include <cstddef>
#include <cstdlib>

// Type Definitions
namespace coder {
namespace ros {
class Subscriber {
public:
  double get_MessageCount() const;
  Subscriber *init();
  void callback();
  void receive() const;
  void get_LatestMessage() const;
  char TopicName[17];
  double BufferSize;
  double MessageCount;

private:
  std::unique_ptr<
      MATLABSubscriber<nav_msgs::Odometry, nav_msgs_OdometryStruct_T>>
      SubscriberHelper;
  nav_msgs_OdometryStruct_T MsgStruct;
  bool IsInitialized;
};

class b_Subscriber {
public:
  b_Subscriber *init();
  void callback();
  double get_MessageCount() const;
  void receive() const;
  void get_LatestMessage() const;
  char TopicName[24];
  double BufferSize;
  double MessageCount;

private:
  std::unique_ptr<
      MATLABSubscriber<sensor_msgs::NavSatFix, sensor_msgs_NavSatFixStruct_T>>
      SubscriberHelper;
  sensor_msgs_NavSatFixStruct_T MsgStruct;
  bool IsInitialized;
};

class c_Subscriber {
public:
  c_Subscriber *init();
  void callback();
  double get_MessageCount() const;
  void receive() const;
  void get_LatestMessage() const;
  char TopicName[35];
  double BufferSize;
  double MessageCount;

private:
  std::unique_ptr<
      MATLABSubscriber<atv_can::WheelDisplacementMeasurement,
                       atv_can_WheelDisplacementMeasurementStruct_T>>
      SubscriberHelper;
  atv_can_WheelDisplacementMeasurementStruct_T MsgStruct;
  bool IsInitialized;
};

class d_Subscriber {
public:
  d_Subscriber *init();
  void callback();
  double get_MessageCount() const;
  void receive() const;
  void get_LatestMessage() const;
  char TopicName[25];
  double BufferSize;
  double MessageCount;

private:
  std::unique_ptr<MATLABSubscriber<atv_can::SteeringMeasurement,
                                   atv_can_SteeringMeasurementStruct_T>>
      SubscriberHelper;
  atv_can_SteeringMeasurementStruct_T MsgStruct;
  bool IsInitialized;
};

class e_Subscriber {
public:
  e_Subscriber *init();
  void callback();
  double get_MessageCount() const;
  void receive() const;
  void get_LatestMessage() const;
  char TopicName[25];
  double BufferSize;
  double MessageCount;

private:
  std::unique_ptr<MATLABSubscriber<atv_can::OdometryMeasurement,
                                   atv_can_OdometryMeasurementStruct_T>>
      SubscriberHelper;
  atv_can_OdometryMeasurementStruct_T MsgStruct;
  bool IsInitialized;
};

class f_Subscriber {
public:
  f_Subscriber *init();
  void callback();
  double get_MessageCount() const;
  char TopicName[32];
  double BufferSize;
  double MessageCount;

private:
  std::unique_ptr<MATLABSubscriber<atv_can::SpeedandSteeringCommands,
                                   atv_can_SpeedandSteeringCommandsStruct_T>>
      SubscriberHelper;
  atv_can_SpeedandSteeringCommandsStruct_T MsgStruct;
  bool IsInitialized;
};

} // namespace ros
} // namespace coder

#endif
// End of code generation (Subscriber.h)
