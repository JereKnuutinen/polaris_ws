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
#include "EKF_Node_data.h"
#include "EKF_Node_rtwutil.h"
#include "EKF_Node_types.h"
#include "atv_can_OdometryMeasurementStruct.h"
#include "atv_can_SpeedandSteeringCommandsStruct.h"
#include "atv_can_SteeringMeasurementStruct.h"
#include "atv_can_WheelDisplacementMeasurementStruct.h"
#include "nav_msgs_OdometryStruct.h"
#include "quat2eul.h"
#include "rt_nonfinite.h"
#include "sensor_msgs_NavSatFixStruct.h"
#include "tand.h"
#include "coder_array.h"
#include "mlroscpp_sub.h"
#include "ros/duration.h"
#include "ros/time.h"
#include <cmath>

// Function Definitions
namespace coder {
namespace ros {
void Subscriber::callback()
{
  array<double, 1U> r;
  array<double, 1U> x;
  array<int, 2U> r1;
  MessageCount = get_MessageCount() + 1.0;
  if (IsInitialized) {
    double RPY[3];
    double RPY_tmp;
    double aSinInput;
    double b_RPY_tmp;
    double c_RPY_tmp;
    double d_RPY_tmp;
    double q_idx_0;
    double q_idx_1;
    double q_idx_2;
    double q_idx_3;
    double unnamed_idx_0;
    int b_trueCount;
    int k;
    int trueCount;
    bool b;
    q_idx_0 = MsgStruct.Pose.Pose.Orientation.W;
    q_idx_1 = MsgStruct.Pose.Pose.Orientation.X;
    q_idx_2 = MsgStruct.Pose.Pose.Orientation.Y;
    q_idx_3 = MsgStruct.Pose.Pose.Orientation.Z;
    aSinInput = 1.0 / std::sqrt(((q_idx_0 * q_idx_0 + q_idx_1 * q_idx_1) +
                                 q_idx_2 * q_idx_2) +
                                q_idx_3 * q_idx_3);
    q_idx_0 *= aSinInput;
    q_idx_1 *= aSinInput;
    q_idx_2 *= aSinInput;
    q_idx_3 *= aSinInput;
    aSinInput = -2.0 * (q_idx_1 * q_idx_3 - q_idx_0 * q_idx_2);
    unnamed_idx_0 = aSinInput;
    if (aSinInput >= 0.99999999999999778) {
      unnamed_idx_0 = 1.0;
    }
    if (aSinInput <= -0.99999999999999778) {
      unnamed_idx_0 = -1.0;
    }
    RPY_tmp = q_idx_0 * q_idx_0;
    b_RPY_tmp = q_idx_1 * q_idx_1;
    c_RPY_tmp = q_idx_2 * q_idx_2;
    d_RPY_tmp = q_idx_3 * q_idx_3;
    RPY[0] = rt_atan2d_snf(2.0 * (q_idx_1 * q_idx_2 + q_idx_0 * q_idx_3),
                           ((RPY_tmp + b_RPY_tmp) - c_RPY_tmp) - d_RPY_tmp);
    RPY[1] = std::asin(unnamed_idx_0);
    RPY[2] = rt_atan2d_snf(2.0 * (q_idx_2 * q_idx_3 + q_idx_0 * q_idx_1),
                           ((RPY_tmp - b_RPY_tmp) - c_RPY_tmp) + d_RPY_tmp);
    trueCount = 0;
    b = ((aSinInput >= 0.99999999999999778) ||
         (aSinInput <= -0.99999999999999778));
    if (b) {
      trueCount = 1;
    }
    x.set_size(trueCount);
    for (k = 0; k < trueCount; k++) {
      x[0] = unnamed_idx_0;
    }
    b_trueCount = 0;
    if (b) {
      b_trueCount = 1;
    }
    for (k = 0; k < b_trueCount; k++) {
      if (std::isnan(x[0])) {
        x[0] = rtNaN;
      } else if (x[0] < 0.0) {
        x[0] = -1.0;
      } else {
        x[0] = (x[0] > 0.0);
      }
    }
    b_trueCount = 0;
    if (b) {
      b_trueCount = 1;
    }
    r.set_size(b_trueCount);
    for (k = 0; k < b_trueCount; k++) {
      r[0] = rt_atan2d_snf(q_idx_1, q_idx_0);
    }
    k = 0;
    if (b) {
      k = 1;
    }
    r1.set_size(1, k);
    if (b) {
      r1[0] = 0;
    }
    if (trueCount == b_trueCount) {
      if (trueCount - 1 >= 0) {
        RPY[0] = -x[0] * 2.0 * r[0];
      }
    } else {
      binary_expand_op(RPY, r1, x, r);
    }
    trueCount = 0;
    if (b) {
      trueCount = 1;
    }
    if (trueCount - 1 >= 0) {
      RPY[2] = 0.0;
    }
    phi_m = RPY[2];
    theta_m = RPY[1];
    psi_m = RPY[0];
    dotE_m = MsgStruct.Twist.Twist.Linear.X;
    dotN_m = MsgStruct.Twist.Twist.Linear.Y;
    dotU_m = MsgStruct.Twist.Twist.Linear.Z;
  }
}

void b_Subscriber::callback()
{
  MessageCount = get_MessageCount() + 1.0;
  if (IsInitialized) {
    //  GPS position to ENU
    pos_gnss_lla[0] = MsgStruct.Latitude;
    pos_gnss_lla[1] = MsgStruct.Longitude;
    pos_gnss_lla[2] = MsgStruct.Altitude;
  }
}

void c_Subscriber::callback()
{
  MessageCount = get_MessageCount() + 1.0;
  if (IsInitialized) {
    //  extract voltages from message
    //  displacements in Dx = m*Voltage + c format
    delta_x_FR = (0.000195989492265 *
                      (static_cast<double>(MsgStruct.FrontRightHeight) + 52.0) -
                  0.2095587474703) -
                 0.83351660939284078;
    delta_x_FL =
        (-0.000192955002783 * static_cast<double>(MsgStruct.FrontLeftHeight) +
         0.855193282527722) -
        0.79819784228631;
    delta_x_RR =
        (-0.000199974706574 * static_cast<double>(MsgStruct.RearLeftHeight) +
         0.999466491187436) -
        0.90675999999987489;
    delta_x_RL =
        (0.000185357944175 * static_cast<double>(MsgStruct.RearRightHeight) -
         0.052225961005522) -
        0.916651946894125;
  }
}

void d_Subscriber::callback()
{
  MessageCount = get_MessageCount() + 1.0;
  if (IsInitialized) {
    double d;
    stg = -0.003784758774 * static_cast<double>(MsgStruct.EncoderPosition) +
          1806.480038254869;
    d = stg;
    b_tand(d);
    curv = d / 1.83;
  }
}

void e_Subscriber::callback()
{
  MessageCount = get_MessageCount() + 1.0;
  if (IsInitialized) {
    V_ODOM =
        static_cast<double>(MsgStruct.RearRight - MsgStruct.RearLeft) / 2000.0;
  }
}

void f_Subscriber::callback()
{
  MessageCount = get_MessageCount() + 1.0;
  if (IsInitialized) {
    stg_cmd =
        -0.000994869050617 * static_cast<double>(MsgStruct.TurningRadiusRef) +
        32.649869050616971;
    V_cmd = static_cast<double>(MsgStruct.MotionControlRef) / 1000.0;
  }
}

void e_Subscriber::get_LatestMessage() const
{
  MATLABSUBSCRIBER_lock(SubscriberHelper);
  MATLABSUBSCRIBER_unlock(SubscriberHelper);
}

void d_Subscriber::get_LatestMessage() const
{
  MATLABSUBSCRIBER_lock(SubscriberHelper);
  MATLABSUBSCRIBER_unlock(SubscriberHelper);
}

void c_Subscriber::get_LatestMessage() const
{
  MATLABSUBSCRIBER_lock(SubscriberHelper);
  MATLABSUBSCRIBER_unlock(SubscriberHelper);
}

void b_Subscriber::get_LatestMessage() const
{
  MATLABSUBSCRIBER_lock(SubscriberHelper);
  MATLABSUBSCRIBER_unlock(SubscriberHelper);
}

void Subscriber::get_LatestMessage() const
{
  MATLABSUBSCRIBER_lock(SubscriberHelper);
  MATLABSUBSCRIBER_unlock(SubscriberHelper);
}

double f_Subscriber::get_MessageCount() const
{
  return MessageCount;
}

double Subscriber::get_MessageCount() const
{
  return MessageCount;
}

double e_Subscriber::get_MessageCount() const
{
  return MessageCount;
}

double b_Subscriber::get_MessageCount() const
{
  return MessageCount;
}

double d_Subscriber::get_MessageCount() const
{
  return MessageCount;
}

double c_Subscriber::get_MessageCount() const
{
  return MessageCount;
}

Subscriber *Subscriber::init()
{
  static const char topic[17]{'/', 'n', 'o', 'v', 'a', 't', 'e', 'l', '/',
                              'o', 'd', 'o', 'm', '_', 'm', 'a', 'p'};
  Subscriber *obj;
  obj = this;
  obj->IsInitialized = false;
  for (int i{0}; i < 17; i++) {
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
                                    17.0, obj->BufferSize);
  obj->callback();
  obj->IsInitialized = true;
  return obj;
}

f_Subscriber *f_Subscriber::init()
{
  static const char topic[32]{'/', 'a', 't', 'v', '_', 's', 'p', 'e',
                              'e', 'd', '_', 'a', 'n', 'd', '_', 's',
                              't', 'e', 'e', 'r', 'i', 'n', 'g', '_',
                              'c', 'o', 'm', 'm', 'a', 'n', 'd', 's'};
  f_Subscriber *obj;
  obj = this;
  obj->IsInitialized = false;
  for (int i{0}; i < 32; i++) {
    obj->TopicName[i] = topic[i];
  }
  obj->BufferSize = 1.0;
  obj->MessageCount = 0.0;
  obj->MsgStruct = atv_can_SpeedandSteeringCommandsStruct();
  auto structPtr = (&obj->MsgStruct);
  obj->SubscriberHelper = std::unique_ptr<
      MATLABSubscriber<atv_can::SpeedandSteeringCommands,
                       atv_can_SpeedandSteeringCommandsStruct_T>>(
      new MATLABSubscriber<atv_can::SpeedandSteeringCommands,
                           atv_can_SpeedandSteeringCommandsStruct_T>(
          structPtr, [this] { this->callback(); })); //();
  MATLABSUBSCRIBER_createSubscriber(obj->SubscriberHelper, &obj->TopicName[0],
                                    32.0, obj->BufferSize);
  obj->callback();
  obj->IsInitialized = true;
  return obj;
}

b_Subscriber *b_Subscriber::init()
{
  static const char topic[24]{'/', 'n', 'o', 'v', 'a', 't', 'e', 'l',
                              '/', 'g', 'p', 's', '_', 'f', 'i', 'x',
                              '_', 'b', 'e', 's', 't', 'p', 'o', 's'};
  b_Subscriber *obj;
  obj = this;
  obj->IsInitialized = false;
  for (int i{0}; i < 24; i++) {
    obj->TopicName[i] = topic[i];
  }
  obj->BufferSize = 1.0;
  obj->MessageCount = 0.0;
  sensor_msgs_NavSatFixStruct(obj->MsgStruct);
  auto structPtr = (&obj->MsgStruct);
  obj->SubscriberHelper = std::unique_ptr<
      MATLABSubscriber<sensor_msgs::NavSatFix, sensor_msgs_NavSatFixStruct_T>>(
      new MATLABSubscriber<sensor_msgs::NavSatFix,
                           sensor_msgs_NavSatFixStruct_T>(
          structPtr, [this] { this->callback(); })); //();
  MATLABSUBSCRIBER_createSubscriber(obj->SubscriberHelper, &obj->TopicName[0],
                                    24.0, obj->BufferSize);
  obj->callback();
  obj->IsInitialized = true;
  return obj;
}

c_Subscriber *c_Subscriber::init()
{
  static const char topic[35]{'/', 'a', 't', 'v', '_', 'w', 'h', 'e', 'e',
                              'l', '_', 'd', 'i', 's', 'p', 'l', 'a', 'c',
                              'e', 'm', 'e', 'n', 't', '_', 'm', 'e', 'a',
                              's', 'u', 'r', 'e', 'm', 'e', 'n', 't'};
  c_Subscriber *obj;
  obj = this;
  obj->IsInitialized = false;
  for (int i{0}; i < 35; i++) {
    obj->TopicName[i] = topic[i];
  }
  obj->BufferSize = 1.0;
  obj->MessageCount = 0.0;
  obj->MsgStruct = atv_can_WheelDisplacementMeasurementStruct();
  auto structPtr = (&obj->MsgStruct);
  obj->SubscriberHelper = std::unique_ptr<
      MATLABSubscriber<atv_can::WheelDisplacementMeasurement,
                       atv_can_WheelDisplacementMeasurementStruct_T>>(
      new MATLABSubscriber<atv_can::WheelDisplacementMeasurement,
                           atv_can_WheelDisplacementMeasurementStruct_T>(
          structPtr, [this] { this->callback(); })); //();
  MATLABSUBSCRIBER_createSubscriber(obj->SubscriberHelper, &obj->TopicName[0],
                                    35.0, obj->BufferSize);
  obj->callback();
  obj->IsInitialized = true;
  return obj;
}

d_Subscriber *d_Subscriber::init()
{
  static const char topic[25]{'/', 'a', 't', 'v', '_', 's', 't', 'e', 'e',
                              'r', 'i', 'n', 'g', '_', 'm', 'e', 'a', 's',
                              'u', 'r', 'e', 'm', 'e', 'n', 't'};
  d_Subscriber *obj;
  obj = this;
  obj->IsInitialized = false;
  for (int i{0}; i < 25; i++) {
    obj->TopicName[i] = topic[i];
  }
  obj->BufferSize = 1.0;
  obj->MessageCount = 0.0;
  obj->MsgStruct = atv_can_SteeringMeasurementStruct();
  auto structPtr = (&obj->MsgStruct);
  obj->SubscriberHelper =
      std::unique_ptr<MATLABSubscriber<atv_can::SteeringMeasurement,
                                       atv_can_SteeringMeasurementStruct_T>>(
          new MATLABSubscriber<atv_can::SteeringMeasurement,
                               atv_can_SteeringMeasurementStruct_T>(
              structPtr, [this] { this->callback(); })); //();
  MATLABSUBSCRIBER_createSubscriber(obj->SubscriberHelper, &obj->TopicName[0],
                                    25.0, obj->BufferSize);
  obj->callback();
  obj->IsInitialized = true;
  return obj;
}

e_Subscriber *e_Subscriber::init()
{
  static const char topic[25]{'/', 'a', 't', 'v', '_', 'o', 'd', 'o', 'm',
                              'e', 't', 'r', 'y', '_', 'm', 'e', 'a', 's',
                              'u', 'r', 'e', 'm', 'e', 'n', 't'};
  e_Subscriber *obj;
  obj = this;
  obj->IsInitialized = false;
  for (int i{0}; i < 25; i++) {
    obj->TopicName[i] = topic[i];
  }
  obj->BufferSize = 1.0;
  obj->MessageCount = 0.0;
  obj->MsgStruct = atv_can_OdometryMeasurementStruct();
  auto structPtr = (&obj->MsgStruct);
  obj->SubscriberHelper =
      std::unique_ptr<MATLABSubscriber<atv_can::OdometryMeasurement,
                                       atv_can_OdometryMeasurementStruct_T>>(
          new MATLABSubscriber<atv_can::OdometryMeasurement,
                               atv_can_OdometryMeasurementStruct_T>(
              structPtr, [this] { this->callback(); })); //();
  MATLABSUBSCRIBER_createSubscriber(obj->SubscriberHelper, &obj->TopicName[0],
                                    25.0, obj->BufferSize);
  obj->callback();
  obj->IsInitialized = true;
  return obj;
}

void c_Subscriber::receive() const
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
  get_LatestMessage();
}

void d_Subscriber::receive() const
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
  get_LatestMessage();
}

void b_Subscriber::receive() const
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
  get_LatestMessage();
}

void e_Subscriber::receive() const
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
  get_LatestMessage();
}

void Subscriber::receive() const
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
  get_LatestMessage();
}

} // namespace ros
} // namespace coder

// End of code generation (Subscriber.cpp)
