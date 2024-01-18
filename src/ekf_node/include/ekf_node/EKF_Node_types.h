//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// EKF_Node_types.h
//
// Code generation for function 'EKF_Node'
//

#ifndef EKF_NODE_TYPES_H
#define EKF_NODE_TYPES_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"

// Type Definitions
struct ros_TimeStruct_T {
  unsigned int Sec;
  unsigned int Nsec;
};

struct atv_can_SpeedandSteeringCommandsStruct_T {
  char MessageType[32];
  short MotionControlRef;
  unsigned short TurningRadiusRef;
  unsigned char GearRatio;
  unsigned char AllWheelDrive;
  unsigned char DirectTorqueFlag;
  unsigned char FwdReverse;
  ros_TimeStruct_T TimeReceived;
};

struct atv_can_OdometryMeasurementStruct_T {
  char MessageType[27];
  short FrontLeft;
  short FrontRight;
  short RearLeft;
  short RearRight;
  ros_TimeStruct_T TimeReceived;
};

struct atv_can_SteeringMeasurementStruct_T {
  char MessageType[27];
  unsigned int EncoderPosition;
  ros_TimeStruct_T TimeReceived;
};

struct atv_can_WheelDisplacementMeasurementStruct_T {
  char MessageType[36];
  short FrontLeftHeight;
  short FrontRightHeight;
  short RearLeftHeight;
  short RearRightHeight;
  ros_TimeStruct_T TimeReceived;
};

struct sensor_msgs_NavSatStatusStruct_T {
  char MessageType[24];
  signed char STATUSNOFIX;
  signed char STATUSFIX;
  signed char STATUSSBASFIX;
  signed char STATUSGBASFIX;
  signed char Status;
  unsigned short SERVICEGPS;
  unsigned short SERVICEGLONASS;
  unsigned short SERVICECOMPASS;
  unsigned short SERVICEGALILEO;
  unsigned short Service;
};

struct geometry_msgs_PointStruct_T {
  char MessageType[19];
  double X;
  double Y;
  double Z;
};

struct geometry_msgs_QuaternionStruct_T {
  char MessageType[24];
  double X;
  double Y;
  double Z;
  double W;
};

struct geometry_msgs_PoseStruct_T {
  char MessageType[18];
  geometry_msgs_PointStruct_T Position;
  geometry_msgs_QuaternionStruct_T Orientation;
};

struct geometry_msgs_PoseWithCovarianceStruct_T {
  char MessageType[32];
  geometry_msgs_PoseStruct_T Pose;
  double Covariance[36];
};

struct geometry_msgs_Vector3Struct_T {
  char MessageType[21];
  double X;
  double Y;
  double Z;
};

struct geometry_msgs_TwistStruct_T {
  char MessageType[19];
  geometry_msgs_Vector3Struct_T Linear;
  geometry_msgs_Vector3Struct_T Angular;
};

struct geometry_msgs_TwistWithCovarianceStruct_T {
  char MessageType[33];
  geometry_msgs_TwistStruct_T Twist;
  double Covariance[36];
};

struct std_msgs_HeaderStruct_T {
  char MessageType[15];
  unsigned int Seq;
  ros_TimeStruct_T Stamp;
  coder::array<char, 2U> FrameId;
};

struct sensor_msgs_NavSatFixStruct_T {
  char MessageType[21];
  std_msgs_HeaderStruct_T Header;
  sensor_msgs_NavSatStatusStruct_T Status;
  double Latitude;
  double Longitude;
  double Altitude;
  double PositionCovariance[9];
  unsigned char COVARIANCETYPEUNKNOWN;
  unsigned char COVARIANCETYPEAPPROXIMATED;
  unsigned char COVARIANCETYPEDIAGONALKNOWN;
  unsigned char COVARIANCETYPEKNOWN;
  unsigned char PositionCovarianceType;
};

struct nav_msgs_OdometryStruct_T {
  char MessageType[17];
  std_msgs_HeaderStruct_T Header;
  coder::array<char, 2U> ChildFrameId;
  geometry_msgs_PoseWithCovarianceStruct_T Pose;
  geometry_msgs_TwistWithCovarianceStruct_T Twist;
};

#endif
// End of code generation (EKF_Node_types.h)
