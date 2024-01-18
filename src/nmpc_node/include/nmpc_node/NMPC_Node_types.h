//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// NMPC_Node_types.h
//
// Code generation for function 'NMPC_Node'
//

#ifndef NMPC_NODE_TYPES_H
#define NMPC_NODE_TYPES_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"

// Type Definitions
struct ros_TimeStruct_T {
  unsigned int Sec;
  unsigned int Nsec;
};

struct c_struct_T {
  double ref[3];
  double MVTarget[2];
  double Parameters[1];
  double X0[98];
  double MV0[14];
  double Slack0;
};

struct atv_can_DriveServiceRequestStruct_T {
  char MessageType[27];
  short MotorControl;
  unsigned short TurningRadius;
  unsigned char GearRatio;
  unsigned char AllWheelDrive;
  bool ControlMode;
  bool Direction;
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

struct geometry_msgs_AccelStruct_T {
  char MessageType[19];
  geometry_msgs_Vector3Struct_T Linear;
  geometry_msgs_Vector3Struct_T Angular;
};

struct atv_can_DriveServiceResponseStruct_T {
  char MessageType[28];
  coder::array<char, 2U> Status;
};

struct std_msgs_HeaderStruct_T {
  char MessageType[15];
  unsigned int Seq;
  ros_TimeStruct_T Stamp;
  coder::array<char, 2U> FrameId;
};

struct nav_msgs_OdometryStruct_T {
  char MessageType[17];
  std_msgs_HeaderStruct_T Header;
  coder::array<char, 2U> ChildFrameId;
  geometry_msgs_PoseWithCovarianceStruct_T Pose;
  geometry_msgs_TwistWithCovarianceStruct_T Twist;
};

#endif
// End of code generation (NMPC_Node_types.h)
