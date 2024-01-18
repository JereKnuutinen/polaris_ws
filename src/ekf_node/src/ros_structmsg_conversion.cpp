#include "ros_structmsg_conversion.h"


// Conversions between atv_can_OdometryMeasurementStruct_T and atv_can::OdometryMeasurement

void struct2msg(atv_can::OdometryMeasurement* msgPtr, atv_can_OdometryMeasurementStruct_T const* structPtr)
{
  const std::string rosMessageType("atv_can/OdometryMeasurement");

  msgPtr->front_left =  structPtr->FrontLeft;
  msgPtr->front_right =  structPtr->FrontRight;
  msgPtr->rear_left =  structPtr->RearLeft;
  msgPtr->rear_right =  structPtr->RearRight;
  struct2msg(&msgPtr->time_received, &structPtr->TimeReceived);
}

void msg2struct(atv_can_OdometryMeasurementStruct_T* structPtr, atv_can::OdometryMeasurement const* msgPtr)
{
  const std::string rosMessageType("atv_can/OdometryMeasurement");

  structPtr->FrontLeft =  msgPtr->front_left;
  structPtr->FrontRight =  msgPtr->front_right;
  structPtr->RearLeft =  msgPtr->rear_left;
  structPtr->RearRight =  msgPtr->rear_right;
  msg2struct(&structPtr->TimeReceived, &msgPtr->time_received);
}


// Conversions between atv_can_SpeedandSteeringCommandsStruct_T and atv_can::SpeedandSteeringCommands

void struct2msg(atv_can::SpeedandSteeringCommands* msgPtr, atv_can_SpeedandSteeringCommandsStruct_T const* structPtr)
{
  const std::string rosMessageType("atv_can/SpeedandSteeringCommands");

  msgPtr->AllWheelDrive =  structPtr->AllWheelDrive;
  msgPtr->DirectTorqueFlag =  structPtr->DirectTorqueFlag;
  msgPtr->FwdReverse =  structPtr->FwdReverse;
  msgPtr->GearRatio =  structPtr->GearRatio;
  msgPtr->MotionControlRef =  structPtr->MotionControlRef;
  struct2msg(&msgPtr->time_received, &structPtr->TimeReceived);
  msgPtr->TurningRadiusRef =  structPtr->TurningRadiusRef;
}

void msg2struct(atv_can_SpeedandSteeringCommandsStruct_T* structPtr, atv_can::SpeedandSteeringCommands const* msgPtr)
{
  const std::string rosMessageType("atv_can/SpeedandSteeringCommands");

  structPtr->AllWheelDrive =  msgPtr->AllWheelDrive;
  structPtr->DirectTorqueFlag =  msgPtr->DirectTorqueFlag;
  structPtr->FwdReverse =  msgPtr->FwdReverse;
  structPtr->GearRatio =  msgPtr->GearRatio;
  structPtr->MotionControlRef =  msgPtr->MotionControlRef;
  msg2struct(&structPtr->TimeReceived, &msgPtr->time_received);
  structPtr->TurningRadiusRef =  msgPtr->TurningRadiusRef;
}


// Conversions between atv_can_SteeringMeasurementStruct_T and atv_can::SteeringMeasurement

void struct2msg(atv_can::SteeringMeasurement* msgPtr, atv_can_SteeringMeasurementStruct_T const* structPtr)
{
  const std::string rosMessageType("atv_can/SteeringMeasurement");

  msgPtr->encoder_position =  structPtr->EncoderPosition;
  struct2msg(&msgPtr->time_received, &structPtr->TimeReceived);
}

void msg2struct(atv_can_SteeringMeasurementStruct_T* structPtr, atv_can::SteeringMeasurement const* msgPtr)
{
  const std::string rosMessageType("atv_can/SteeringMeasurement");

  structPtr->EncoderPosition =  msgPtr->encoder_position;
  msg2struct(&structPtr->TimeReceived, &msgPtr->time_received);
}


// Conversions between atv_can_WheelDisplacementMeasurementStruct_T and atv_can::WheelDisplacementMeasurement

void struct2msg(atv_can::WheelDisplacementMeasurement* msgPtr, atv_can_WheelDisplacementMeasurementStruct_T const* structPtr)
{
  const std::string rosMessageType("atv_can/WheelDisplacementMeasurement");

  msgPtr->front_left_height =  structPtr->FrontLeftHeight;
  msgPtr->front_right_height =  structPtr->FrontRightHeight;
  msgPtr->rear_left_height =  structPtr->RearLeftHeight;
  msgPtr->rear_right_height =  structPtr->RearRightHeight;
  struct2msg(&msgPtr->time_received, &structPtr->TimeReceived);
}

void msg2struct(atv_can_WheelDisplacementMeasurementStruct_T* structPtr, atv_can::WheelDisplacementMeasurement const* msgPtr)
{
  const std::string rosMessageType("atv_can/WheelDisplacementMeasurement");

  structPtr->FrontLeftHeight =  msgPtr->front_left_height;
  structPtr->FrontRightHeight =  msgPtr->front_right_height;
  structPtr->RearLeftHeight =  msgPtr->rear_left_height;
  structPtr->RearRightHeight =  msgPtr->rear_right_height;
  msg2struct(&structPtr->TimeReceived, &msgPtr->time_received);
}


// Conversions between geometry_msgs_PointStruct_T and geometry_msgs::Point

void struct2msg(geometry_msgs::Point* msgPtr, geometry_msgs_PointStruct_T const* structPtr)
{
  const std::string rosMessageType("geometry_msgs/Point");

  msgPtr->x =  structPtr->X;
  msgPtr->y =  structPtr->Y;
  msgPtr->z =  structPtr->Z;
}

void msg2struct(geometry_msgs_PointStruct_T* structPtr, geometry_msgs::Point const* msgPtr)
{
  const std::string rosMessageType("geometry_msgs/Point");

  structPtr->X =  msgPtr->x;
  structPtr->Y =  msgPtr->y;
  structPtr->Z =  msgPtr->z;
}


// Conversions between geometry_msgs_PoseStruct_T and geometry_msgs::Pose

void struct2msg(geometry_msgs::Pose* msgPtr, geometry_msgs_PoseStruct_T const* structPtr)
{
  const std::string rosMessageType("geometry_msgs/Pose");

  struct2msg(&msgPtr->orientation, &structPtr->Orientation);
  struct2msg(&msgPtr->position, &structPtr->Position);
}

void msg2struct(geometry_msgs_PoseStruct_T* structPtr, geometry_msgs::Pose const* msgPtr)
{
  const std::string rosMessageType("geometry_msgs/Pose");

  msg2struct(&structPtr->Orientation, &msgPtr->orientation);
  msg2struct(&structPtr->Position, &msgPtr->position);
}


// Conversions between geometry_msgs_PoseWithCovarianceStruct_T and geometry_msgs::PoseWithCovariance

void struct2msg(geometry_msgs::PoseWithCovariance* msgPtr, geometry_msgs_PoseWithCovarianceStruct_T const* structPtr)
{
  const std::string rosMessageType("geometry_msgs/PoseWithCovariance");

  convertFromStructPrimitiveArray(msgPtr->covariance, structPtr->Covariance);
  struct2msg(&msgPtr->pose, &structPtr->Pose);
}

void msg2struct(geometry_msgs_PoseWithCovarianceStruct_T* structPtr, geometry_msgs::PoseWithCovariance const* msgPtr)
{
  const std::string rosMessageType("geometry_msgs/PoseWithCovariance");

  convertToStructPrimitiveArray(structPtr->Covariance, msgPtr->covariance);
  msg2struct(&structPtr->Pose, &msgPtr->pose);
}


// Conversions between geometry_msgs_QuaternionStruct_T and geometry_msgs::Quaternion

void struct2msg(geometry_msgs::Quaternion* msgPtr, geometry_msgs_QuaternionStruct_T const* structPtr)
{
  const std::string rosMessageType("geometry_msgs/Quaternion");

  msgPtr->w =  structPtr->W;
  msgPtr->x =  structPtr->X;
  msgPtr->y =  structPtr->Y;
  msgPtr->z =  structPtr->Z;
}

void msg2struct(geometry_msgs_QuaternionStruct_T* structPtr, geometry_msgs::Quaternion const* msgPtr)
{
  const std::string rosMessageType("geometry_msgs/Quaternion");

  structPtr->W =  msgPtr->w;
  structPtr->X =  msgPtr->x;
  structPtr->Y =  msgPtr->y;
  structPtr->Z =  msgPtr->z;
}


// Conversions between geometry_msgs_TwistStruct_T and geometry_msgs::Twist

void struct2msg(geometry_msgs::Twist* msgPtr, geometry_msgs_TwistStruct_T const* structPtr)
{
  const std::string rosMessageType("geometry_msgs/Twist");

  struct2msg(&msgPtr->angular, &structPtr->Angular);
  struct2msg(&msgPtr->linear, &structPtr->Linear);
}

void msg2struct(geometry_msgs_TwistStruct_T* structPtr, geometry_msgs::Twist const* msgPtr)
{
  const std::string rosMessageType("geometry_msgs/Twist");

  msg2struct(&structPtr->Angular, &msgPtr->angular);
  msg2struct(&structPtr->Linear, &msgPtr->linear);
}


// Conversions between geometry_msgs_TwistWithCovarianceStruct_T and geometry_msgs::TwistWithCovariance

void struct2msg(geometry_msgs::TwistWithCovariance* msgPtr, geometry_msgs_TwistWithCovarianceStruct_T const* structPtr)
{
  const std::string rosMessageType("geometry_msgs/TwistWithCovariance");

  convertFromStructPrimitiveArray(msgPtr->covariance, structPtr->Covariance);
  struct2msg(&msgPtr->twist, &structPtr->Twist);
}

void msg2struct(geometry_msgs_TwistWithCovarianceStruct_T* structPtr, geometry_msgs::TwistWithCovariance const* msgPtr)
{
  const std::string rosMessageType("geometry_msgs/TwistWithCovariance");

  convertToStructPrimitiveArray(structPtr->Covariance, msgPtr->covariance);
  msg2struct(&structPtr->Twist, &msgPtr->twist);
}


// Conversions between geometry_msgs_Vector3Struct_T and geometry_msgs::Vector3

void struct2msg(geometry_msgs::Vector3* msgPtr, geometry_msgs_Vector3Struct_T const* structPtr)
{
  const std::string rosMessageType("geometry_msgs/Vector3");

  msgPtr->x =  structPtr->X;
  msgPtr->y =  structPtr->Y;
  msgPtr->z =  structPtr->Z;
}

void msg2struct(geometry_msgs_Vector3Struct_T* structPtr, geometry_msgs::Vector3 const* msgPtr)
{
  const std::string rosMessageType("geometry_msgs/Vector3");

  structPtr->X =  msgPtr->x;
  structPtr->Y =  msgPtr->y;
  structPtr->Z =  msgPtr->z;
}


// Conversions between nav_msgs_OdometryStruct_T and nav_msgs::Odometry

void struct2msg(nav_msgs::Odometry* msgPtr, nav_msgs_OdometryStruct_T const* structPtr)
{
  const std::string rosMessageType("nav_msgs/Odometry");

  convertFromStructPrimitiveArray(msgPtr->child_frame_id, structPtr->ChildFrameId);
  struct2msg(&msgPtr->header, &structPtr->Header);
  struct2msg(&msgPtr->pose, &structPtr->Pose);
  struct2msg(&msgPtr->twist, &structPtr->Twist);
}

void msg2struct(nav_msgs_OdometryStruct_T* structPtr, nav_msgs::Odometry const* msgPtr)
{
  const std::string rosMessageType("nav_msgs/Odometry");

  convertToStructPrimitiveArray(structPtr->ChildFrameId, msgPtr->child_frame_id);
  msg2struct(&structPtr->Header, &msgPtr->header);
  msg2struct(&structPtr->Pose, &msgPtr->pose);
  msg2struct(&structPtr->Twist, &msgPtr->twist);
}


// Conversions between ros_TimeStruct_T and ros::Time

void struct2msg(ros::Time* msgPtr, ros_TimeStruct_T const* structPtr)
{
  const std::string rosMessageType("ros_time/Time");

  msgPtr->nsec =  structPtr->Nsec;
  msgPtr->sec =  structPtr->Sec;
}

void msg2struct(ros_TimeStruct_T* structPtr, ros::Time const* msgPtr)
{
  const std::string rosMessageType("ros_time/Time");

  structPtr->Nsec =  msgPtr->nsec;
  structPtr->Sec =  msgPtr->sec;
}


// Conversions between sensor_msgs_NavSatFixStruct_T and sensor_msgs::NavSatFix

void struct2msg(sensor_msgs::NavSatFix* msgPtr, sensor_msgs_NavSatFixStruct_T const* structPtr)
{
  const std::string rosMessageType("sensor_msgs/NavSatFix");

  msgPtr->altitude =  structPtr->Altitude;
  struct2msg(&msgPtr->header, &structPtr->Header);
  msgPtr->latitude =  structPtr->Latitude;
  msgPtr->longitude =  structPtr->Longitude;
  convertFromStructPrimitiveArray(msgPtr->position_covariance, structPtr->PositionCovariance);
  msgPtr->position_covariance_type =  structPtr->PositionCovarianceType;
  struct2msg(&msgPtr->status, &structPtr->Status);
}

void msg2struct(sensor_msgs_NavSatFixStruct_T* structPtr, sensor_msgs::NavSatFix const* msgPtr)
{
  const std::string rosMessageType("sensor_msgs/NavSatFix");

  structPtr->Altitude =  msgPtr->altitude;
  msg2struct(&structPtr->Header, &msgPtr->header);
  structPtr->Latitude =  msgPtr->latitude;
  structPtr->Longitude =  msgPtr->longitude;
  convertToStructPrimitiveArray(structPtr->PositionCovariance, msgPtr->position_covariance);
  structPtr->PositionCovarianceType =  msgPtr->position_covariance_type;
  msg2struct(&structPtr->Status, &msgPtr->status);
}


// Conversions between sensor_msgs_NavSatStatusStruct_T and sensor_msgs::NavSatStatus

void struct2msg(sensor_msgs::NavSatStatus* msgPtr, sensor_msgs_NavSatStatusStruct_T const* structPtr)
{
  const std::string rosMessageType("sensor_msgs/NavSatStatus");

  msgPtr->service =  structPtr->Service;
  msgPtr->status =  structPtr->Status;
}

void msg2struct(sensor_msgs_NavSatStatusStruct_T* structPtr, sensor_msgs::NavSatStatus const* msgPtr)
{
  const std::string rosMessageType("sensor_msgs/NavSatStatus");

  structPtr->Service =  msgPtr->service;
  structPtr->Status =  msgPtr->status;
}


// Conversions between std_msgs_HeaderStruct_T and std_msgs::Header

void struct2msg(std_msgs::Header* msgPtr, std_msgs_HeaderStruct_T const* structPtr)
{
  const std::string rosMessageType("std_msgs/Header");

  convertFromStructPrimitiveArray(msgPtr->frame_id, structPtr->FrameId);
  msgPtr->seq =  structPtr->Seq;
  struct2msg(&msgPtr->stamp, &structPtr->Stamp);
}

void msg2struct(std_msgs_HeaderStruct_T* structPtr, std_msgs::Header const* msgPtr)
{
  const std::string rosMessageType("std_msgs/Header");

  convertToStructPrimitiveArray(structPtr->FrameId, msgPtr->frame_id);
  structPtr->Seq =  msgPtr->seq;
  msg2struct(&structPtr->Stamp, &msgPtr->stamp);
}

