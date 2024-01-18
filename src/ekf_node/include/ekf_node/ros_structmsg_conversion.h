#ifndef _ROS_STRUCTMSG_CONVERSION_H_
#define _ROS_STRUCTMSG_CONVERSION_H_

#include <ros/ros.h>
#include <atv_can/OdometryMeasurement.h>
#include <atv_can/SpeedandSteeringCommands.h>
#include <atv_can/SteeringMeasurement.h>
#include <atv_can/WheelDisplacementMeasurement.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <ros/time.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <std_msgs/Header.h>
#include "EKF_Node_types.h"
#include "mlroscpp_msgconvert_utils.h"


void struct2msg(atv_can::OdometryMeasurement* msgPtr, atv_can_OdometryMeasurementStruct_T const* structPtr);
void msg2struct(atv_can_OdometryMeasurementStruct_T* structPtr, atv_can::OdometryMeasurement const* msgPtr);

void struct2msg(atv_can::SpeedandSteeringCommands* msgPtr, atv_can_SpeedandSteeringCommandsStruct_T const* structPtr);
void msg2struct(atv_can_SpeedandSteeringCommandsStruct_T* structPtr, atv_can::SpeedandSteeringCommands const* msgPtr);

void struct2msg(atv_can::SteeringMeasurement* msgPtr, atv_can_SteeringMeasurementStruct_T const* structPtr);
void msg2struct(atv_can_SteeringMeasurementStruct_T* structPtr, atv_can::SteeringMeasurement const* msgPtr);

void struct2msg(atv_can::WheelDisplacementMeasurement* msgPtr, atv_can_WheelDisplacementMeasurementStruct_T const* structPtr);
void msg2struct(atv_can_WheelDisplacementMeasurementStruct_T* structPtr, atv_can::WheelDisplacementMeasurement const* msgPtr);

void struct2msg(geometry_msgs::Point* msgPtr, geometry_msgs_PointStruct_T const* structPtr);
void msg2struct(geometry_msgs_PointStruct_T* structPtr, geometry_msgs::Point const* msgPtr);

void struct2msg(geometry_msgs::Pose* msgPtr, geometry_msgs_PoseStruct_T const* structPtr);
void msg2struct(geometry_msgs_PoseStruct_T* structPtr, geometry_msgs::Pose const* msgPtr);

void struct2msg(geometry_msgs::PoseWithCovariance* msgPtr, geometry_msgs_PoseWithCovarianceStruct_T const* structPtr);
void msg2struct(geometry_msgs_PoseWithCovarianceStruct_T* structPtr, geometry_msgs::PoseWithCovariance const* msgPtr);

void struct2msg(geometry_msgs::Quaternion* msgPtr, geometry_msgs_QuaternionStruct_T const* structPtr);
void msg2struct(geometry_msgs_QuaternionStruct_T* structPtr, geometry_msgs::Quaternion const* msgPtr);

void struct2msg(geometry_msgs::Twist* msgPtr, geometry_msgs_TwistStruct_T const* structPtr);
void msg2struct(geometry_msgs_TwistStruct_T* structPtr, geometry_msgs::Twist const* msgPtr);

void struct2msg(geometry_msgs::TwistWithCovariance* msgPtr, geometry_msgs_TwistWithCovarianceStruct_T const* structPtr);
void msg2struct(geometry_msgs_TwistWithCovarianceStruct_T* structPtr, geometry_msgs::TwistWithCovariance const* msgPtr);

void struct2msg(geometry_msgs::Vector3* msgPtr, geometry_msgs_Vector3Struct_T const* structPtr);
void msg2struct(geometry_msgs_Vector3Struct_T* structPtr, geometry_msgs::Vector3 const* msgPtr);

void struct2msg(nav_msgs::Odometry* msgPtr, nav_msgs_OdometryStruct_T const* structPtr);
void msg2struct(nav_msgs_OdometryStruct_T* structPtr, nav_msgs::Odometry const* msgPtr);

void struct2msg(ros::Time* msgPtr, ros_TimeStruct_T const* structPtr);
void msg2struct(ros_TimeStruct_T* structPtr, ros::Time const* msgPtr);

void struct2msg(sensor_msgs::NavSatFix* msgPtr, sensor_msgs_NavSatFixStruct_T const* structPtr);
void msg2struct(sensor_msgs_NavSatFixStruct_T* structPtr, sensor_msgs::NavSatFix const* msgPtr);

void struct2msg(sensor_msgs::NavSatStatus* msgPtr, sensor_msgs_NavSatStatusStruct_T const* structPtr);
void msg2struct(sensor_msgs_NavSatStatusStruct_T* structPtr, sensor_msgs::NavSatStatus const* msgPtr);

void struct2msg(std_msgs::Header* msgPtr, std_msgs_HeaderStruct_T const* structPtr);
void msg2struct(std_msgs_HeaderStruct_T* structPtr, std_msgs::Header const* msgPtr);


#endif
