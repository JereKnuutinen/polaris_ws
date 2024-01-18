#ifndef _ROS_STRUCTMSG_CONVERSION_H_
#define _ROS_STRUCTMSG_CONVERSION_H_

#include <ros/ros.h>
#include <atv_can/DriveService.h>
#include <atv_can/DriveServiceRequest.h>
#include <atv_can/DriveServiceResponse.h>
#include <geometry_msgs/Accel.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <ros/time.h>
#include <std_msgs/Header.h>
#include "NMPC_Node_types.h"
#include "mlroscpp_msgconvert_utils.h"


void struct2msg(atv_can::DriveServiceRequest* msgPtr, atv_can_DriveServiceRequestStruct_T const* structPtr);
void msg2struct(atv_can_DriveServiceRequestStruct_T* structPtr, atv_can::DriveServiceRequest const* msgPtr);

void struct2msg(atv_can::DriveServiceResponse* msgPtr, atv_can_DriveServiceResponseStruct_T const* structPtr);
void msg2struct(atv_can_DriveServiceResponseStruct_T* structPtr, atv_can::DriveServiceResponse const* msgPtr);

void struct2msg(geometry_msgs::Accel* msgPtr, geometry_msgs_AccelStruct_T const* structPtr);
void msg2struct(geometry_msgs_AccelStruct_T* structPtr, geometry_msgs::Accel const* msgPtr);

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

void struct2msg(std_msgs::Header* msgPtr, std_msgs_HeaderStruct_T const* structPtr);
void msg2struct(std_msgs_HeaderStruct_T* structPtr, std_msgs::Header const* msgPtr);


#endif
