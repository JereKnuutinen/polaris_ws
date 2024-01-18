//=================================
// include guard
#ifndef __CAN_H_INCLUDED__
#define __CAN_H_INCLUDED__

//=================================
// define
enum EnumTypeMsg { STEER, SPEED_SET, SPEED_STOP, GEAR };
//=================================
// included dependencies
#include "../include/atvAcrosser/parameterCar.h"
#include "ackermann_msgs/AckermannDrive.h"
#include <boost/thread/thread.hpp>
#include "ros/ros.h"
#include <stdio.h>
#include "ARV6005Lib.h"


//=================================
// class
class CanBus{
	private:
		boost::mutex mtxCANbusBusy;
		boost::condition_variable condCANbusBusy;
		bool CANbusBusy;
		CanMsg msg2send[2];
		ackermann_msgs::AckermannDrive ackermannMsg;
		static void restartCommunication();
		static void canShowFwVersion();
		static void readDataFromCan(
		ackermann_msgs::AckermannDrive &ackermannMsg,
		CanBus &canBusCommunication);
		static void readDataFromRos(
		const ackermann_msgs::AckermannDrive::ConstPtr& msg,
		CanBus &canBusCommunication);
		//this function is used when I send just one message in
		//the CAN bus
		//static void readDataFromRos(
		//const ackermann_msgs::AckermannDrive::ConstPtr& msg);
	public:
		CanBus();
		~CanBus();
		void readCANmessageThread(int argc, char **argv,
		CanBus &canBusCommunication);
		void sendCANmessageThread(int argc, char **argv,
		CanBus &canBusCommunication);
		//
};
#endif // __CAN_H_INCLUDED__
