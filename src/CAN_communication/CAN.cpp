#include "../include/atvAcrosser/CAN.h"

#include <atv_acrosser/killApp.h>
using namespace std;

/* Class initialization */
CanBus::CanBus()
{
CANbusBusy = false;

/*
initialization of the parameters used and interpreted by EPEC after
sending them by CAN bus*/
/* STEER*/
msg2send[STEER].id =0x18AD0500;
msg2send[STEER].id_type = EXT_ID;
msg2send[STEER].length = 3;
msg2send[STEER].data[0] = 0x00;
msg2send[STEER].data[1] = 0x00;
msg2send[STEER].data[2] = 0x01;
/* SPEED_SET */
msg2send[SPEED_SET].id =0x18FD4300;
msg2send[SPEED_SET].id_type = EXT_ID;
msg2send[SPEED_SET].length = 8;
msg2send[SPEED_SET].data[0] = 0x00;
msg2send[SPEED_SET].data[1] = 0x00;
msg2send[SPEED_SET].data[2] = 0xD0;
msg2send[SPEED_SET].data[3] = 0x00;
msg2send[SPEED_SET].data[4] = 0x00;
msg2send[SPEED_SET].data[5] = 0x00;
msg2send[SPEED_SET].data[6] = 0x00;
msg2send[SPEED_SET].data[7] = 0x00;
/* SPEED_STOP */
msg2send[SPEED_STOP].id =0x18FD4300;
msg2send[SPEED_STOP].id_type = EXT_ID;
msg2send[SPEED_STOP].length = 8;
msg2send[SPEED_STOP].data[0] = 0x00;
msg2send[SPEED_STOP].data[1] = 0x00;
msg2send[SPEED_STOP].data[2] = 0xD0;
msg2send[SPEED_STOP].data[3] = 0x00;
msg2send[SPEED_STOP].data[4] = 0x00;
msg2send[SPEED_STOP].data[5] = 0x00;
msg2send[SPEED_STOP].data[6] = 0x00;
msg2send[SPEED_STOP].data[7] = 0x00;
/* GEAR */
msg2send[GEAR].id =0x18FA5700;
msg2send[GEAR].id_type = EXT_ID;
msg2send[GEAR].length = 1;
msg2send[GEAR].data[0] = 0x06;
}

/* Class destroyer */
CanBus::~CanBus(){}
/******************************************************************
* @function: readDataFromCan
* Read data from CAN bus.
* Interprets the type of the message and updates the variable
* ackermannMsg with the value updated.
******************************************************************/
void CanBus::readDataFromCan(ackermann_msgs::AckermannDrive
&ackermannMsg, CanBus &canBusCommunication)
{
//temporary variable for put the value read from CAN bus
struct CanMsg canPkgReceive;
memset((void *)&canPkgReceive, 0, sizeof(canPkgReceive));
while(1)
	{
	boost::mutex::scoped_lock lock (canBusCommunication.mtxCANbusBusy);
	//the condition variable doesn’t allow to access to CAN bus till
	//the sending function has finished to send the message through the
	CAN bus
	while(canBusCommunication.CANbusBusy == true)
	canBusCommunication.condCANbusBusy.wait(lock);

	//call the funcion for get CAN message
	boost::thread function_caller(::getCanMessage, &canPkgReceive,1);
	//if the function getCanMessage doesn’t terminate before
	//DELAY_MAX_READ_CAN_MESSAGE ms, this program must be
	//restarted.
	//it’s necessary this control because happen that CAN bus stops
	// running and it’s necessary restart the program to resume CAN bus
	if (!function_caller.timed_join(boost::posix_time::milliseconds
	 (DELAY_MAX_READ_CAN_MESSAGE)))
	{
	restartCommunication();
	}
	//the message from CAN bus is interpreted and stored in the
	//variable ackermannMsg
	switch (canPkgReceive.id)
		{
		case SPEED_MESUREMENT:
			{
			float speed = 0; // mm/s −> millimeters, not meters
			//low byte: resolution: 0.001 m/s/bit
			speed = canPkgReceive.data[0] * 0.001;
			//upper byte: resolution: 0.256 m/s/bit
			speed += canPkgReceive.data[1] * 0.256;
			if(canPkgReceive.data[7] == 0x40)speed *= −1;
			ackermannMsg.speed = speed;
			break;
			}
		case STEERING_POSITION:
			{
			int position;
			115 position = canPkgReceive.data[0];
			position += canPkgReceive.data[1] * 256;
			//rad
			ackermannMsg.steering_angle = atan(LENGHT_VEHICLE*
			(position− STEERING_DEFAULT_POSITION) /
			(RESOLUTION_CURVATURE_CONVERSION ) );
			break;
			}
		}
	}//while(1)
}
/******************************************************************
130 * @function: readCANmessageThread
* Thread − send to ROS topic the ackermannMsg readed
* Periodically the function readDataFromCan is called to update
* the values of ackermannMsg from * CAN bus and published them
* via ROS.
*******************************************************************/
void CanBus::readCANmessageThread(int argc, char **argv, CanBus
&canBusCommunication)
{
//show the firmware version, it’s also used for checking if the CAN
//bus works
canShowFwVersion();
//ros initialization
ros::init(argc, argv, "");
ros::NodeHandle nodePlannerController;
//create a ros::Publisher which is used to publish on a topic called "fromCan2Ros"
ros::Publisher msgPublic = nodePlannerController.advertise
<ackermann_msgs::AckermannDrive>("fromCan2Ros", 1);
ros::Rate loop_rate(ROS_SEND_MESSAGE_FREQUENCE_HZ); //Hz

//call the thread readDataFromCan
boost::thread threadReadDataFromCan(&CanBus::readDataFromCan,
boost::ref(ackermannMsg),
boost::ref(canBusCommunication));

while (ros::ok())
	{
	//The publish() function is how you send messages.
	//The parameter is the message object.
	msgPublic.publish(ackermannMsg);
	ros::spinOnce();
	loop_rate.sleep();
	}
threadReadDataFromCan.join();
}
/******************************************************************
* @function: readDataFromRos
* The function is called periodically from the subscribe of the
* topic "fromRos2Can" and read the * ackermann_msgs received via
* ROS.
* The messaged received include steering and speed values are
* adapted for sending via CAN bus.
*******************************************************************/
void CanBus::readDataFromRos(const ackermann_msgs::AckermannDrive::
ConstPtr& msg, CanBus &canBusCommunication)
{
	// the values of speed and steer are read and adapted to be
	//correctly interpreted by the EPAC
	int steer = ( tan (msg−>steering_angle) *
	RESOLUTION_CURVATURE_CONVERSION / LENGHT_VEHICLE ) +
	STEERING_DEFAULT_POSITION;
	struct CanMsg msgSteeringdirection;
	canBusCommunication.msg2send[STEER].data[0] = (unsigned char)
	((steer) & 0xFF); //select lower byte
	canBusCommunication.msg2send[STEER].data[1] = (unsigned char)
	(((steer) & 0xFF00) >> 8); //select higher byte

	int absSpeed = (int) fabs(msg−>speed * 1000);
	canBusCommunication.msg2send[SPEED_SET].data[0] =
	(unsigned char)(absSpeed & 0xFF); //select lower byte
	canBusCommunication.msg2send[SPEED_SET].data[1] =
	(unsigned char)((absSpeed & 0xFF00) >> 8); //select higher byte
	canBusCommunication.msg2send[SPEED_SET].data[7] =
	(msg−>speed >= 0 )?0x40:0x00;
	boost::mutex::scoped_lock lock (canBusCommunication.
	mtxCANbusBusy);
	canBusCommunication.CANbusBusy = true;

	boost::thread function_caller(::sendCanMessage,
	canBusCommunication.msg2send,2);
	//if the function sendCanMessage doesn’t terminate before
	//DELAY_MAX_SEND_CAN_MESSAGE ms, this program must be
	//restarted
	if (!function_caller.timed_join(boost::posix_time::
	milliseconds(DELAY_MAX_SEND_CAN_MESSAGE)))
	{
		restartCommunication();
	}
	//set to false the boolean variable that rappresent the status
	//of the CAN bus and alert the condition variable that
	//the CAN bus is available for a possible read
	canBusCommunication.CANbusBusy = false;
	canBusCommunication.condCANbusBusy.notify_one();
}
/******************************************************************
220 * @function: sendCANmessageThread
* Thread − read from ROS topic "fromRos2Can"
* Periodically the function readDataFromRos is called read data
* from ROS and send them to CAN.
*******************************************************************/
void CanBus::sendCANmessageThread(int argc, char **argv, CanBus
&canBusCommunication)
{
//ros initialization
ros::init(argc, argv, "sendCANmessageThread");
ros::NodeHandle n;
ros::Rate loop_rate(ROS_READ_MESSAGE_FREQUENCE_HZ); //Hz
//The subscribe() call the ROS master node for communicate that
//the function wants to receive ROS messages from the topic
//"fromRos2Can"
ros::Subscriber sub = n.subscribe<ackermann_msgs::AckermannDrive>
("fromRos2Can", 1, boost::bind(readDataFromRos, _1,
boost::ref(canBusCommunication)));
while (ros::ok())
	{
	ros::spinOnce();
	loop_rate.sleep();
}
exit(1);
}
/******************************************************************
* @function: restartCommunication
250 * Kill the program
* There is a syncronous communication with the program
* startCommunication via ROS with a service.
* ./restartCommunication signal to startCommunication that the
* process must die, when get the feedback from
255 * ./restartCommunication it proceed to kill itself.
******************************************************************/
void CanBus::restartCommunication()
{
int argc = 0;
char** argv;
//ros initialization
ros::init(argc, argv, "talker");
ros::NodeHandle n;

//creates a client for the killApp service
ros::ServiceClient client = n.serviceClient<atv_acrosser::
killApp>("killCAN");
atv_acrosser::killApp srv;
//get the name of the pid
srv.request.pid2Kill = getpid();
if (client.call(srv))
{
	//when receive the feedback from the
	ROS_INFO("Sum: %ld", (long int)srv.response.pidKilled);
	exit(0);
}
else
	ROS_ERROR("Failed to call service killCAN");
 }
/******************************************************************
* @function: canShowFwVersion
* Show the version of the firmware version
285 * If there are some problem for getting the firmware version
* there are some problems in * the CAN communication and then
* then communication must be restarte calling the function
* restartCommunication().
******************************************************************/
void CanBus::canShowFwVersion()
{
	PicInfo picInfo = {0};
		int result = getCanFwVer(&picInfo);

		if(!result) {
				cout << picInfo.info;
		} else {
			restartCommunication();
		}
		return;
}
