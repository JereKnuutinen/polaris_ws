#include "atvAcrosser/CAN.h"

int main(int argc, char **argv)
{

/*************************** THREAD *************************/
CanBus canBusCommunication;
//from CAN 2 ROS
boost::thread tReadCANmessageThread(&CanBus::readCANmessageThread,
&canBusCommunication, argc, argv, boost::ref(canBusCommunication));

//from ROS 2 CAN
boost::thread tSendCANmessageThread(&CanBus::sendCANmessageThread,
&canBusCommunication, argc, argv, boost::ref(canBusCommunication));
tReadCANmessageThread.join(); //from CAN 2 ROS
tSendCANmessageThread.join(); //from ROS 2 CAN
return 0;

}
