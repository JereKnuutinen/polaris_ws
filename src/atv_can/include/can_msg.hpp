#ifndef ATV_CAN_CANMSG_HEADER
#define ATV_CAN_CANMSG_HEADER

#include "ros/ros.h"
#include <iostream>
#include <cstdint>
#include <linux/can.h>
#include <linux/can/raw.h>

namespace atv_can {
    struct CanMsg {
        long int id;
        uint64_t data;
        //uint8_t data;
        unsigned int data_length;
        unsigned int flag;
        unsigned long int can_timestamp;
        ros::Time ros_timestamp;

        static const unsigned int MESSAGE_ID_GUIDANCE_STATUS         = 0xac00; // WSHS
        static const unsigned int MESSAGE_ID_MACHINE_SELECTED_SPEEDS = 0xf022; // MSS
        static const unsigned int MESSAGE_ID_ODOMETRY =                0xfb00; // WODO
        static const unsigned int MESSAGE_ID_STEERING_COMMANDS =       0xfb01; // GASC
        static const unsigned int MESSAGE_ID_STEERING_POSITION =       0xfb02; // SODO
        static const unsigned int MESSAGE_ID_DASH_GAS_PEDAL_POSITION = 0xfb03; // GASP
        static const unsigned int MESSAGE_ID_WHEEL_DISPLACEMENT      = 0xfb05; // WSHS

        static const unsigned int ATV_CONTROL_MESSAGE_ID =          0xcfb0100;
        static const unsigned int BREAKING_COMMANDS_ID =            0xcfb0400;

        //struct can_frame frame;
    };

    union eight_bytes {
      uint64_t u64;
      uint8_t b8[sizeof(uint64_t)];
    };

    //from Chat GPT
    /*
    uint64_t convertToUint64(const uint8_t* byteArray){
    	uint64_t result=0;

    	for (int i =0; i < 8; i++){
    		result |= static_cast<uint64_t>(byteArray[i]) << (8*(7- i));
    	}
    	return result;
    } */

    //struct can_frame frame;
    std::ostream& operator<<(std::ostream& stream, const CanMsg& msg);

}

#endif
