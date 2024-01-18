//=================================
// include guard
#ifndef __PARAMETERCARH_INCLUDED__
#define __PARAMETERCARH_INCLUDED__

//CAN communication parameters
#define AUTOMATION_STATUS 0x18FF2004ULL
#define SPEED_MESUREMENT 0xCF02205ULL
#define STEERING_POSITION 0xCAC0005ULL

//VEHICLE communication steering parameter
#define STEERING_DEFAULT_POSITION 32128
#define STEERING_MAX_RANGE 4000 //bits
#define STEERING_MAX_GRADE 70 //grads

//VEHICLE parameters
#define LENGHT_VEHICLE 1.85 //m
//resolution curvature: 0.25km^−1/bit

#define RESOLUTION_CURVATURE 4
#define CONVERSION_m2Km 1000
#define RESOLUTION_CURVATURE_CONVERSION
RESOLUTION_CURVATURE*CONVERSION_m2Km

//ROS frequencies parameters
#define ROS_SEND_MESSAGE_FREQUENCE_HZ 100 //Hz
#define ROS_READ_MESSAGE_FREQUENCE_HZ 100 //Hz

//maximum waiting time for a CAN communication
#define DELAY_MAX_SEND_CAN_MESSAGE 5000 //ms
#define DELAY_MAX_READ_CAN_MESSAGE 5000 //ms

#endif // __PARAMETERCARH_INCLUDED__
