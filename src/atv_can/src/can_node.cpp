#include "ros/ros.h"
#include "can_node.hpp"
#include "bitmanip.hpp"
#include <sstream>
#include <functional>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

using namespace atv_can;

CanNode::CanNode(int CAN_bitrate, int CAN_channel, unsigned int read_timeout_ms,
		unsigned int publications_per_sec) : /*can_bus_handler(CAN_bitrate, CAN_channel)
 ,*/
		read_timeout_ms(read_timeout_ms), publication_frequency_hz(
				publications_per_sec), keep_reading(false) {
	ROS_INFO_ONCE("Starting CAN Socket.");
	can_socket_handler = new SocketCanHandle();
}

CanNode::~CanNode() {
}

uint64_t CanNode::generate_can_message_data(const DriveService::Request &req) {
	uint64_t data = 0;
	/*
	 bits (0 starting from the right, bit 0 is LSB):
	 0-15:   motor control value
	 16-31:  turning radius
	 32-33:  gear ratio (low, mid, high)
	 34-35:  all wheel drive
	 36:     motor control mode (direct (0) or reference (1))
	 37:     forward (0) / reverse (1)
	 */
	ROS_INFO_ONCE("can node send message here");
	data |= req.direction;
	data <<= 1;
	data |= req.control_mode;
	data <<= 2;
	data |= req.all_wheel_drive;
	data <<= 2;
	data |= req.gear_ratio;
	data <<= 16;
	data |= req.turning_radius;
	data <<= 16;

	// unsigned integer, need to mask before or-operation (intermediate int64_t will be padded with 1s if negative)
	uint64_t mask_16 = 0xffff;
	data |= (req.motor_control & mask_16);

	return data;
}

bool CanNode::serve_send_request(DriveService::Request &req,
		DriveService::Response &resp) {
	CanMsg msg;
	struct can_frame frame;
	eight_bytes result;
	msg.data = generate_can_message_data(req);
	msg.data_length = 8;
	msg.id = CanMsg::ATV_CONTROL_MESSAGE_ID;
	bool status;
	//auto status = can_socket_handler->send_raw_message(msg);
	//resp.status = status.status_string;
	//std::cout << "response status is:\n " << resp.status << std::endl;

	result.u64 = generate_can_message_data(req);
	frame.can_dlc = 8;
	frame.can_id = CanMsg::ATV_CONTROL_MESSAGE_ID | CAN_EFF_FLAG; //CanMsg::ATV_CONTROL_MESSAGE_IDU | ;
	for (int i = 0; i < frame.can_dlc; i++) {
		frame.data[i] = result.b8[i];
	}

	//To see what is the can id
	//printf("can_frame id: 0x%03X, can dlc: [%d] \ncan data:", frame.can_id, frame.can_dlc);

	//for ( int i = 0; i < frame.can_dlc; i++) {
	//	printf("%022X ", frame.data[i]);
	//}

	status = can_socket_handler->SendFrame(&frame);
	if(status)
		ROS_INFO_ONCE("Message SENT.");

	publish_service_data(msg);
	return status;//(status.status_code >= 0);
}

void CanNode::read_forever() {
	keep_reading = true;
	while (keep_reading) {
		read_next_message();
	}
}

void CanNode::publish_service_data(const CanMsg &message) {
	commands_msg.MotionControlRef = get_bit_range(message.data, 0, 16);
	commands_msg.TurningRadiusRef = get_bit_range(message.data, 16, 16);
	commands_msg.GearRatio = get_bit_range(message.data, 32, 2);
	commands_msg.AllWheelDrive = get_bit_range(message.data, 34, 2);
	commands_msg.DirectTorqueFlag = get_bit_range(message.data, 36, 1);
	commands_msg.FwdReverse = get_bit_range(message.data, 37, 1);
	commands_msg.time_received = ros::Time::now();
	commands_publisher.publish(commands_msg);
	ROS_INFO_ONCE("GASC Message sent to CAN bus");
}

void CanNode::read_next_message() {
	CanMsg message;
	struct can_frame frame;
	bool status;
	//CanMsgStatusInfo status = can_socket_handler->read_raw_message(message, read_timeout_ms);
//    if (status.status_code != ::canOK) {
//        return;
//    }
	status = can_socket_handler->ReceiveFrame(&frame);
	if (!status) {
		return;
	}

	static double elapsed_time = 0.0;
	static double init_time = 0.0;
	static double NumPacket = 0.0;
	static int start_counter = 0;
	static double PacketRate = 0.0;

	//message = frame;

	//uint8_t result[8] = frame.data;
	eight_bytes result;
	for (int i = 0; i < frame.can_dlc; i++) {
		result.b8[i] = frame.data[i];
	}
	//result.b8[frame.can_dlc] = frame.data[frame.can_dlc];
	message.data = result.u64;

	//To see what is the can id
	//printf("can_node.cpp: RECEIVED can_frame id: 0x%03X, can dlc: [%d] \n", frame.can_id, frame.can_dlc);
	//printf("can_node.cpp: The correct message id is 0x%03X\n", message.ATV_CONTROL_MESSAGE_ID);


	//ROS_INFO("SocketCanHandle: received object - 0x%03X [%d] ", (frame.can_id >> 8) & 0x0ffff, frame.can_dlc);
	//ROS_INFO("Transformed Message: %ll", );
	//printf("b=%ju (0x%jx)\n", message.data, result.u64);

	switch ((frame.can_id >> 8) & 0x0ffff) // MultiTool PGN format, ignore priority, reserved, DP and SA
	{
	case 0: {
		//probably error?
		break;
	}
	case CanMsg::MESSAGE_ID_ODOMETRY: {
		odo_msg.front_left = get_bit_range(message.data, 0, 16);
		odo_msg.front_right = get_bit_range(message.data, 16, 16);
		odo_msg.rear_left = get_bit_range(message.data, 32, 16);
		odo_msg.rear_right = get_bit_range(message.data, 48, 16);
		odo_msg.time_received = ros::Time::now();
		odo_publisher.publish(odo_msg);
		break;
	}
	case CanMsg::MESSAGE_ID_STEERING_POSITION: {
		steer_msg.encoder_position = get_bit_range(message.data, 0, 32);
		steer_msg.time_received = ros::Time::now();
		steer_publisher.publish(steer_msg);
		break;
	}
	case CanMsg::MESSAGE_ID_DASH_GAS_PEDAL_POSITION: {
		gas_msg.pwm_ratio1 = get_bit_range(message.data, 0, 16);
		gas_msg.pwm_ratio2 = get_bit_range(message.data, 16, 16);
		gas_msg.gear_ratio = get_bit_range(message.data, 32, 2);
		gas_msg.all_wheel_drive = get_bit_range(message.data, 34, 2);
		gas_msg.time_received = ros::Time::now();
		gas_publisher.publish(gas_msg);
		break;
	}
	case CanMsg::MESSAGE_ID_WHEEL_DISPLACEMENT: {
		height_msg.front_left_height = get_bit_range(message.data, 0, 16);
		height_msg.front_right_height = get_bit_range(message.data, 16, 16);
		height_msg.rear_left_height = get_bit_range(message.data, 32, 16);
		height_msg.rear_right_height = get_bit_range(message.data, 48, 16);
		height_msg.time_received = ros::Time::now();
		displacement_publisher.publish(height_msg);
		/*ROS_INFO("FL = %d",  height_msg.front_left_height);
		 ROS_INFO("FR = %d",  height_msg.front_right_height);
		 ROS_INFO("RL = %d",  height_msg.rear_left_height);
		 ROS_INFO("RR = %d",  height_msg.rear_right_height);*/
		if (start_counter == 0) {
			init_time = ros::Time::now().toSec();
			NumPacket = 0.0;
			start_counter = 1;
		} else {
			elapsed_time = ros::Time::now().toSec() - init_time;
			NumPacket++;
			if (elapsed_time >= 2.0) {
				PacketRate = NumPacket / elapsed_time;
				start_counter = 0;
			}
		}

		break;
	}
		/*        case CanMsg::MESSAGE_ID_STEERING_COMMANDS:
		 {
		 commands_msg.MotionControlRef = get_bit_range(message.data, 0, 16);
		 commands_msg.TurningRadiusRef = get_bit_range(message.data, 16, 16);
		 commands_msg.GearRatio = get_bit_range(message.data, 32, 2);
		 commands_msg.AllWheelDrive = get_bit_range(message.data, 34, 2);
		 commands_msg.DirectTorqueFlag = get_bit_range(message.data, 36, 1);
		 commands_msg.FwdReverse = get_bit_range(message.data, 37, 1);
		 commands_msg.time_received = ros::Time::now();
		 commands_publisher.publish(commands_msg);
		 ROS_INFO("GASC Message read from CAN bus");
		 break;
		 }*/
	case CanMsg::MESSAGE_ID_MACHINE_SELECTED_SPEEDS: {
		mss_msg.SelectedSpeed = get_bit_range(message.data, 0, 16);
		mss_msg.SelectedDistance = get_bit_range(message.data, 16, 32);
		mss_msg.Reserved = get_bit_range(message.data, 48, 8);
		mss_msg.SpeedLimitStatus = get_bit_range(message.data, 56, 3);
		mss_msg.SelectedSpeedSource = get_bit_range(message.data, 59, 3);
		mss_msg.Direction = get_bit_range(message.data, 62, 2);
		mss_msg.time_received = ros::Time::now();
		mss_publisher.publish(mss_msg);
		//ROS_INFO("MSS Message read from CAN bus");
		break;
	}
	case CanMsg::MESSAGE_ID_GUIDANCE_STATUS: {
		guidance_msg.EstimatedCurvature = get_bit_range(message.data, 0, 16);
		guidance_msg.time_received = ros::Time::now();
		guidance_publisher.publish(guidance_msg);
		//ROS_INFO("MSS Message read from CAN bus");
		break;
	}
	default: {
		//unknown message, don't care
		break;
	}
	}
	/*ROS_INFO("WSHS Packet Rate: %f", PacketRate);*/
}

void CanNode::joy_callback(const sensor_msgs::Joy::ConstPtr &msg) {
	ROS_INFO("joy callback is here");
	ROS_INFO("please take notes which axis stands for which direction");
}
/*
 void CanNode::run_publisher(ros::NodeHandle& node_handle) {
 ros::Publisher odo_publisher = node_handle.advertise<atv_can::OdometryMeasurement>(  "atv_odometry_measurement", 0);
 ros::Publisher steer_publisher = node_handle.advertise<atv_can::SteeringMeasurement>("atv_steering_measurement", 0);
 ros::Publisher gas_publisher = node_handle.advertise<atv_can::DashGasPedalMeasurement>("atv_dash_gas_pedal_measurement",0);
 ros::Publisher displacement_publisher = node_handle.advertise<atv_can::WheelDisplacementMeasurement>("atv_wheel_displacement_measurement",0);

 ros::Rate loop_rate = publication_frequency_hz;
 while (ros::ok()) {
 odo_publisher.publish(odo_msg);
 steer_publisher.publish(steer_msg);
 gas_publisher.publish(gas_msg);
 displacement_publisher.publish(height_msg);
 loop_rate.sleep();
 }
 }
 */
void CanNode::run() {
	ros::NodeHandle node_handle;
	this->odo_publisher = node_handle.advertise<atv_can::OdometryMeasurement>("atv_odometry_measurement", 100);
	this->steer_publisher = node_handle.advertise<atv_can::SteeringMeasurement>("atv_steering_measurement", 100);
	this->gas_publisher = node_handle.advertise<atv_can::DashGasPedalMeasurement>("atv_dash_gas_pedal_measurement", 100);
	this->displacement_publisher = node_handle.advertise<atv_can::WheelDisplacementMeasurement>("atv_wheel_displacement_measurement", 100);
	this->commands_publisher = node_handle.advertise<atv_can::SpeedandSteeringCommands>("atv_speed_and_steering_commands", 100);
	this->mss_publisher = node_handle.advertise<atv_can::MachineSelectedSpeed>("atv_machine_selected_speed", 100);
	this->guidance_publisher = node_handle.advertise<atv_can::GuidanceMachineStatus>("atv_guidance_status", 100);

	//joy_subscriber = node_handle.subscribe<sensor_msgs::Joy>("joy", 0, &CanNode::joy_callback, this);

	std::thread listener_thread(&CanNode::read_forever, this);
	//std::thread publisher_thread(&CanNode::run_publisher, this, std::ref(node_handle));
	//ros::ServiceServer service = node_handle.advertiseService("can_service", &CanNode::serve_send_request, this);

	this->can_service = node_handle.advertiseService("can_service",&CanNode::serve_send_request, this);
	ROS_INFO("CAN service, requests can be sent");

	ROS_INFO("Start reading CAN bus");

	ros::spin();

	keep_reading = false;
	listener_thread.join();
	//publisher_thread.join();
}

int main(int argc, char **argv) {
	try {
		canInitializeLibrary();
		CanNode node(BAUD_250K, 0, 1000, 20);
		ros::init(argc, argv, "can_node");
		node.run();
	} catch (const std::exception &e) {
		std::cerr << e.what() << std::endl;
		return 1;
	}
	return 0;
}
