#include "teleop_joy.hpp"

bool stopPushed = false;
bool speedTest = false;
bool steeringTest = false;
int max_torque;
TeleopJoyNode::TeleopJoyNode(int16_t max_torque)
//: max_torque(max_torque)
:
		mtr_ctrl(0), trn_rad(STEERING_MIDDLE_POSITION) {
}
void TeleopJoyNode::read_joystick_input(const sensor_msgs::Joy::ConstPtr &joy) {
	//ROS_INFO("joystck ---------------");
	update_motor_control(joy, max_torque);
	update_turning_radius(joy);
	std::cout << trn_rad << "  " << mtr_ctrl << std::endl;
}
void TeleopJoyNode::read_joystick_input_modified(
		const sensor_msgs::Joy::ConstPtr &joy) {
	//ROS_INFO("joystck ---------------");
	if (joy->buttons[1]) // kill switch
	{
		stopPushed = true;
		trn_rad = STEERING_MIDDLE_POSITION;
		mtr_ctrl = 0;
		return;
	} else if (joy->buttons[0] or not stopPushed) //OR stopPushed , uncomment below
			{
		stopPushed = false;
		update_motor_control(joy, max_torque);
		update_turning_radius(joy);
	} else if (joy->buttons[2]) {
		if (not speedTest) {
			mtr_ctrl = 0;
			speedTest = true;
		}
		if (mtr_ctrl < max_torque) //(mtr_ctrl < 50 )
				{
			mtr_ctrl += 10; // increment speed/throttle commands in steps of 10 mm/s
			ROS_INFO("Setting mtr_ctrl to %u", mtr_ctrl);
		} else {
			speedTest = false;
			mtr_ctrl = 0; // stopping motor
			ROS_INFO("Ending test and setting mtr_ctrl to %u", mtr_ctrl);
		}
	} else if (joy->buttons[3]) {
		if (not steeringTest) {
			trn_rad = 0;
			steeringTest = true;
		}
		uint16_t maxValue = 65535;

		if (trn_rad < maxValue) {
			uint32_t new_trn_rad = trn_rad + (32768 / 5);
			trn_rad =
					(new_trn_rad > maxValue) ?
							static_cast<uint16_t>(maxValue) : new_trn_rad; // maximun is STEERING_MIDDLE_POSITION * 2
			ROS_INFO("Setting new_trn_rad to %u", trn_rad);
		} else {
			mtr_ctrl = 0; //make sure the motor is off
			trn_rad = STEERING_MIDDLE_POSITION;
			ROS_INFO("Ending test and setting trn_rad to %d", trn_rad);
			steeringTest = false;
		}
	}
	std::cout << trn_rad << "  " << mtr_ctrl << std::endl;
}

void TeleopJoyNode::send_controls(ros::ServiceClient &client) const {
	atv_can::DriveService service_call;

	service_call.request.turning_radius = trn_rad;
	service_call.request.motor_control = mtr_ctrl;
	service_call.request.gear_ratio = 0;
	service_call.request.all_wheel_drive = 0;
	service_call.request.control_mode = false; // Torque control (set to false for velocity
	// set point control.
	service_call.request.direction = true;
	if (client.call(service_call)) {
		//std::cout << "sent-received" << service_call.response.status << std::endl;
		//ROS_DEBUG("message sent");
	} else {
		//std::cout << "Failed to call service" << std::endl;
		ROS_ERROR("Failed to call service");
	}
}

void TeleopJoyNode::update_turning_radius(
		const sensor_msgs::Joy::ConstPtr &joy) {
	// joy->axes[0] is floating point number from -32768 to 32767
	trn_rad = 32767 + static_cast<uint16_t>(joy->axes[3]);
}

void TeleopJoyNode::update_motor_control(const sensor_msgs::Joy::ConstPtr &joy,
		int max_torque) {
	mtr_ctrl = static_cast<int16_t>(-max_torque * joy->axes[1]
			/ JOYSTICK_INPUT_SCALE);
}

void TeleopJoyNode::runTest() {

}

int main(int argc, char **argv) {
	ros::init(argc, argv, "teleop_joy");
	ros::NodeHandle nh_("~"); //Private nodehandle
	ros::NodeHandle nh;
	//int max_torque;

//  if (nh_.getParam("max_torque", max_torque))
//  {
//    if (max_torque < 0 || max_torque > 100)
//    {
//      ROS_WARN("Torque value out of bounds, using default torque 50");
//      max_torque = 50;
//    }
//    else
//    {
//      ROS_INFO("Setting maximum torque to %d", max_torque);
//    }
//  }
//  else
//  {
//    ROS_WARN("Maximum torque not set, using default 50");
//    max_torque = 50;
//  }
//
	if (nh_.getParam("max_torque", max_torque)) // MAX_TORQUE IS MAX_SPEED HERE
			{
		if (max_torque < 0 || max_torque > 5000) {
			ROS_WARN("Speed value out of bounds, using default speed 5000");
			max_torque = 5000;
		} else {
			ROS_INFO("Setting maximum 5000 to %d", max_torque);
		}
	} else {
		ROS_WARN("Maximum speed not set, using default 5000");
		max_torque = 5000;
	}
	// Wait for the can_service to start
	ros::service::waitForService("can_service");
	std::cout << "can service started" << std::endl;

	ros::Rate loop_rate(10);
	TeleopJoyNode teleop_joy((int16_t) max_torque);

	ros::ServiceClient client(
			nh.serviceClient<atv_can::DriveService>("can_service"));

	ros::Subscriber subscriber = nh.subscribe<sensor_msgs::Joy>("joy", 0,
			&TeleopJoyNode::read_joystick_input, &teleop_joy);
	//signal(SIGINT,quit);
	int64_t count = 0;

	while (ros::ok()) {
		if (nh_.getParam("max_torque", max_torque)) // MAX_TORQUE IS MAX_SPEED HERE
				{
			if (max_torque < 0 || max_torque > 5000) {
				ROS_WARN_STREAM_ONCE("Speed value out of bounds, using default speed 5000");
				max_torque = 5000;
			}
		}
		teleop_joy.send_controls(client);
		//std::cout << "Entering teleop loop" << std::endl;
		ros::spinOnce();
		loop_rate.sleep();
		count++;
	}
	return (0);
}
