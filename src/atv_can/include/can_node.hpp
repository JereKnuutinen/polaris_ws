#ifndef ATV_CAN_CAN_NODE_HEADER
#define ATV_CAN_CAN_NODE_HEADER

#include "can_bus_handler.hpp"
#include "atv_can/DriveService.h"
#include "atv_can/OdometryMeasurement.h"
#include "atv_can/SteeringMeasurement.h"
#include "atv_can/DashGasPedalMeasurement.h"
#include "atv_can/WheelDisplacementMeasurement.h"
#include "atv_can/SpeedandSteeringCommands.h"
#include "atv_can/MachineSelectedSpeed.h"
#include "atv_can/GuidanceMachineStatus.h"

#include <sensor_msgs/Joy.h>
#include "boost/thread.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

namespace atv_can {
    class CanNode {
    public:
        CanNode(int CAN_bitrate, int CAN_channel, unsigned int read_timeout_ms, unsigned int publications_per_sec);
        ~CanNode();

        void run();
        void joy_callback(const sensor_msgs::Joy::ConstPtr& msg);
    private:
        SocketCanHandle *can_socket_handler;
        //CanBusHandler can_bus_handler;
        atv_can::OdometryMeasurement odo_msg;
        atv_can::SteeringMeasurement steer_msg;
        atv_can::DashGasPedalMeasurement gas_msg;
        atv_can::WheelDisplacementMeasurement height_msg;
        atv_can::SpeedandSteeringCommands commands_msg;
        atv_can::MachineSelectedSpeed mss_msg;
        atv_can::GuidanceMachineStatus guidance_msg;

        ros::Publisher odo_publisher;
        ros::Publisher steer_publisher;
        ros::Publisher gas_publisher;
        ros::Publisher displacement_publisher;
        ros::Publisher commands_publisher;
        ros::Publisher mss_publisher;
        ros::Publisher guidance_publisher;

        ros::Subscriber joy_subscriber;

        ros::ServiceServer can_service;

        unsigned int read_timeout_ms;
        double publication_frequency_hz;
        bool keep_reading;

        CanNode(const CanNode& other);
        CanNode& operator=(const CanNode& other);

        uint64_t generate_can_message_data(const DriveService::Request& req);
        bool serve_send_request(DriveService::Request& req, DriveService::Response& resp);
        void read_next_message();
        void read_forever();
        void publish_service_data(const CanMsg& msg);

        //void run_publisher(ros::NodeHandle& node_handle);
    };
}

#endif
