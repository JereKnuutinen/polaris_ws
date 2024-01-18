

// sudo ip link set can0 up type can bitrate 250000
//

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <sstream>
#include <thread>

//using namespace std::chrono_literals;


/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */


class SocketCanHandle
{
  public:
  SocketCanHandle(std::string interfaceName, std::string loggername)
  {
    this->connected = false;
    this->socketHandle = -1;
    this->interfaceName = interfaceName;
    this->loggername = loggername;
  }

  bool IsConnected(void)
  {return this->connected;}

  bool Connect(void)
  {
    if(!this->connected)
    {
      if ((socketHandle = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
      {
        ROS_INFO("SocketCanHandle: Failed to create socket");
        return false;
      }


      struct ifreq ifr;
      strcpy(ifr.ifr_name, this->interfaceName.c_str());
      ioctl(this->socketHandle, SIOCGIFINDEX, &ifr);


      struct sockaddr_can addr;
      memset(&addr, 0, sizeof(addr));
      addr.can_family = AF_CAN;
      addr.can_ifindex = ifr.ifr_ifindex;
      //addr.can_addr.j1939.addr = 
      //addr.can_addr.j1939.name = 
      //addr.can_addr.j1939.pgn = 

      if (bind(this->socketHandle, (struct sockaddr *)&addr, sizeof(addr)) < 0)
      {
        perror("Bind");
        ROS_INFO("SocketCanHandle: Failed to bind socket");
        return false;
      }

      this->connected = true;
      ROS_INFO("SocketCanHandle: CAN socket opened successfully");
    }
    else
    {
      ROS_INFO("SocketCanHandle: CAN socket already connected");
    }

    return this->connected;
  }

  void Disconnect(void)
  {
    if (this->socketHandle > 0 && close(this->socketHandle) < 0)
    {
      ROS_INFO("SocketCanHandle: Error when closing socket");
    }
    this->socketHandle = -1;
    this->connected = false;
  }

  bool Reconnect(void)
  {
    this->Disconnect();
    return this->Connect();
  }

  bool ReceiveFrame(struct can_frame *frame)
  {
    if(!this->connected)
    {

      ROS_INFO("SocketCanHandle: Receive error, socket not connected");
      return false;
    }

    int nbytes = read(this->socketHandle, frame, sizeof(struct can_frame));

    if (nbytes < 0)
    {
      ROS_INFO("SocketCanHandle: CAN Sockets READ FAILED");
      return false;
    }

    ROS_INFO("SocketCanHandle: received object - 0x%03X [%d] ", (frame->can_id >> 8) & 0x0ffff, frame->can_dlc);

    printf("0x%03X [%d] ", frame->can_id, frame->can_dlc);
    for (int i = 0; i < frame->can_dlc; i++)
       printf("%02X ", frame->data[i]);

    printf("\r\n");

    return true;
  } 

  bool SendFrame(struct can_frame frame)
  {
    return false;
  }

  private:
//
//  ros::Logger get_logger()
//  {
//    return ros::get_logger(loggername);
//  }

  bool connected;
  int socketHandle;
  std::string interfaceName;
  std::string loggername;
};



class CanValtra : public ros::NodeHandle
{
public:
  CanValtra():NodeHandle("can_valtra"), count_(0)
  {

    //publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10); //original
    //ros::Publisher publisher_ = create_publisher<std_msgs::String>("topic", 10);

//    timer_ = this->create_wall_timer(1000ms, std::bind(&CanValtra::timer_callback, this));

    canHandle = new SocketCanHandle("can0", "loggername");//getLogger(std::string& name);//(this->get_logger().get_name()));
    //std::cout <<"debug"<<std::endl;
    canHandle->Connect();
    //listener_thread = new std::thread(&CanValtra::receive_forever, this);
    //std::thread listener_thread(&CanValtra::receive_forever, this);

  }

  void receive_forever(void)
  {
	//ros::Rate r(5);
    while(ros::ok())
    {
      //r.sleep();
      if(canHandle->IsConnected()) //error here
      {
        struct can_frame frame;
        canHandle->ReceiveFrame(&frame);
        //Parse frame to messages
      }
    }
  }

private:
  void timer_callback()
  {
    canHandle->Connect();
    //struct can_frame frame;
    //canHandle->ReceiveFrame(&frame);
  }

  //ros::TimerBase::SharedPtr timer_;
  //ros::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
  SocketCanHandle *canHandle; //add call parameters
  std::thread *listener_thread;

};

int main(int argc, char** argv) {
	ros::init(argc, argv, "can_socket_node");
	CanValtra nh_;
	std::thread listener_thread(&CanValtra::receive_forever, &nh_);
	ros::spin();
	listener_thread.join();
	return 0;
}
