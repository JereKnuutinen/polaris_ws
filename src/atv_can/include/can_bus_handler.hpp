#ifndef ATV_CAN_CANCOMMUNICATOR_HEADER
#define ATV_CAN_CANCOMMUNICATOR_HEADER

#include <canlib.h>
#include "can_msg.hpp"
#include <thread>
#include <mutex>
#include <exception>
#include <utility>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

namespace atv_can {

    struct CanMsgStatusInfo {
        canStatus status_code;
        std::string status_string;
        CanMsgStatusInfo(canStatus sc, const std::string& ss) : status_code(sc), status_string(ss) {}
    };

    class CanBusError : public std::runtime_error {
    public:
        CanBusError(const std::string& msg) : std::runtime_error(std::string("CAN bus error: ") + msg) {}
    };

    /*
    * Simple class for reading and writing CAN bus data.
    *
    * Wraps all required functionality from canlib, except
    * canInitializeLibrary(), which needs to be called prior to creating an instance of this class.
    *
    * Reads and writes are synchronized so a CanBusHandler will not write to the CAN bus while it is reading and vice versa.
    * Note that the synchronization happens only at instance level (using std::mutex), which means that the bus is not protected
    * from several instances of this class running simultaneously (whether within the same program or in two separate applications).
    * Simply DO NOT create multiple simultaneous instances of this class on the same CAN bus and you will be fine.
    *
    */
    class CanBusHandler {
    public:
        CanBusHandler(int bitrate, int channel);
        ~CanBusHandler();

        CanMsgStatusInfo read_raw_message(CanMsg& message, unsigned int timeout_ms);
        CanMsgStatusInfo send_raw_message(const CanMsg& msg);
    private:
        std::mutex can_bus_mutex;
        canHandle handle;

        // class is not copyable:
        CanBusHandler(const CanBusHandler& other);
        CanBusHandler& operator=(const CanBusHandler& other);
    };

    class SocketCanHandle
    {
      public:
      //SocketCanHandle(std::string interfaceName, std::string loggername)
    	SocketCanHandle()
    	{
        this->connected = false;
        this->socketHandle = -1;
        this->interfaceName = "can0";
        this->loggername = "loggername";
        Connect();
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
          // Added this condition:
         //if (ioctl(this->socketHandle, SIOCGIFINDEX, &ifr) == -1) {
        	//  ROS_INFO("SocketCanHandle: failed to get CAN interface index");
        	  //return false;
          //}



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

    	//printf("can_bus_handler.hpp: RECEIVED [%d] bytes can_frame id:0x%03X, can dlc: [%d] \n",nbytes, frame->can_id, frame->can_dlc);

        if (nbytes < 0)
        {
          ROS_INFO("SocketCanHandle: CAN Sockets READ FAILED");
          return false;
        }

        //ROS_INFO("SocketCanHandle: received object - 0x%03X [%d] ", (frame->can_id >> 8) & 0x0ffff, frame->can_dlc);

        /*printf("0x%03X [%d] ", frame->can_id, frame->can_dlc);
        for (int i = 0; i < frame->can_dlc; i++)
           printf("%02X ", frame->data[i]);

        printf("\r\n");*/

        return true;
      }

      bool SendFrame(struct can_frame *frame)
      {
    	  if(!this->connected)
    	          {

    	            ROS_INFO("SocketCanHandle: Write error, socket not connected");
    	            return false;
    	          }

    	          int nbytes = write(this->socketHandle, frame, sizeof(struct can_frame));

    	          //printf("\nwrote [%d] bytes \n", nbytes);
    	          if (nbytes < 0)
    	          {
    	            ROS_INFO("SocketCanHandle: CAN Sockets WRITE FAILED");
    	            return false;
    	          }


        return true;
      }

      private:
    //
    //  ros::Logger get_logger()
    //  {
    //    return ros::get_logger(loggername);
    //  }
      std::mutex can_bus_mutex;
      //canHandle handle;
      bool connected;
      int socketHandle;
      std::string interfaceName;
      std::string loggername;
    };

}

#endif
