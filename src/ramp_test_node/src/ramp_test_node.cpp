
#include "ros/ros.h"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Accel.h>  
#include <vector>
#include "atv_can/DriveService.h" 
#include <chrono>
#include "time.h"
#include <memory>


#include <termios.h>
#include <cstdio>
#include <iostream>
// void tic(int mode=0) {
//     static std::chrono::_V2::system_clock::time_point t_start;
    
//     if (mode==0)
//         t_start = std::chrono::high_resolution_clock::now();
//     else {
//         auto t_end = std::chrono::high_resolution_clock::now();
//         std::cout << "Elapsed time is " << (t_end-t_start).count()*1E-9 << " seconds\n";
//     }
// }
// void toc() { tic(1); }

char getch()
{
    std::cout << "meni keyboard fun " << std::endl;
    fd_set set;
    struct timeval timeout;
    int rv;
    char buff = 0;
    int len = 1;
    int filedesc = 0;
    FD_ZERO(&set);
    FD_SET(filedesc, &set);

    timeout.tv_sec = 0;
    timeout.tv_usec = 1000;

    rv = select(filedesc + 1, &set, NULL, NULL, &timeout);

    struct termios old = {0};
    if (tcgetattr(filedesc, &old) < 0)
        ROS_ERROR("tcsetattr()");
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(filedesc, TCSANOW, &old) < 0)
        ROS_ERROR("tcsetattr ICANON");

    if(rv == -1)
        ROS_ERROR("select");
    else if(rv == 0)
        ROS_INFO("no_key_pressed");
    else
        read(filedesc, &buff, len );

    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(filedesc, TCSADRAIN, &old) < 0)
        ROS_ERROR ("tcsetattr ~ICANON");
    return (buff);
}
class RampTestNode {
public:
    RampTestNode() {
        ros::NodeHandle nh;

        cmd_pub_ = nh.advertise<nav_msgs::Odometry>("/ramp_out", 100);

        // ROS service client
        atvServiceClient_ = nh.serviceClient<atv_can::DriveService>("/can_service");

    }


    void runRampControl() {
        //*ptr = (void *) nmpc;
        //set initial control trajectory
        int i,j;

        // // // Wait for ATV service server
        // if (!atvServiceClient_.waitForExistence(ros::Duration(10.0))) {
        //     ROS_ERROR("ATV service server not available on network (Ramp NODE) !!!!!!");
        //     //return -1;
        // }


        // Thisng related to keyboard stuff /////////////////
        int init_or_opt = 0; // zero for zero inputs and 1 for opt inputs
        ros::Rate rate(20); // ROS Rate at 5Hz
        /////////////////////////////////////////////////////
        //atv_can::DriveService srv;
        double Vcmd = 0;
        double Kcmd = 0;
        double acmd = 2.0;
        double dkcmd = 0.01;

        double K_max = 0.05; 
        double K_min = -0.05;
        double amplitude = (K_max - K_min) / 2;  // amplitude of the sinusoid
        double frequency = 0.1;     // frequency of the sinusoid
        
        while (ros::ok()) {
  

            //run NMPC
            //std::cout << "optimize nmpc" << std::endl;
            //tic();
            char c = getch();   // call your non-blocking input function
            if (c == 'k') {
                init_or_opt = 1;
            }
            if (c == 'i') {
                init_or_opt = 0;
            }
            if (c == 's') {
                init_or_opt = 2;
            }
            if (c == 'd') {
                init_or_opt = 3;
            }
            if (c == 'a') {
                init_or_opt = 4;
            }
            if (c == 'f') {
                init_or_opt = 5;
            }
           
            if (init_or_opt == 1){
                Kcmd = Kcmd + dkcmd * 0.05;
                if (Kcmd > (K_max - dkcmd * 0.05) || Kcmd < (K_min - dkcmd * 0.05)) {
                    dkcmd = -dkcmd;
                }
            } 
            if (init_or_opt == 0) {
                Kcmd = 0.0;
            }
            
            if (init_or_opt == 2) {
                Kcmd = 0.02;
            }
            if (init_or_opt == 3) {
                Kcmd = -0.02;
            }
            if (init_or_opt == 4) {
                Kcmd = 0.04;
            }
            if (init_or_opt == 5) {
                Kcmd = -0.04;
            }
            std::cout << "Vcmd command is " << Vcmd << std::endl;
            std::cout << "Kcmd command is " << Kcmd << std::endl;
            nav_msgs::Odometry control_state_msg;
            control_state_msg.header.stamp = ros::Time::now();
            control_state_msg.pose.pose.position.x =  Vcmd; // 
            control_state_msg.pose.pose.position.y =  Kcmd; // 
            control_state_msg.pose.pose.position.z =  dkcmd; //
            control_state_msg.twist.twist.linear.x =  acmd;

            // Publish the Accel message
            cmd_pub_.publish(control_state_msg);

            double stgCmd = atan(1.83 * Kcmd) * 180.0 / M_PI; // steering command, convert to degrees
            // atv service
            // %K_ref=[-0.000994869050617; 32.649869050616971];
            //Turn_Ref = uint16((stg_cmd - K_ref(2))/K_ref(1));
            atv_can::DriveService srv;
            srv.request.motor_control = static_cast<int16_t>(Vcmd * 1000.0); // mm/s
            srv.request.turning_radius = static_cast<uint16_t>(((stgCmd-32.649869050616971)/(-0.000994869050617))); // 
            srv.request.all_wheel_drive = static_cast<uint8_t>(0);
            srv.request.gear_ratio = static_cast<uint8_t>(0);
            srv.request.control_mode = false; // true for torque, false for speed
            srv.request.direction = true;

            if (atvServiceClient_.call(srv)) {
                //srv.response;
            } else {
                std::cout << "error in service call (VIATOC NODE)" << std::endl;
                ROS_ERROR("Failed to call ATV service from VIATOC node");
            }

            rate.sleep();
        }
    }

private:
    ros::Publisher cmd_pub_; // Controlcommand publisher
    ros::ServiceClient atvServiceClient_;
    int test_muuttuja;

    // state for kinematic + 3 dof model
    // for x y path tracking is used and for velocity refrence
    // [x, y, psi, v, v_lowpass, K, K_lowpass, S, z, z_dot, roll, roll_dot, pitch, pitch_dot]

    // For old NMPC: [x, y, psi, v, v_lowpass, K, K_lowpass, S, z, z_dot, roll, roll_dot, pitch, pitch_dot]
    //std::vector<double> x_ref_ = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "ramp_test_node");
    RampTestNode ramp_controller;

    ramp_controller.runRampControl();

    return 0;
}

