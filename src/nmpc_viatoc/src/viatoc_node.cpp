
#include "ros/ros.h"
#include <iostream>
#include "NMPCProblem.h" // Include MPC solver header file
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
class MPCControllerNode {
public:
    MPCControllerNode() {
        ros::NodeHandle nh;
        // Subscribe to EKF data
        EKF_state_estimate_sub_ = nh.subscribe("/ekf_out1", 1, &MPCControllerNode::EKFCallback, this);

        // Publisher for control commands
        cmd_pub_ = nh.advertise<geometry_msgs::Accel>("/nmpc_out", 100);
        control_state_pub_ = nh.advertise<nav_msgs::Odometry>("/control_state", 100);

        // ROS service client
        atvServiceClient_ = nh.serviceClient<atv_can::DriveService>("/can_service");

        // MPC solver initialization
        //nmpc = new NMPCProblem();
        nmpc = std::make_shared<NMPCProblem>();

        // Initialize the flags to false
        new_sensor_data_available_ = false;
        is_initialization_done_ = false;
    }

    // void EKFCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    //     // body to global (CG) transformation
    //     // C = Rz(yaw)*Ry(pitch)*Rx(roll);
    //     // compute velocities in inertial frame
    //     // dotENU = C*[u; v; w];
    //     // dE = dotENU(1); dN = dotENU(2); dU = dotENU(3);

    //     // Process and store sensor data (e.g., state estimation)
    //     ROS_INFO("Received Odometry message:\n%s", msg->header.frame_id.c_str());
    //     // Extract relevant information from the Odometry message
    //     // For old model
    //     double x = msg->pose.pose.position.x;
    //     double y = msg->pose.pose.position.y;
    //     double z = msg->pose.pose.position.z;

    //     // std::cout <<"x " << msg->pose.pose.position.x << std::endl;
    //     // std::cout <<"y " << msg->pose.pose.position.y << std::endl;
    //     // std::cout <<"z " << msg->pose.pose.position.z << std::endl;

    //     double u = msg->twist.twist.linear.x;
    //     double v = msg->twist.twist.linear.y;
    //     double w = msg->twist.twist.linear.z;

    //     double V = u; //sqrt(u*u + v*v);

    //     double p = msg->twist.twist.angular.x; // roll_rate
    //     double q = msg->twist.twist.angular.y; // pitch_rate
    //     double r = msg->twist.twist.angular.z; // yaw rate

    //     double psi = msg->twist.covariance[6];
    //     //std::cout << psi << std::endl;
    //     double roll = msg->twist.covariance[4];
    //     double pitch = msg->twist.covariance[5];

    //     // compute z_dot in inertial frame
    //     double z_dot = w*cos(pitch)*cos(roll) - u*sin(pitch) + v*cos(pitch)*sin(roll);

    //     // Use psi, roll and pitch to calculate derivatives of angles from angle rates
    //     double yaw_dot   = (q*sin(roll) + r*cos(roll))/cos(pitch);
    //     double roll_dot = (q*cos(roll) - r*sin(roll));
    //     double pitch_dot = yaw_dot*sin(pitch) + p;

    //     double x_quat = msg->pose.pose.orientation.x;
    //     double y_quat = msg->pose.pose.orientation.y;
    //     double z_quat = msg->pose.pose.orientation.z;
    //     double w_quat = msg->pose.pose.orientation.w;

    //     double K = msg->twist.covariance[7];
    //     double lla0_lat_ = msg->twist.covariance[1];
    //     double lla0_lon_ = msg->twist.covariance[2];
    //     double lla0_at_ = msg->twist.covariance[3];
        
    //     double distance = 1.37;
    //     EKF_state_ = {x, y, psi, V, V, K, K, distance, z, z_dot, roll, roll_dot, pitch, pitch_dot};
    //     //std::cout << "state: " << std::endl;
    //     //for (auto it = EKF_state_.begin(); it != EKF_state_.end(); ++it)
    //     //    std::cout << std::setprecision(15) << *it << std::endl;
    //     lla0_ = {lla0_lat_, lla0_lon_, lla0_at_};
    //     // Set the flag to indicate new sensor data is available
    //     new_sensor_data_available_ = true;
    // }

    // void EKFCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    //         // body to global (CG) transformation
    //         // C = Rz(yaw)*Ry(pitch)*Rx(roll);
    //         // compute velocities in inertial frame
    //         // dotENU = C*[u; v; w];
    //         // dE = dotENU(1); dN = dotENU(2); dU = dotENU(3);

    //         // Process and store sensor data (e.g., state estimation)
    //         ROS_INFO("Received Odometry message:\n%s", msg->header.frame_id.c_str());
    //         // Extract relevant information from the Odometry message
    //         // For old model
    //         double x = msg->pose.pose.position.x;
    //         double y = msg->pose.pose.position.y;
    //         double z = msg->pose.pose.position.z;
    //         double l2 = 1.83/2;
    //         double h2 = 0.8767;

    //         // std::cout <<"x " << msg->pose.pose.position.x << std::endl;
    //         // std::cout <<"y " << msg->pose.pose.position.y << std::endl;
    //         // std::cout <<"z " << msg->pose.pose.position.z << std::endl;

    //         double u = msg->twist.twist.linear.x;
    //         double v = msg->twist.twist.linear.y;
    //         double w = msg->twist.twist.linear.z;

    //         double V = u; //sqrt(u*u + v*v);

    //         double p = msg->twist.twist.angular.x; // roll_rate
    //         double q = msg->twist.twist.angular.y; // pitch_rate
    //         double r = msg->twist.twist.angular.z; // yaw rate

    //         double psi = msg->twist.covariance[6];
    //         //std::cout << psi << std::endl;
    //         double roll = msg->twist.covariance[4];
    //         double pitch = msg->twist.covariance[5];

    //         double x_rear = x - h2*(sin(psi)*sin(roll) + cos(psi)*cos(roll)*sin(pitch)) - l2*cos(pitch)*cos(psi);
    //         double y_rear = y + h2*(cos(psi)*sin(roll) - cos(roll)*sin(pitch)*sin(psi)) - l2*cos(pitch)*sin(psi);
    //         double z_rear =  z + l2*sin(pitch) - h2*cos(pitch)*cos(roll);

    //         // compute z_dot in inertial frame
    //         double z_dot = w*cos(pitch)*cos(roll) - u*sin(pitch) + v*cos(pitch)*sin(roll);

    //         // Use psi, roll and pitch to calculate derivatives of angles from angle rates
    //         double yaw_dot   = (q*sin(roll) + r*cos(roll))/cos(pitch);
    //         double roll_dot = (q*cos(roll) - r*sin(roll));
    //         double pitch_dot = yaw_dot*sin(pitch) + p;

    //         double x_quat = msg->pose.pose.orientation.x;
    //         double y_quat = msg->pose.pose.orientation.y;
    //         double z_quat = msg->pose.pose.orientation.z;
    //         double w_quat = msg->pose.pose.orientation.w;

    //         double K = msg->twist.covariance[7];
    //         double lla0_lat_ = msg->twist.covariance[1];
    //         double lla0_lon_ = msg->twist.covariance[2];
    //         double lla0_at_ = msg->twist.covariance[3];
            
    //         double distance = 1.37;
    //     //[x, y, z, psi, u, u_lowpass, K, K_lowpass, w, roll, p, pitch, q]
    //         EKF_state_ = {x, y, z, psi, V, V, K, K, w, roll, p, pitch, q};
    //         //std::cout << "state: " << std::endl;
    //         //for (auto it = EKF_state_.begin(); it != EKF_state_.end(); ++it)
    //         //    std::cout << std::setprecision(15) << *it << std::endl;
    //         lla0_ = {lla0_lat_, lla0_lon_, lla0_at_};
    //         // Set the flag to indicate new sensor data is available
    //         new_sensor_data_available_ = true;
    //     }

        void EKFCallback(const nav_msgs::Odometry::ConstPtr& msg) {
            // body to global (CG) transformation
            // C = Rz(yaw)*Ry(pitch)*Rx(roll);
            // compute velocities in inertial frame
            // dotENU = C*[u; v; w];
            // dE = dotENU(1); dN = dotENU(2); dU = dotENU(3);

            // Process and store sensor data (e.g., state estimation)
            //ROS_INFO("Received Odometry message:\n%s", msg->header.frame_id.c_str());
            // Extract relevant information from the Odometry message
            // For old model
            double x = msg->pose.pose.position.x;
            double y = msg->pose.pose.position.y;
            double z = msg->pose.pose.position.z;
            double l2 = 1.83/2;
            double h2 = 0.8767;

            // std::cout <<"x " << msg->pose.pose.position.x << std::endl;
            // std::cout <<"y " << msg->pose.pose.position.y << std::endl;
            // std::cout <<"z " << msg->pose.pose.position.z << std::endl;

            double u = msg->twist.twist.linear.x;
            double v = msg->twist.twist.linear.y;
            double w = msg->twist.twist.linear.z;

            double V = u; //sqrt(u*u + v*v);

            double p = msg->twist.twist.angular.x; // roll_rate
            double q = msg->twist.twist.angular.y; // pitch_rate
            double r = msg->twist.twist.angular.z; // yaw rate

            double psi = msg->twist.covariance[6];
            //std::cout << psi << std::endl;
            double roll = msg->twist.covariance[4];
            double pitch = msg->twist.covariance[5];

            //double x_rear = x - h2*(sin(psi)*sin(roll) + cos(psi)*cos(roll)*sin(pitch)) - l2*cos(pitch)*cos(psi);
            //double y_rear = y + h2*(cos(psi)*sin(roll) - cos(roll)*sin(pitch)*sin(psi)) - l2*cos(pitch)*sin(psi);
            //double z_rear =  z + l2*sin(pitch) - h2*cos(pitch)*cos(roll);

            // compute z_dot in inertial frame
            //double z_dot = w*cos(pitch)*cos(roll) - u*sin(pitch) + v*cos(pitch)*sin(roll);

            // Use psi, roll and pitch to calculate derivatives of angles from angle rates
            //double yaw_dot   = (q*sin(roll) + r*cos(roll))/cos(pitch);
            //double roll_dot = (q*cos(roll) - r*sin(roll));
            //double pitch_dot = yaw_dot*sin(pitch) + p;

            //double x_quat = msg->pose.pose.orientation.x;
            //double y_quat = msg->pose.pose.orientation.y;
            //double z_quat = msg->pose.pose.orientation.z;
            //double w_quat = msg->pose.pose.orientation.w;

            double K = msg->twist.covariance[7];
            double lla0_lat_ = msg->twist.covariance[1];
            double lla0_lon_ = msg->twist.covariance[2];
            double lla0_at_ = msg->twist.covariance[3];
            
            double distance = 1.37;
        //[x, y, z, psi, u, u_lowpass, K, K_lowpass, roll, p, pitch, q]
            EKF_state_ = {x, y, z, psi, V, V, K, K, roll, p, pitch, q};
            //std::cout << "state: " << std::endl;
            //for (auto it = EKF_state_.begin(); it != EKF_state_.end(); ++it)
            //    std::cout << std::setprecision(15) << *it << std::endl;
            lla0_ = {lla0_lat_, lla0_lon_, lla0_at_};
            // Set the flag to indicate new sensor data is available
            new_sensor_data_available_ = true;
        }

    void runMPCControl() {
        //*ptr = (void *) nmpc;
        //set initial control trajectory
        int i,j;

        // // // Wait for ATV service server
        // if (!atvServiceClient_.waitForExistence(ros::Duration(10.0))) {
        //     ROS_ERROR("ATV service server not available on network (VIATOC NODE) !!!!!!");
        //     //return -1;
        // }
        double acmd;
        double dkcmd;
        double Vcmd;
        double Kcmd;


        // Thisng related to keyboard stuff /////////////////
        int init_or_opt = 0; // zero for zero inputs and 1 for opt inputs
        ros::WallTime start_2, end_2;
        /////////////////////////////////////////////////////
        //atv_can::DriveService srv;
        while (ros::ok()) {
            //Check if new sensor data has arrived
            if (new_sensor_data_available_) {
                  //tic();
                ros::WallTime start_, end_;
                auto start_chrono = std::chrono::high_resolution_clock::now();
                start_ = ros::WallTime::now();
                start_2 = ros::WallTime::now();
                // Check if the initialization is done
                if (!is_initialization_done_) {
                    std::cout << "initialize nmpc (VIATOC NODE)" << std::endl;
                    initializeMPC(EKF_state_);
                    is_initialization_done_ = true;
                    //toc();
                }

                // Set current state
                for(i = 0; i < nmpc->numStates; i++)
                    nmpc->x[i] = EKF_state_[i];

                //

                // set x_ref
                for(i = 0; i < nmpc->numStates*nmpc->numSteps; i++) {
                    nmpc->x_ref[i] = x_ref_[i % x_ref_.size()];
                }

                // set u_ref
                for(i = 0; i < nmpc->numControls*nmpc->numSteps; i++) {
                    nmpc->u_ref[i] = u_ref_[i % u_ref_.size()];
                }

                //run NMPC
                //std::cout << "optimize nmpc" << std::endl;
                //tic();
                char c = getch();   // call your non-blocking input function
                if (c == 'o') {
                    init_or_opt = 1;
                }
                if (c == 'i') {
                    init_or_opt = 0;
                }
                //std::cout << "printtaapi " << c << std::endl;
                //struct timespec tic, toc; 
                //clock_gettime(CLOCK_MONOTONIC, &tic);
                struct timespec start, end;
                clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &start);
                nmpc->optimize(50);
                auto end_chrono = std::chrono::high_resolution_clock::now();

                //clock_gettime(CLOCK_MONOTONIC, &toc);
                clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &end);
                end_ = ros::WallTime::now();
                end_2 = ros::WallTime::now();
                // print results
                double execution_time = (end_ - start_).toNSec() * 1e-9;
                double execution_time2 = (end_2 - start_2).toNSec() * 1e-9;
                std::cout << "Time (ROS walltime) (s)" << execution_time << std::endl;
                std::cout << "Time (ROS walltime2) (s)" << execution_time2<< std::endl;

                //toc();
                //std::cout << "Aika " << (toc.tv_sec - tic.tv_sec) + (toc.tv_nsec - tic.tv_nsec) / 1.0e9 << std::endl;
                // Calculate the elapsed time
                long seconds = end.tv_sec - start.tv_sec;
                long nanoseconds = end.tv_nsec - start.tv_nsec;
                double elapsed = seconds + nanoseconds*1e-9;
                std::cout << "Time (clock_gettime) (s) " << elapsed << std::endl;
                std::cout << "Duration loop(Chrono CPU Time): " << std::chrono::duration<double, std::milli>(end_chrono-start_chrono).count() << "ms"<< std::endl;
                //std::cout << " after optimize nmpc" << std::endl;
                acmd = nmpc->u[0];
                dkcmd = nmpc->u[1];
                // Take values from second step. since the first corrensponds to current state
                if (init_or_opt == 1){
                    Vcmd = nmpc->x[16];
                    Kcmd = nmpc->x[18];
                } else {
                    Vcmd = 0.0;
                    Kcmd = 0.0;
                }
                // std::cout << "acc " << acmd << std::endl;
                // std::cout << "dkcmd1 " << dkcmd << std::endl;
                // std::cout << " Iteraatio " << std::endl;
                // for (int iii = 0; iii < 100; iii++) {
                //     std::cout << nmpc->x[(iii)*12 + 2] << std::endl;
                // }
                std::cout << "Vcmd command is " << Vcmd << std::endl;
                std::cout << "Kcmd command is " << Kcmd << std::endl;
                //std::cout << "Velocity command " << Vcmd << std::endl;
                //std::cout << "curv command " << Kcmd << std::endl;
                // Create and populate the Accel message
                geometry_msgs::Accel cmd_msg;
                cmd_msg.linear.x = acmd; 
                cmd_msg.linear.y = dkcmd;      
                cmd_msg.linear.z = 0.0;

                cmd_msg.angular.x = Vcmd;     
                cmd_msg.angular.y = Kcmd;
                cmd_msg.angular.z = 0.0;

                nav_msgs::Odometry control_state_msg;
                control_state_msg.header.stamp = ros::Time::now();
                control_state_msg.pose.pose.position.x =  nmpc->x[12]; // x
                control_state_msg.pose.pose.position.y =  nmpc->x[13]; // y
                control_state_msg.pose.pose.position.z =  nmpc->x[14]; // z

                control_state_msg.twist.twist.linear.x =  nmpc->x[16]; // Forward velocity
                //control_state_msg.twist.twist.linear.z =  nmpc->x[20]; // Up velocity

                control_state_msg.pose.pose.orientation.x = nmpc->x[20]; // roll
                control_state_msg.pose.pose.orientation.y = nmpc->x[23]; // pitch
                control_state_msg.pose.pose.orientation.z = nmpc->x[15]; // heading

                control_state_msg.twist.twist.angular.x = nmpc->x[22]; // roll rate 
                control_state_msg.twist.twist.angular.y = nmpc->x[24]; //  pitch rate
                control_state_msg.twist.twist.angular.z = nmpc->x[18]; //  curvature

                // Put also lowpass states to the cov message

                control_state_msg.twist.covariance[0] = execution_time; // Optimization time
                control_state_msg.twist.covariance[1] = EKF_state_[0]; // x
                control_state_msg.twist.covariance[2] = EKF_state_[1]; // y
                control_state_msg.twist.covariance[3] = EKF_state_[2]; // z
                control_state_msg.twist.covariance[4] = EKF_state_[3]; // psi
                control_state_msg.twist.covariance[5] = EKF_state_[4]; // v
                control_state_msg.twist.covariance[6] = EKF_state_[6]; // k
                control_state_msg.twist.covariance[8] = EKF_state_[8]; // roll
                control_state_msg.twist.covariance[9] = EKF_state_[9]; // roll rate
                control_state_msg.twist.covariance[10] = EKF_state_[10]; // pitch
                control_state_msg.twist.covariance[11] = EKF_state_[11]; // pitch rate

                control_state_msg.twist.covariance[12] = nmpc->x[17]; // velocity Low pass state of the controller
                control_state_msg.twist.covariance[13] = nmpc->x[19];// curvature Low pass state of the controller
                control_state_msg.twist.covariance[14] = elapsed;// 
                control_state_msg.twist.covariance[15] = execution_time2;// 
                control_state_pub_.publish(control_state_msg);

                // Publish the Accel message
                cmd_pub_.publish(cmd_msg);

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

                // if (atvServiceClient_.call(srv)) {
                //     //srv.response;
                // } else {
                //     std::cout << "error in service call (VIATOC NODE)" << std::endl;
                //     ROS_ERROR("Failed to call ATV service from VIATOC node");
                // }

                new_sensor_data_available_ = false;
            }
            ros::spinOnce();
        }
    }

private:
    ros::Subscriber EKF_state_estimate_sub_; // EKF subscriber
    ros::Publisher cmd_pub_; // Controlcommand publisher
    ros::Publisher control_state_pub_; // control state publisher publish second state
    ros::ServiceClient atvServiceClient_;
    int test_muuttuja2;

    std::vector<double> u_ref_ = {0.0, 0.0}; 
    // state for kinematic + 3 dof model
    // for x y path tracking is used and for velocity refrence
    // [x, y, psi, v, v_lowpass, K, K_lowpass, S, z, z_dot, roll, roll_dot, pitch, pitch_dot]

    // For old NMPC: [x, y, psi, v, v_lowpass, K, K_lowpass, S, z, z_dot, roll, roll_dot, pitch, pitch_dot]
    //std::vector<double> x_ref_ = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // For new NMPC: [x, y, z, psi, u, u_lowpass, K, K_lowpass, w, roll, p, pitch, q]
    //std::vector<double> x_ref_ = {0.0, 0.0, 0.0, 0.0, 1.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // For NMPC that does not have w: [x, y, z, psi, u, u_lowpass, K, K_lowpass, roll, p, pitch, q]
    std::vector<double> x_ref_ = {0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    //NMPCProblem* nmpc;
    std::shared_ptr<NMPCProblem> nmpc;
    //std::shared_ptr<NMPCProblem> nmpc;
    bool new_sensor_data_available_;
    bool is_initialization_done_;
    std::vector<double> EKF_state_;
    std::vector<double> dcmd_; // Derivative of control commands
    std::vector<double> cmd_; // actual control commands that are send 
    std::vector<double> lla0_;
    void initializeMPC(std::vector<double> EKF_state_) {
        nmpc->setMap(lla0_); // 

        // Set initial control trajectory
        for (int i = 0; i < nmpc->numControls * nmpc->numSteps; i++) {
            nmpc->u[i] = 0.0;
        }

        // Set initial state trajectory
        for (int j = 0; j < nmpc->numSteps; j++) {
            for (int i = 0; i < nmpc->numStates; i++) {
                nmpc->x[j * nmpc->numStates + i] = EKF_state_[i % x_ref_.size()]; 
            }
        }
    }

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "viatoc_node");
    MPCControllerNode mpc_controller;

    mpc_controller.runMPCControl();

    return 0;
}

