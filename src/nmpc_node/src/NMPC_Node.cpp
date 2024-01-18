//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// NMPC_Node.cpp
//
// Code generation for function 'NMPC_Node'
//

// Include files
#include "NMPC_Node.h"
#include "NMPC_Node_data.h"
#include "NMPC_Node_initialize.h"
#include "NMPC_Node_internal_types.h"
#include "NMPC_Node_internal_types1.h"
#include "NMPC_Node_internal_types2.h"
#include "NMPC_Node_types.h"
#include "Publisher.h"
#include "Rate.h"
#include "ServiceClient.h"
#include "Subscriber.h"
#include "anonymous_function.h"
#include "atv_can_DriveServiceRequestStruct.h"
#include "custom_cost_dynamic.h"
#include "fmincon.h"
#include "rt_nonfinite.h"
#include "tic.h"
#include "toc.h"
#include "znlmpc_getUBounds.h"
#include "coder_array.h"
#include "coder_posix_time.h"
#include "mlroscpp_pub.h"
#include "mlroscpp_rate.h"
#include "rtGetInf.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

// Function Definitions
void NMPC_Node()
{
  static const double varargin_26[14]{-0.25, -0.25, -0.25, -0.25, -0.25,
                                      -0.25, -0.25, -0.5,  -0.5,  -0.5,
                                      -0.5,  -0.5,  -0.5,  -0.5};
  static const double varargin_28[14]{0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25,
                                      0.5,  0.5,  0.5,  0.5,  0.5,  0.5,  0.5};
  static double varargin_22[98]{
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  static double varargin_18[21]{0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                                0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                                -3.0, -3.0, -3.0, -3.0, -3.0, -3.0, -3.0};
  static double varargin_20[21]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0};
  static const signed char varargin_10[21]{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                                           1, 1, 1, 0, 0, 0, 0, 0, 0, 0};
  static const signed char b_iv[7]{2, 3, 4, 5, 6, 7, 7};
  coder::anonymous_function ConFcn;
  coder::anonymous_function CostFcn;
  coder::ros::Publisher NMPC_out;
  coder::ros::Rate rr;
  coder::ros::ServiceClient atv_srv_client;
  coder::ros::Subscriber EKF_sub;
  coder::ros::b_Publisher RefPoint_out;
  coder::array<double, 2U> A;
  coder::array<double, 1U> B;
  atv_can_DriveServiceRequestStruct_T expl_temp;
  geometry_msgs_AccelStruct_T NMPC_msg;
  geometry_msgs_AccelStruct_T RefPoint_msg;
  geometry_msgs_TwistWithCovarianceStruct_T EKF_msg_Twist;
  m_struct_T Out;
  double c_varargin_22[113];
  double z[113];
  double z0[113];
  double X0[112];
  double b_varargin_22[98];
  double U0[16];
  double Umv[16];
  double c_a[14];
  double a;
  double b_a;
  double d;
  double f2;
  int Umv_tmp;
  int i;
  if (!isInitialized_NMPC_Node) {
    NMPC_Node_initialize();
  }
  varargin_18[0U] = rtGetMinusInf();
  varargin_18[1U] = rtGetMinusInf();
  varargin_18[2U] = rtGetMinusInf();
  varargin_18[3U] = rtGetMinusInf();
  varargin_18[4U] = rtGetMinusInf();
  varargin_18[5U] = rtGetMinusInf();
  varargin_18[6U] = rtGetMinusInf();
  varargin_18[7U] = rtGetMinusInf();
  varargin_18[8U] = rtGetMinusInf();
  varargin_18[9U] = rtGetMinusInf();
  varargin_18[10U] = rtGetMinusInf();
  varargin_18[11U] = rtGetMinusInf();
  varargin_18[12U] = rtGetMinusInf();
  varargin_18[13U] = rtGetMinusInf();
  varargin_20[0U] = rtGetInf();
  varargin_20[1U] = rtGetInf();
  varargin_20[2U] = rtGetInf();
  varargin_20[3U] = rtGetInf();
  varargin_20[4U] = rtGetInf();
  varargin_20[5U] = rtGetInf();
  varargin_20[6U] = rtGetInf();
  varargin_20[7U] = rtGetInf();
  varargin_20[8U] = rtGetInf();
  varargin_20[9U] = rtGetInf();
  varargin_20[10U] = rtGetInf();
  varargin_20[11U] = rtGetInf();
  varargin_20[12U] = rtGetInf();
  varargin_20[13U] = rtGetInf();
  varargin_22[0U] = rtGetMinusInf();
  varargin_22[1U] = rtGetMinusInf();
  varargin_22[2U] = rtGetMinusInf();
  varargin_22[3U] = rtGetMinusInf();
  varargin_22[4U] = rtGetMinusInf();
  varargin_22[5U] = rtGetMinusInf();
  varargin_22[6U] = rtGetMinusInf();
  varargin_22[7U] = rtGetMinusInf();
  varargin_22[8U] = rtGetMinusInf();
  varargin_22[9U] = rtGetMinusInf();
  varargin_22[10U] = rtGetMinusInf();
  varargin_22[11U] = rtGetMinusInf();
  varargin_22[12U] = rtGetMinusInf();
  varargin_22[13U] = rtGetMinusInf();
  varargin_22[14U] = rtGetMinusInf();
  varargin_22[15U] = rtGetMinusInf();
  varargin_22[16U] = rtGetMinusInf();
  varargin_22[17U] = rtGetMinusInf();
  varargin_22[18U] = rtGetMinusInf();
  varargin_22[19U] = rtGetMinusInf();
  varargin_22[20U] = rtGetMinusInf();
  varargin_22[28U] = rtGetMinusInf();
  varargin_22[29U] = rtGetMinusInf();
  varargin_22[30U] = rtGetMinusInf();
  varargin_22[31U] = rtGetMinusInf();
  varargin_22[32U] = rtGetMinusInf();
  varargin_22[33U] = rtGetMinusInf();
  varargin_22[34U] = rtGetMinusInf();
  varargin_22[35U] = rtGetMinusInf();
  varargin_22[36U] = rtGetMinusInf();
  varargin_22[37U] = rtGetMinusInf();
  varargin_22[38U] = rtGetMinusInf();
  varargin_22[39U] = rtGetMinusInf();
  varargin_22[40U] = rtGetMinusInf();
  varargin_22[41U] = rtGetMinusInf();
  varargin_22[42U] = rtGetMinusInf();
  varargin_22[43U] = rtGetMinusInf();
  varargin_22[44U] = rtGetMinusInf();
  varargin_22[45U] = rtGetMinusInf();
  varargin_22[46U] = rtGetMinusInf();
  varargin_22[47U] = rtGetMinusInf();
  varargin_22[48U] = rtGetMinusInf();
  varargin_22[49U] = rtGetMinusInf();
  varargin_22[50U] = rtGetMinusInf();
  varargin_22[51U] = rtGetMinusInf();
  varargin_22[52U] = rtGetMinusInf();
  varargin_22[53U] = rtGetMinusInf();
  varargin_22[54U] = rtGetMinusInf();
  varargin_22[55U] = rtGetMinusInf();
  varargin_22[56U] = rtGetMinusInf();
  varargin_22[57U] = rtGetMinusInf();
  varargin_22[58U] = rtGetMinusInf();
  varargin_22[59U] = rtGetMinusInf();
  varargin_22[60U] = rtGetMinusInf();
  varargin_22[61U] = rtGetMinusInf();
  varargin_22[62U] = rtGetMinusInf();
  varargin_22[63U] = rtGetMinusInf();
  varargin_22[64U] = rtGetMinusInf();
  varargin_22[65U] = rtGetMinusInf();
  varargin_22[66U] = rtGetMinusInf();
  varargin_22[67U] = rtGetMinusInf();
  varargin_22[68U] = rtGetMinusInf();
  varargin_22[69U] = rtGetMinusInf();
  varargin_22[70U] = rtGetMinusInf();
  varargin_22[71U] = rtGetMinusInf();
  varargin_22[72U] = rtGetMinusInf();
  varargin_22[73U] = rtGetMinusInf();
  varargin_22[74U] = rtGetMinusInf();
  varargin_22[75U] = rtGetMinusInf();
  varargin_22[76U] = rtGetMinusInf();
  varargin_22[77U] = rtGetMinusInf();
  varargin_22[78U] = rtGetMinusInf();
  varargin_22[79U] = rtGetMinusInf();
  varargin_22[80U] = rtGetMinusInf();
  varargin_22[81U] = rtGetMinusInf();
  varargin_22[82U] = rtGetMinusInf();
  varargin_22[83U] = rtGetMinusInf();
  varargin_22[84U] = rtGetMinusInf();
  varargin_22[85U] = rtGetMinusInf();
  varargin_22[86U] = rtGetMinusInf();
  varargin_22[87U] = rtGetMinusInf();
  varargin_22[88U] = rtGetMinusInf();
  varargin_22[89U] = rtGetMinusInf();
  varargin_22[90U] = rtGetMinusInf();
  varargin_22[91U] = rtGetMinusInf();
  varargin_22[92U] = rtGetMinusInf();
  varargin_22[93U] = rtGetMinusInf();
  varargin_22[94U] = rtGetMinusInf();
  varargin_22[95U] = rtGetMinusInf();
  varargin_22[96U] = rtGetMinusInf();
  varargin_22[97U] = rtGetMinusInf();
  //     %{
  //     global Ca
  //     global pFR pFL pRR pRL
  //     global l tw h h_T
  //     global mFL mFR mRL mRR m
  //     global kFL kFR kRL kRR
  //     global cFL cFR cRR cRL
  //     global Ixx Iyy Izz xoff yoff zoff
  //     global g D2R R2D
  //     global str_a str_b
  //     global Cr acc_a acc_b acc_c
  //     global VEC_OFF
  //     global roll_bias pitch_bias
  //     global lla0
  //     global R_t Q_t Pkm1 xkm1
  //     global nd neq np ns ny na nu dt2
  //     global D_Epec D_Novatel
  //     % persistent myFilter
  //
  //     % global variables to be used in while(1)
  //     global V_cmd acmd acc V_ODOM
  //     global stg stg_cmd curv Kcmd
  //     global pos_enu_m pos_enu_gnss
  //     global pos_gnss_lla
  //     global phi_m theta_m psi_m
  //     global dotE_m dotN_m dotU_m
  //     global delta_x_FR delta_x_FL delta_x_RR delta_x_RL;
  //     global TGC
  //     global ref_test Index
  //     global X_ekf dt
  //     global K_ref
  //
  //     % rotation matrix around X-axis and its derivatives
  //     Rx = @(roll)[1 0 0;0 cos(roll) -sin(roll); 0 sin(roll) cos(roll)];
  //     % Rotation matrix around Y-axis and its derivatives
  //     Ry = @(pitch)[cos(pitch) 0 sin(pitch); 0 1 0;-sin(pitch) 0 cos(pitch)];
  //     % Rotation matrix around Z-axis and its derivatives
  //     Rz = @(yaw)[cos(yaw) -sin(yaw) 0;sin(yaw) cos(yaw) 0; 0 0 1];
  //
  //     % initialize the global variables
  //     delta_x_FR = 0.0;
  //     delta_x_FL = 0.0;
  //     delta_x_RR = 0.0;
  //     delta_x_RL = 0.0;
  //     phi_m = 0.0; theta_m = 0.0; psi_m = 0.0;
  //     dotE_m = 0.0; dotN_m = 0.0; dotU_m = 0.0;
  //     pos_gnss_lla = [0.0 0.0 0.0]; % fixed GCP point in Vakola
  //     stg = 0.0; curv = 0.0;
  //     stg_cmd=0.0; V_cmd= 0.0;
  //     acmd = 0.0; Kcmd = 0.0;
  //     TGC = 0.2;
  //     dt=0.1;
  //
  //     % initialize the global constants
  //     Ca = 1.0419e4; % cornering stiffness
  //     g = 9.8; % acceleration due to gravity -- m/s^2
  //     Cr = 0.0397; % rolling resistance coefficient
  //     D2R = pi/180; % degrees to radians
  //     R2D = 1/D2R; % radians to degrees
  //     % coefficient for force acceleration model
  //     acc_a = -0.515; acc_b = -acc_a; acc_c = g*Cr;
  //     % coefficient for steering dynamics model
  //     str_a = -1.427; str_b = -str_a;
  //     % mass at each corner
  //     mFL = 190.0; mFR = 190.0;
  //     mRL = 260.0; mRR = 260.0;
  //     md = 180.0; % mass of two drivers
  //     m = mFL + mFR + mRL + mRR + md;
  //     % the moment arms from tests
  //     l = 1.83; tw = 1.160; h =  2*0.8767;
  //     % tire radius
  //     h_T = 0.3175;
  //     % extended lengths of each side are used to compute inertia matrix
  //     lt = 2.74; % depth -- m
  //     wt = 1.44; % width -- m
  //     ht = 1.85; % height -- m
  //     % compute inertia matrix values by using the full lengths of polaris
  //     Ixx = (m/12)*(wt^2+ht^2); % inertia around X
  //     Iyy = (m/12)*(lt^2+ht^2); % inertia around Y
  //     Izz = (m/12)*(wt^2+lt^2); % inertia around Z
  //     % even mass on each side
  //     mFL = m/4; mFR = m/4;
  //     mRL = m/4; mRR = m/4;
  //     % spring coefficients from calibration test
  //     kFL = 1.5791e4; kFR = 1.3099e4;
  //     kRL = 1.7327e4; kRR = 1.6467e4;
  //     K_avg = (kFL+kRL+kFR+kRR)/4;
  //     kFL = K_avg; kFR = K_avg;
  //     kRL = K_avg; kRR = K_avg;
  //     % define offsets of CG from VC
  //     xoff = 0.0; % front side is lighter than rear
  //     yoff = 0.0; % load is evenly distributed to left and right sides
  //     zoff = 0.0; % below H_VC
  //     zeta = 2; % high damping with high settling time -- TBD
  //     cFL = 2*zeta*sqrt(kFL*mFL); cFR = 2*zeta*sqrt(kFR*mFR);
  //     cRL = 2*zeta*sqrt(kRL*mRL); cRR = 2*zeta*sqrt(kRR*mRR);
  //     % initialize the moment arms here, so the same should be used in
  //     dynamic % model and for data post-processing. pFR = [ l/2; -tw/2; -h/2]
  //     - [xoff; yoff; zoff]; pRR = [-l/2; -tw/2; -h/2] - [xoff; yoff; zoff];
  //     pFL = [ l/2;  tw/2; -h/2] - [xoff; yoff; zoff];
  //     pRL = [-l/2;  tw/2; -h/2] - [xoff; yoff; zoff];
  //
  //     % Turn Reference to Steering Command converion coefficients
  //     K_ref=[-0.000994869050617; 32.649869050616971];
  //
  //     %create NMPC object
  //     nx = 14;
  //     ny = 5;
  //     nu = 2;
  //     nlobj = nlmpc(nx,ny,nu);
  //
  //     %set sampling time, prediction- and control horizon
  //     Ts=0.1;
  //     nlobj.Ts = Ts;
  //     nlobj.PredictionHorizon = 7;
  //     nlobj.ControlHorizon = nlobj.PredictionHorizon; %same because goal is
  //     not to stabilize, reference is changing
  //
  //     %define state space model
  //     nlobj.Model.StateFcn = "dynamic_model";
  //     nlobj.Model.IsContinuousTime = true;
  //     nlobj.Model.NumberOfParameters = 1;
  //
  //
  //     %define state constraints
  //     nlobj.States(4).Min=0;
  //
  //     %nlobj.States(7).Min = deg2rad(-40); %roll angle limit
  //     %nlobj.States(7).Max = deg2rad(40);
  //
  //     %define input constraints
  //     nlobj.ManipulatedVariables(1).Min=-0.2625; % curvature
  //     nlobj.ManipulatedVariables(1).Max=0.2625;
  //
  //     %nlobj.ManipulatedVariables(1).RateMin=-10;%-5.25; % rate of curvature
  //     %nlobj.ManipulatedVariables(1).RateMax=10;%5.25;
  //
  //     nlobj.ManipulatedVariables(2).Min=-0.5; %acceleration
  //     nlobj.ManipulatedVariables(2).Max=0.5;
  //
  //     %nlobj.ManipulatedVariables(2).RateMin=-50;%-5.25; % rate of curvature
  //     %nlobj.ManipulatedVariables(2).RateMax=50;%5.25;
  //
  //     %define custom ouput fnc
  //     nlobj.Model.OutputFcn='model_output_fnc';
  //
  //     %define output constraints
  //     %nlobj.OV(3).Min = -0.9; %LTR limits
  //     %nlobj.OV(3).Max = 0.9;
  //
  //     nlobj.OV(4).Min = 0.1; %ground speed limits
  //     nlobj.OV(4).Max = 4;
  //
  //     %nlobj.OV(5).Min=-3; % lateral acceleration
  //     %nlobj.OV(5).Max=3;
  //
  //     nlobj.Optimization.CustomCostFcn = 'custom_cost_dynamic';
  //     nlobj.Optimization.ReplaceStandardCost=true;
  //     %}
  // Create code generation data structures for the controller
  //  initialize subscriber
  EKF_sub.init();
  EKF_sub.receive(EKF_msg_Twist);
  NMPC_out.init();
  coder::ros::Publisher::rosmessage(NMPC_msg);
  RefPoint_out.init();
  coder::ros::Publisher::rosmessage(RefPoint_msg);
  atv_srv_client.init();
  expl_temp = atv_can_DriveServiceRequestStruct();
  atv_srv_client.waitForServer();
  //     %{
  //     X0=[...
  //     EKF_msg.Pose.Pose.Position.X;
  //     EKF_msg.Pose.Pose.Position.Y;
  //     EKF_msg.Pose.Pose.Position.Z;
  //     EKF_msg.Twist.Twist.Linear.X;
  //     EKF_msg.Twist.Twist.Linear.Y;
  //     EKF_msg.Twist.Twist.Linear.Z;
  //     EKF_msg.Twist.Covariance(5);% roll
  //     EKF_msg.Twist.Covariance(6);% pitch
  //     EKF_msg.Twist.Covariance(7);% heading
  //     EKF_msg.Twist.Twist.Angular.X;% p -- roll rate
  //     EKF_msg.Twist.Twist.Angular.Y; % q -- pitch rate
  //     EKF_msg.Twist.Twist.Angular.Z; % r -- yaw rate
  //     EKF_msg.Twist.Covariance(8); %curvaure
  //     EKF_msg.Twist.Covariance(1); % TGC
  //     ]; % put the TGC here
  //     %}
  std::printf("Initial Position: %f, %f, %f.\n", EKF_msg_Twist.Covariance[1],
              EKF_msg_Twist.Covariance[2], EKF_msg_Twist.Covariance[3]);
  std::fflush(stdout);
  // GS=CMD_msg.Linear.Z;
  // U=[0;0];
  // Ts=0.1;
  // params = {Ts};
  // [coreData,onlineData] = getCodeGenerationData(nlobj,X0,U,params);
  //     %{
  //     ref_track=load("Track06_POS.mat");
  //     ref_track_lla=ref_track.P_GNSS_LLA;
  //     ref_track_enu=lla2enu(ref_track_lla, lla0, 'flat');
  //     ref_test=[ref_track_enu(:,1) ref_track_enu(:,2)];
  //     %}
  // ref_test=get_test_track();
  //  Create publisher
  // pub =
  // rospublisher("/control_cmd","input_msgs/ControlCommand","DataFormat","struct");
  // NMPC_pub =
  // rospublisher("/NMPC_out",'geometry_msgs/Point','Dataformat','struct');
  // CMD_Client = rossvcclient("/can_service");
  //  Create a message
  // msg_send = rosmessage("geometry_msgs/Twist","DataFormat","struct");
  // msg_send.ControlCommand{1} = 'u1';
  // msg_send.ControlCommand{2} = 'u2';
  CostFcn.workspace.runtimedata.lastMV[0] = 0.0;
  CostFcn.workspace.runtimedata.lastMV[1] = 0.0;
  // Index=1;
  rr.init();
  a = X_ekf[0] - ref_test[0];
  b_a = X_ekf[1] - ref_test[22000];
  CostFcn.workspace.runtimedata.Parameters[0] = onlineData.Parameters[0];
  CostFcn.workspace.runtimedata.ECRWeight = 100000.0;
  CostFcn.workspace.userdata.Ts = 0.1;
  CostFcn.workspace.userdata.PredictionHorizon = 7.0;
  CostFcn.workspace.userdata.NumOfStates = 14.0;
  CostFcn.workspace.userdata.NumOfOutputs = 3.0;
  CostFcn.workspace.userdata.NumOfInputs = 2.0;
  CostFcn.workspace.userdata.InputPassivityIndex = 0.0;
  CostFcn.workspace.userdata.OutputPassivityIndex = 0.1;
  CostFcn.workspace.userdata.PassivityUsePredictedX = true;
  ConFcn.workspace.runtimedata.ECRWeight = 100000.0;
  ConFcn.workspace.runtimedata.Parameters[0] = onlineData.Parameters[0];
  ConFcn.workspace.userdata.Ts = 0.1;
  ConFcn.workspace.userdata.PredictionHorizon = 7.0;
  ConFcn.workspace.userdata.NumOfStates = 14.0;
  ConFcn.workspace.userdata.NumOfOutputs = 3.0;
  ConFcn.workspace.userdata.NumOfInputs = 2.0;
  ConFcn.workspace.userdata.InputPassivityIndex = 0.0;
  ConFcn.workspace.userdata.OutputPassivityIndex = 0.1;
  ConFcn.workspace.userdata.PassivityUsePredictedX = true;
  NMPC_msg.Linear.Z = X_ekf[3];
  NMPC_msg.Angular.Z = X_ekf[12];
  RefPoint_msg.Linear.Z = 0.0;
  for (i = 0; i < 2; i++) {
    CostFcn.workspace.runtimedata.MVScaledTarget[7 * i] =
        onlineData.MVTarget[i];
    for (Umv_tmp = 0; Umv_tmp < 6; Umv_tmp++) {
      CostFcn.workspace.runtimedata.MVScaledTarget[(Umv_tmp + 7 * i) + 1] =
          onlineData.MVTarget[i];
    }
  }
  for (i = 0; i < 14; i++) {
    CostFcn.workspace.runtimedata.MVRateWeights[i] = 0.1;
  }
  for (i = 0; i < 98; i++) {
    CostFcn.workspace.runtimedata.StateMax[i] = rtInf;
  }
  for (int b_i{0}; b_i < 14; b_i++) {
    CostFcn.workspace.runtimedata.MVRateMin[b_i] = rtMinusInf;
    CostFcn.workspace.runtimedata.MVRateMax[b_i] = rtInf;
    CostFcn.workspace.runtimedata.x[b_i] = X_ekf[b_i];
  }
  for (i = 0; i < 21; i++) {
    CostFcn.workspace.runtimedata.OutputWeights[i] = varargin_10[i];
  }
  std::memset(&CostFcn.workspace.runtimedata.MVWeights[0], 0,
              14U * sizeof(double));
  std::copy(&varargin_18[0], &varargin_18[21],
            &CostFcn.workspace.runtimedata.OutputMin[0]);
  std::copy(&varargin_20[0], &varargin_20[21],
            &CostFcn.workspace.runtimedata.OutputMax[0]);
  std::copy(&varargin_22[0], &varargin_22[98],
            &CostFcn.workspace.runtimedata.StateMin[0]);
  CostFcn.workspace.userdata.MVIndex[0] = 1.0;
  CostFcn.workspace.userdata.MVIndex[1] = 2.0;
  for (int b_i{0}; b_i < 14; b_i++) {
    CostFcn.workspace.runtimedata.MVMin[b_i] = varargin_26[b_i];
    CostFcn.workspace.runtimedata.MVMax[b_i] = varargin_28[b_i];
    d = X_ekf[b_i];
    CostFcn.workspace.userdata.CurrentStates[b_i] = d;
    CostFcn.workspace.userdata.MVTarget[b_i] =
        CostFcn.workspace.runtimedata.MVScaledTarget[b_i];
    ConFcn.workspace.runtimedata.x[b_i] = d;
    ConFcn.workspace.runtimedata.MVWeights[b_i] = 0.0;
    ConFcn.workspace.runtimedata.MVRateWeights[b_i] = 0.1;
  }
  std::copy(&varargin_18[0], &varargin_18[21],
            &ConFcn.workspace.runtimedata.OutputMin[0]);
  std::copy(&varargin_20[0], &varargin_20[21],
            &ConFcn.workspace.runtimedata.OutputMax[0]);
  for (i = 0; i < 98; i++) {
    ConFcn.workspace.runtimedata.StateMin[i] = varargin_22[i];
    ConFcn.workspace.runtimedata.StateMax[i] = rtInf;
  }
  for (int b_i{0}; b_i < 14; b_i++) {
    ConFcn.workspace.runtimedata.MVMin[b_i] = varargin_26[b_i];
    ConFcn.workspace.runtimedata.MVMax[b_i] = varargin_28[b_i];
    ConFcn.workspace.runtimedata.MVRateMin[b_i] = rtMinusInf;
    ConFcn.workspace.runtimedata.MVRateMax[b_i] = rtInf;
    d = CostFcn.workspace.runtimedata.MVScaledTarget[b_i];
    ConFcn.workspace.runtimedata.MVScaledTarget[b_i] = d;
    ConFcn.workspace.userdata.CurrentStates[b_i] = X_ekf[b_i];
    ConFcn.workspace.userdata.MVTarget[b_i] = d;
  }
  ConFcn.workspace.userdata.MVIndex[0] = 1.0;
  ConFcn.workspace.userdata.MVIndex[1] = 2.0;
  onlineData.ref[2] = 0.0;
  while (1) {
    double zUB[113];
    double dv[14];
    double Kcmd;
    double acmd;
    double b_min;
    double d1;
    double dist;
    // X=Xc(1); Y=Xc(2);
    // [X, Y]=ll2utm(lat,long);
    b_min = std::sqrt(a * a + b_a * b_a);
    acmd = Index;
    i = static_cast<int>((Index + 1000.0) + (1.0 - Index));
    for (int b_i{0}; b_i < i; b_i++) {
      Kcmd = Index + static_cast<double>(b_i);
      // length(path)
      dist = X_ekf[0] - ref_test[static_cast<int>(Kcmd) - 1];
      f2 = X_ekf[1] - ref_test[static_cast<int>(Kcmd) + 21999];
      dist = std::sqrt(dist * dist + f2 * f2);
      if (dist <= b_min) {
        b_min = dist;
        acmd = Kcmd;
      }
    }
    //  get the closest point from the referene path
    Index = acmd;
    d = ref_test[static_cast<int>(acmd) - 1];
    onlineData.ref[0] = d;
    d1 = ref_test[static_cast<int>(acmd) + 21999];
    onlineData.ref[1] = d1;
    // ref_lla = enu2lla([ref 0], lla0, 'flat');
    for (i = 0; i < 3; i++) {
      CostFcn.workspace.runtimedata.ref[7 * i] = onlineData.ref[i];
      for (Umv_tmp = 0; Umv_tmp < 6; Umv_tmp++) {
        CostFcn.workspace.runtimedata.ref[(Umv_tmp + 7 * i) + 1] =
            onlineData.ref[i];
      }
    }
    for (i = 0; i < 7; i++) {
      for (Umv_tmp = 0; Umv_tmp < 14; Umv_tmp++) {
        b_varargin_22[Umv_tmp + 14 * i] = onlineData.X0[i + 7 * Umv_tmp];
      }
      Umv_tmp = i << 1;
      dv[Umv_tmp] = onlineData.MV0[i];
      dv[Umv_tmp + 1] = onlineData.MV0[i + 7];
    }
    for (i = 0; i < 14; i++) {
      b_min = 0.0;
      for (Umv_tmp = 0; Umv_tmp < 14; Umv_tmp++) {
        b_min += static_cast<double>(iv[i + 14 * Umv_tmp]) * dv[Umv_tmp];
      }
      c_a[i] = b_min;
    }
    std::copy(&b_varargin_22[0], &b_varargin_22[98], &z0[0]);
    std::copy(&c_a[0], &c_a[14], &z0[98]);
    z0[112] = onlineData.Slack0;
    for (i = 0; i < 98; i++) {
      zUB[i] = rtInf;
    }
    for (i = 0; i < 14; i++) {
      zUB[i + 98] = rtInf;
    }
    zUB[112] = rtInf;
    std::memset(&X0[0], 0, 112U * sizeof(double));
    std::memset(&Umv[0], 0, 16U * sizeof(double));
    for (i = 0; i < 14; i++) {
      b_min = 0.0;
      for (Umv_tmp = 0; Umv_tmp < 14; Umv_tmp++) {
        b_min += static_cast<double>(iv[i + 14 * Umv_tmp]) * z0[Umv_tmp + 98];
      }
      c_a[i] = b_min;
    }
    for (i = 0; i < 2; i++) {
      for (Umv_tmp = 0; Umv_tmp < 7; Umv_tmp++) {
        Umv[Umv_tmp + (i << 3)] = c_a[i + (Umv_tmp << 1)];
      }
    }
    std::copy(&z0[0], &z0[98], &b_varargin_22[0]);
    for (i = 0; i < 14; i++) {
      for (Umv_tmp = 0; Umv_tmp < 7; Umv_tmp++) {
        X0[(Umv_tmp + (i << 3)) + 1] = b_varargin_22[i + 14 * Umv_tmp];
      }
      X0[i << 3] = X_ekf[i];
    }
    for (int b_i{0}; b_i < 2; b_i++) {
      Umv_tmp = b_i << 3;
      Umv[Umv_tmp + 7] = Umv[Umv_tmp + 6];
      std::copy(&Umv[Umv_tmp],
                &Umv[static_cast<int>(static_cast<unsigned int>(Umv_tmp) + 8U)],
                &U0[Umv_tmp]);
    }
    dist = custom_cost_dynamic(X0, U0, CostFcn.workspace.runtimedata.ref);
    f2 = custom_cost_dynamic(X0, U0, CostFcn.workspace.runtimedata.ref);
    if (f2 <= dist) {
      zUB[112] = 0.0;
    }
    coder::znlmpc_getUBounds(CostFcn.workspace.runtimedata.lastMV, varargin_26,
                             varargin_28,
                             CostFcn.workspace.runtimedata.MVRateMin,
                             CostFcn.workspace.runtimedata.MVRateMax, A, B);
    CostFcn.workspace.userdata.LastMV[0] =
        CostFcn.workspace.runtimedata.lastMV[0];
    CostFcn.workspace.userdata.LastMV[1] =
        CostFcn.workspace.runtimedata.lastMV[1];
    ConFcn.workspace.runtimedata.lastMV[0] =
        CostFcn.workspace.runtimedata.lastMV[0];
    ConFcn.workspace.runtimedata.lastMV[1] =
        CostFcn.workspace.runtimedata.lastMV[1];
    ConFcn.workspace.userdata.LastMV[0] =
        CostFcn.workspace.runtimedata.lastMV[0];
    ConFcn.workspace.userdata.LastMV[1] =
        CostFcn.workspace.runtimedata.lastMV[1];
    for (i = 0; i < 21; i++) {
      b_min = CostFcn.workspace.runtimedata.ref[i];
      CostFcn.workspace.userdata.References[i] = b_min;
      ConFcn.workspace.runtimedata.ref[i] = b_min;
      ConFcn.workspace.runtimedata.OutputWeights[i] = varargin_10[i];
      ConFcn.workspace.userdata.References[i] = b_min;
    }
    for (i = 0; i < 7; i++) {
      for (Umv_tmp = 0; Umv_tmp < 14; Umv_tmp++) {
        b_varargin_22[Umv_tmp + 14 * i] = varargin_22[i + 7 * Umv_tmp];
      }
    }
    std::copy(&b_varargin_22[0], &b_varargin_22[98], &c_varargin_22[0]);
    for (i = 0; i < 14; i++) {
      c_varargin_22[i + 98] = rtMinusInf;
    }
    c_varargin_22[112] = 0.0;
    coder::fmincon(CostFcn, z0, A, B, c_varargin_22, zUB, ConFcn, z, f2, Out);
    b_min = std::round(f2);
    if (b_min < 2.147483648E+9) {
      if (b_min >= -2.147483648E+9) {
        i = static_cast<int>(b_min);
      } else {
        i = MIN_int32_T;
      }
    } else if (b_min >= 2.147483648E+9) {
      i = MAX_int32_T;
    } else {
      i = 0;
    }
    if ((i == 0) && (Out.constrviolation > 1.0E-6)) {
      f2 = -2.0;
    }
    std::memset(&X0[0], 0, 112U * sizeof(double));
    std::memset(&Umv[0], 0, 16U * sizeof(double));
    for (i = 0; i < 14; i++) {
      b_min = 0.0;
      for (Umv_tmp = 0; Umv_tmp < 14; Umv_tmp++) {
        b_min += static_cast<double>(iv[i + 14 * Umv_tmp]) * z[Umv_tmp + 98];
      }
      c_a[i] = b_min;
    }
    for (i = 0; i < 2; i++) {
      for (Umv_tmp = 0; Umv_tmp < 7; Umv_tmp++) {
        Umv[Umv_tmp + (i << 3)] = c_a[i + (Umv_tmp << 1)];
      }
    }
    std::copy(&z[0], &z[98], &b_varargin_22[0]);
    for (i = 0; i < 14; i++) {
      for (Umv_tmp = 0; Umv_tmp < 7; Umv_tmp++) {
        X0[(Umv_tmp + (i << 3)) + 1] = b_varargin_22[i + 14 * Umv_tmp];
      }
      X0[i << 3] = X_ekf[i];
    }
    for (int b_i{0}; b_i < 2; b_i++) {
      Umv_tmp = b_i << 3;
      Umv[Umv_tmp + 7] = Umv[Umv_tmp + 6];
      std::copy(&Umv[Umv_tmp],
                &Umv[static_cast<int>(static_cast<unsigned int>(Umv_tmp) + 8U)],
                &U0[Umv_tmp]);
    }
    if (f2 > 0.0) {
      CostFcn.workspace.runtimedata.lastMV[0] = U0[0];
      CostFcn.workspace.runtimedata.lastMV[1] = U0[8];
    }
    for (i = 0; i < 2; i++) {
      for (Umv_tmp = 0; Umv_tmp < 7; Umv_tmp++) {
        onlineData.MV0[Umv_tmp + 7 * i] = U0[(Umv_tmp + (i << 3)) + 1];
      }
    }
    for (i = 0; i < 14; i++) {
      for (Umv_tmp = 0; Umv_tmp < 7; Umv_tmp++) {
        onlineData.X0[Umv_tmp + 7 * i] = X0[b_iv[Umv_tmp] + (i << 3)];
      }
    }
    onlineData.Slack0 = std::fmax(0.0, z[112]);
    if (CostFcn.workspace.runtimedata.lastMV[0] < -0.25) {
      Kcmd = -0.25;
    } else if (CostFcn.workspace.runtimedata.lastMV[0] > 0.25) {
      Kcmd = 0.25;
    } else {
      Kcmd = CostFcn.workspace.runtimedata.lastMV[0];
    }
    if (CostFcn.workspace.runtimedata.lastMV[1] < -0.5) {
      acmd = -0.5;
    } else if (CostFcn.workspace.runtimedata.lastMV[1] > 0.5) {
      acmd = 0.5;
    } else {
      acmd = CostFcn.workspace.runtimedata.lastMV[1];
    }
    dist = X_ekf[3] + dt * acmd;
    if (dist < 0.0) {
      dist = 0.0;
    } else if (dist > 2.0) {
      dist = 2.0;
    }
    //  m/s
    //  assign values to driver service request message
    //  mm/s
    f2 = 57.295779513082323 * std::atan(1.83 * Kcmd);
    // int16(0.0);% Motor_Value
    // uint16(32768);% Turn_Ref
    //  true for torque, false for speed
    if (atv_srv_client.b_isServerAvailable()) {
      short i1;
      unsigned short u;
      b_min = std::round(dist * 1000.0);
      if (b_min < 32768.0) {
        if (b_min >= -32768.0) {
          i1 = static_cast<short>(b_min);
        } else {
          i1 = MIN_int16_T;
        }
      } else if (b_min >= 32768.0) {
        i1 = MAX_int16_T;
      } else {
        i1 = 0;
      }
      b_min = std::round((f2 - 32.649869050616971) / -0.000994869050617);
      if (b_min < 65536.0) {
        if (b_min >= 0.0) {
          u = static_cast<unsigned short>(b_min);
        } else {
          u = 0U;
        }
      } else if (b_min >= 65536.0) {
        u = MAX_uint16_T;
      } else {
        u = 0U;
      }
      atv_srv_client.call(expl_temp.MessageType, i1, u);
      //  put the steering/cruvature commands in angular part and acceleration
      //  command in the linear part of the geomtery_msgs/Accel type message
    }
    //  assign data to the NMPC publisher
    NMPC_msg.Linear.X = acmd;
    NMPC_msg.Linear.Y = dist;
    //  measured speed from state vector
    NMPC_msg.Angular.X = f2;
    NMPC_msg.Angular.Y = Kcmd;
    //  curvature from state
    RefPoint_msg.Linear.X = d;
    RefPoint_msg.Linear.Y = d1;
    RefPoint_msg.Angular.X = CostFcn.workspace.runtimedata.lastMV[0];
    RefPoint_msg.Angular.Y = CostFcn.workspace.runtimedata.lastMV[1];
    RefPoint_msg.Angular.Z = Index;
    MATLABPUBLISHER_publish(NMPC_out.PublisherHelper, &NMPC_msg);
    MATLABPUBLISHER_publish(RefPoint_out.PublisherHelper, &RefPoint_msg);
    MATLABRate_sleep(rr.RateHelper);
    coder::toc(rr.PreviousPeriod.tv_sec, rr.PreviousPeriod.tv_nsec);
    rr.PreviousPeriod.tv_sec = coder::tic(rr.PreviousPeriod.tv_nsec);
  }
}

// End of code generation (NMPC_Node.cpp)
