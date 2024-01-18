%% Extract, Decode and Plot data from GASP, WODO, and SODO Messages.
% GASP -- EPEC gas pedal measurements
bGASP = select(bag,'Topic','/can/atv_dash_gas_pedal_measurement');
TimeGASP = bGASP.MessageList.Time;
msgGASP = readMessages(bGASP,'DataFormat','struct');

TV_GASP = zeros(length(msgGASP),1);
for k = 1 : length(msgGASP)
    % Following time conversion is verified by toSec() function in ROS.
    TV_GASP(k) = double ( msgGASP{k,1}.TimeReceived.Sec ) + ...
        double ( msgGASP{k,1}.TimeReceived.Nsec ) * 1e-9;
end
% compute log time for /can/atv_dash_gas_pedal_measurement topic
LogTime_GASP = TimeGASP(end) - TimeGASP(1);
% compute sampling frequency from data
Fs_GASP = length(msgGASP)/LogTime_GASP;

pwm_ratio1 = zeros(length(msgGASP), 1);
pwm_ratio2 = zeros(length(msgGASP), 1);
wheel_drive_mode = zeros(length(msgGASP), 1);
gear_ratio = zeros(length(msgGASP), 1);
for k = 1 : length(msgGASP)
    pwm_ratio1(k) = msgGASP{k,1}.PwmRatio1;
    pwm_ratio2(k) = msgGASP{k,1}.PwmRatio2;    
    wheel_drive_mode(k) = msgGASP{k,1}.AllWheelDrive;
    gear_ratio(k) = msgGASP{k,1}.GearRatio;
end

% WODO -- EPEC wheel odometry measurements
bWODO = select(bag,'Topic','/can/atv_odometry_measurement');
TimeWODO = bWODO.MessageList.Time;
msgWODO = readMessages(bWODO,'DataFormat','struct');

TV_WODO = zeros(length(msgWODO),1);
for k = 1 : length(msgWODO)
    % Following time conversion is verified by toSec() function in ROS.
    TV_WODO(k) = double ( msgWODO{k,1}.TimeReceived.Sec ) + ...
        double ( msgWODO{k,1}.TimeReceived.Nsec ) * 1e-9;
end
% compute log time for /can/atv_odometry_measurement topic
LogTime_WODO = TimeWODO(end) - TimeWODO(1);
% compute sampling frequency from data
Fs_WODO = length(msgWODO)/LogTime_WODO;

speed_FL = zeros(length(msgWODO), 1);
speed_FR = zeros(length(msgWODO), 1);
speed_RL = zeros(length(msgWODO), 1);
speed_RR = zeros(length(msgWODO), 1);

for k = 1 : length(msgWODO)
    speed_FL(k) = msgWODO{k,1}.FrontLeft;
    speed_FR(k) = msgWODO{k,1}.FrontRight;
    speed_RL(k) = msgWODO{k,1}.RearLeft;
    speed_RR(k) = msgWODO{k,1}.RearRight;
end

wheel_ground_speed_rear = (-speed_RL + speed_RR)./2000.0;
wheel_ground_speed_front = (-speed_FL + speed_FR)./2000.0;
wheel_ground_speed_frontrear = 0.5*(wheel_ground_speed_rear + wheel_ground_speed_front);

wheel_ground_speed_left = (-speed_RL + -speed_FL)./2000.0;
wheel_ground_speed_right = (speed_FR + speed_FR)./2000.0;
wheel_ground_speed_leftright = 0.5*(wheel_ground_speed_left + wheel_ground_speed_right);

 % flag to choose between which definition of wheel speed to follow
choose_speed = -1; % 1 = left/right, 0 = front/rear.
switch choose_speed
    case 1
        wheel_ground_speed = wheel_ground_speed_rear;
    case 2
        wheel_ground_speed = wheel_ground_speed_front;
    case 3 
        wheel_ground_speed = wheel_ground_speed_left;
    case 4 
        wheel_ground_speed = wheel_ground_speed_right;
    otherwise
        wheel_ground_speed = wheel_ground_speed_rear;
end

% SODO -- EPEC steering measurements
bSODO = select(bag,'Topic','/can/atv_steering_measurement');
TimeSODO = bSODO.MessageList.Time;
msgSODO = readMessages(bSODO,'DataFormat','struct');

TV_SODO = zeros(length(msgSODO),1);
for k = 1 : length(msgSODO)
    % Following time conversion is verified by toSec() function in ROS.
    TV_SODO(k) = double ( msgSODO{k,1}.TimeReceived.Sec ) + ...
        double ( msgSODO{k,1}.TimeReceived.Nsec ) * 1e-9;
end
% compute log time for /can/atv_odometry_measurement topic
LogTime_SODO = TimeSODO(end) - TimeSODO(1);
% compute sampling frequency from data
Fs_SODO = length(msgSODO)/LogTime_SODO;

% steering angle encoder values -- TBD
steering_encode_position = zeros(length(msgSODO), 1);
for k = 1 : length(msgSODO)
    steering_encode_position(k) = msgSODO{k,1}.EncoderPosition;
end

% MSS -- This sentence has machine selected speed which is an output from
% SteeringandOdometry EPEC controller. This value is then used by
% AccelerationandDashboard EPEC controller in its speed control loop.
bMSS = select(bag,'Topic','/can/atv_machine_selected_speed');
TimeMSS = bMSS.MessageList.Time;
msgMSS = readMessages(bMSS,'DataFormat','struct');

TV_MSS = zeros(length(msgMSS),1);
for k = 1 : length(msgMSS)
    % Following time conversion is verified by toSec() function in ROS.
    TV_MSS(k) = double ( msgMSS{k,1}.TimeReceived.Sec ) + ...
        double ( msgMSS{k,1}.TimeReceived.Nsec ) * 1e-9;
end

% compute log time for /can/atv_odometry_measurement topic
LogTime_MSS = TimeMSS(end) - TimeMSS(1);
% compute sampling frequency from data
Fs_MSS = length(msgMSS)/LogTime_MSS;

% machine selected speed to be used in speed controller
% As long as slip is absent, the controlled variable being front wheel 
% speed or rear wheel speed has same affect.
machine_selected_speed = zeros(length(msgMSS), 1);
machine_selected_distance = zeros(length(msgMSS), 1);
for k = 1 : length(msgMSS)
    machine_selected_speed(k) = msgMSS{k,1}.SelectedSpeed;
    machine_selected_distance(k) = msgMSS{k,1}.SelectedDistance;
end

% GASC -- EPEC message for recording commands
bGASC = select(bag,'Topic','/can/atv_speed_and_steering_commands');
TimeGASC = bGASC.MessageList.Time;
msgGASC = readMessages(bGASC,'DataFormat','struct');

TV_GASC = zeros(length(msgGASC),1);
for k = 1 : length(msgGASC)
    % Following time conversion is verified by toSec() function in ROS.
    TV_GASC(k) = double ( msgGASC{k,1}.TimeReceived.Sec ) + ...
        double ( msgGASC{k,1}.TimeReceived.Nsec ) * 1e-9;
end
% compute log time for /can/atv_odometry_measurement topic
LogTime_GASC = TimeGASC(end) - TimeGASC(1);
% compute sampling frequency from data
Fs_GASC = length(msgGASC)/LogTime_GASC;

% motion control reference
motion_control_ref = zeros(length(msgGASC), 1);
turn_radius_ref = zeros(length(msgGASC), 1);
direct_torque_flag = zeros(length(msgGASC), 1);
for k = 1 : length(msgGASC)
    motion_control_ref(k) = msgGASC{k,1}.MotionControlRef;
    turn_radius_ref(k) = msgGASC{k,1}.TurningRadiusRef;
    direct_torque_flag(k) = msgGASC{k,1}.DirectTorqueFlag;
end