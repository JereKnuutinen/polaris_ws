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

wheel_drive_mode = zeros(length(msgGASP), 1);
gear_ratio = zeros(length(msgGASP), 1);
for k = 1 : length(msgGASP)
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
choose_speed = 1; % 1 = left/right, 0 = front/rear.
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