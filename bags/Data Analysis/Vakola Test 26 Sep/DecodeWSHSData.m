%% WSHS -- EPEC wheel displacement measurements
bWSHS = select(bag,'Topic','/can/atv_wheel_displacement_measurement');    
TimeWSHS = bWSHS.MessageList.Time;
msgWSHS = readMessages(bWSHS,'DataFormat','struct');

TV_WSHS = zeros(length(msgWSHS),1);
for k = 1 : length(msgWSHS)
    % Following time conversion is verified by toSec() function in ROS.
    TV_WSHS(k) = double ( msgWSHS{k,1}.TimeReceived.Sec ) + ...
        double ( msgWSHS{k,1}.TimeReceived.Nsec ) * 1e-9;
end
% compute log time for /can/atv_odometry_measurement topic
LogTime_WSHS = TimeWSHS(end) - TimeWSHS(1);
% compute sampling frequency from data
Fs_WSHS = length(msgWSHS)/LogTime_WSHS;

% extract voltage values from RTP sensors
voltage_FL = zeros(length(msgWSHS), 1);
voltage_FR = zeros(length(msgWSHS), 1);
voltage_RL = zeros(length(msgWSHS), 1);
voltage_RR = zeros(length(msgWSHS), 1);
for k = 1 : length(msgWSHS)
    voltage_FL(k) = msgWSHS{k,1}.FrontLeftHeight;
    voltage_FR(k) = msgWSHS{k,1}.FrontRightHeight;
    voltage_RL(k) = msgWSHS{k,1}.RearLeftHeight;
    voltage_RR(k) = msgWSHS{k,1}.RearRightHeight;
end

%% Get the Coefficients from Calibration Data and Estimate Wheel Heights
% wheel heights above ground in meters
height_FR = K1_FR(1) * voltage_FR + K1_FR(2);
height_FL = K1_FL(1) * voltage_FL + K1_FL(2);
height_RL = K1_RL(1) * voltage_RL + K1_RL(2);
height_RR = K1_RR(1) * voltage_RR + K1_RR(2);
% mass on each wheel in kgs
mass_FR = K2_FR(1) * height_FR + K2_FR(2);
mass_FL = K2_FL(1) * height_FL + K2_FL(2);
mass_RL = K2_RL(1) * height_RL + K2_RL(2);
mass_RR = K2_RR(1) * height_RR + K2_RR(2);
% compute the displacement of each wheel
x_FR = height_FR - height_FR(1);
x_FL = height_FL - height_FL(1);
x_RL = height_RL - height_RL(1);
x_RR = height_RR - height_RR(1);
% compute change in load (Newton)
W_FR = (mass_FR-mass_FR(1))*g;
W_FL = (mass_FL-mass_FL(1))*g;
W_RL = (mass_RL-mass_RL(1))*g;
W_RR = (mass_RR-mass_RR(1))*g;
% compute net force on each wheel due to spring compression
F_FR = Ks_FR * x_FR;
F_FL = Ks_FL * x_FL;
F_RL = Ks_RL * x_RL;
F_RR = Ks_RR * x_RR;