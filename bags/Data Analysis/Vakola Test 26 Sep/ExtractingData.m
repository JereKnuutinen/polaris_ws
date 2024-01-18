%% Determine value of flags
plot_gps_data = 1;
plot_epec_data = 0;
plot_imu_data = 0;
decode_rpy = 0;
display_gpsdata_UTM = 0;

%% Check the information of a bag file.
filename = 'VK_OUTER_SLOW.bag';
bagInfo = rosbag('info',filename);

%% Extract the bag file into a matlab structure.
bag = rosbag(filename);

%% Define and load constants, coefficients, and parameters
% load necessary coefficients
load StrEncConvCoeff.mat;
load SpringCoeffs.mat;
load HgtVsVoltCoeffs.mat;
load MassVsHgtCoeffs.mat;

% declare constants
g = 9.8; % m/s/s

% declare car parameters
l = 2.74; % depth -- m
t = 1.44; % width -- m
h = 1.85; % height -- m
L = 1.83; % Wheel base -- m

% declare sampling constant
Fs = 20.0; % down or up sample to 20Hz

%% Decode important data from logged topics
DecodeWSHSData;
DecodeGPSData;
DecodeINSPVAData;
DecodeIMUData;
DecodeEPECData;

%% Detect time delay between UTM and other sentences
% Note that for accurate delay between GPS and SPAN data, track angle 
% from BESTUTM and azimuth angle from INSPVA should be used. 
% However, the track heading is very noisy in general. 
[DT_UTMandINS, Dsample_UTMandINS] = detect_delays(XLinVel_utm, ...
    GSpeed_INS, LogTime_UTM, LogTime_GPS);
[DT_INSandIMU, Dsample_INSandIMU] = detect_delays(roll_INS, ...
    roll_imu, LogTime_INS, LogTime_IMU);
[DT_UTMandODOM, Dsample_UTMandODOM] = detect_delays(XLinVel_utm,...
    wheel_ground_speed, LogTime_UTM, LogTime_WODO);

%% Create the time vectors for data analysis
start_time = 0.0;
Time_UTM  = linspace(start_time, LogTime_UTM,  length(msgUTM));
Time_GPS  = linspace(start_time, LogTime_GPS,  length(msgGPS));
Time_PSR  = linspace(start_time, LogTime_PSR,  length(msgPSR));
Time_INS = linspace(start_time, LogTime_INS, length(msgINS)) + DT_UTMandINS;
Time_PVA = linspace(start_time, LogTime_PVA, length(msgPVA)) + DT_UTMandINS;
Time_IMU  = linspace(start_time, LogTime_IMU,  length(msgIMU)) + DT_UTMandINS;
Time_WODO = linspace(start_time, LogTime_WODO, length(msgWODO));% + DT_UTMandODOM;
Time_SODO = linspace(start_time, LogTime_SODO, length(msgSODO));% + DT_UTMandODOM;
Time_GASP = linspace(start_time, LogTime_GASP, length(msgGASP));% + DT_UTMandODOM;
Time_WSHS = linspace(start_time, LogTime_WSHS, length(msgWSHS));% + DT_UTMandODOM;

% figure;
% subplot(211);
% plot(Time_UTM, XLinVel_utm, 'LineWidth', 2);hold on;
% plot(Time_WODO-DT_UTMandWODO, wheel_ground_speed, 'LineWidth', 2);hold off;
% grid on; axis tight; title('Actual');
% legend("GPS Speed","Wheel Speed", 'Location','best');
% subplot(212);
% plot(Time_UTM, XLinVel_utm, 'LineWidth', 2);hold on;
% plot(Time_WODO, wheel_ground_speed, 'LineWidth', 2);hold off;
% grid on; axis tight;xlabel('Time -- (sec)');
% legend("GPS Speed","Wheel Speed", 'Location','best');
% title('Aligned')

%% Call Files to Plot Data
pwidth = 3;
pheight = 3;
if plot_gps_data
    PlotGPSData; % data from BESTUTM messages
    PlotINSPVAData; % data from INSPVA messages
end
if plot_imu_data
    PlotIMUData; % data from CORRIMU messages
end
if plot_epec_data
    PlotEPECData; % EPEC Data
    PlotWheelDisplacementData;
end
