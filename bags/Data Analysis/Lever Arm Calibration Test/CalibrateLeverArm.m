% Important flag to do steering encoder calibration
close all; clear; clc;

%% Check the information of a bag file.
filename = 'AA_LEVERARMS_CALIB.bag';
bagInfo = rosbag('info',filename);
bag = rosbag(filename);

% decode data.
DecodeINSPVAData;
DecodeGPSData;
DecodeIMUData;

% define time vectors
start_time = 0.0;
Time_UTM  = linspace(start_time, LogTime_UTM,  length(msgUTM));
Time_INS = linspace(start_time, LogTime_INS, length(msgINS));
Time_GPS = linspace(start_time, LogTime_GPS, length(msgGPS));
pwidth = 3;
pheight = 3;

% define rotation matrix defined for positive counter-clockwise rotation
Rz = @(yaw)[cos(yaw) -sin(yaw);sin(yaw) cos(yaw)];

% To record data points from 
% positioning w.r.t gnss (antenna) and ins (span) center
figure('Name','Calibration','Units','inches','PaperUnits', 'inches','Position', [1 1 pwidth pheight], ...
    'PaperPositionMode','Auto','PaperSize',[pwidth pheight]);
subplot(211);
plot(Time_UTM, UTM_GPS(1,:),'LineWidth', 2, 'DisplayName', 'Easting -- GNSS');hold on;
plot(Time_INS, UTM_INS(1,:),'LineWidth', 2, 'DisplayName', 'Easting -- INS');hold off;
grid on; axis tight; legend('Location','best');
subplot(212);
plot(Time_UTM, UTM_GPS(2,:),'LineWidth', 2, 'DisplayName', 'Northing -- GNSS');hold on;
plot(Time_INS, UTM_INS(2,:),'LineWidth', 2, 'DisplayName', 'Northing -- INS');hold off;
grid on; axis tight; legend('Location','best');

% Course Angle -- Correct course angle is in RPY angle information
figure('Name','Calibration','Units','inches','PaperUnits', 'inches','Position', [1 1 pwidth pheight], ...
    'PaperPositionMode','Auto','PaperSize',[pwidth pheight]);
plot(Time_INS, yaw_imu, 'LineWidth', 2, 'DisplayName', 'Azimuth');hold on;
plot(Time_INS, yaw_INS, 'LineWidth', 2, 'DisplayName', 'Heading');hold on;
xlabel('Time (sec)'); ylabel('deg');
title('Heading');
hold off;grid on; axis tight;
legend off;%('Location','best');

% check roll and pitch angles
figure('Units','inches','PaperUnits', 'inches','Position', [1 1 pwidth pheight], ...
    'PaperPositionMode','Auto','PaperSize',[pwidth pheight]);
subplot(211);
plot(Time_INS, roll_INS, 'LineWidth', 2, 'DisplayName', 'Roll (\phi)');
grid on; axis tight; legend('Location','best');
xlabel('Time (sec)'); ylabel('degrees');
subplot(212);
plot(Time_INS, pitch_INS, 'LineWidth', 2, 'DisplayName', 'Pitch (\theta)');
grid on; axis tight; legend('Location','best');
xlabel('Time (sec)'); ylabel('degrees');

%% load cursor point data collected from above figure
load GNSS_Easting_Points.mat;
load INS_Easting_Points.mat;
load GNSS_Northing_Points.mat;
load INS_Northing_Points.mat;

GNSS_POINTS_EAST = gnss_easting_points(:,2);
GNSS_POINTS_NORTH = gnss_northing_points(:,2);
INS_POINTS_EAST = ins_easting_points(:,2);
INS_POINTS_NORTH = ins_northing_points(:,2);

% first point -- P1
GNSS_P1 = [GNSS_POINTS_EAST(1), GNSS_POINTS_NORTH(1)]';
INS_P1 = [INS_POINTS_EAST(1), INS_POINTS_NORTH(1)]';
yaw_P1 = deg2rad(100.9668);

% second point -- P2
GNSS_P2 = [GNSS_POINTS_EAST(2), GNSS_POINTS_NORTH(2)]';
INS_P2 = [INS_POINTS_EAST(2), INS_POINTS_NORTH(2)]';
yaw_P2 = deg2rad(101.1247);

% third point -- P3
GNSS_P3 = [GNSS_POINTS_EAST(3), GNSS_POINTS_NORTH(3)]';
INS_P3 = [INS_POINTS_EAST(3), INS_POINTS_NORTH(3)]';
yaw_P3 = deg2rad(102.7987);

% fourth point -- P4
GNSS_P4 = [GNSS_POINTS_EAST(4), GNSS_POINTS_NORTH(4)]';
INS_P4 = [INS_POINTS_EAST(4), INS_POINTS_NORTH(4)]';
yaw_P4 = deg2rad(282.679);

wheel_base = norm(GNSS_P1 - GNSS_P2);
track_width = norm(GNSS_P1 - GNSS_P3);
fprintf('Wheel base = %3.3f m, and Track width = %3.3f m\n', wheel_base, track_width);

INS_DIST_P1P2 = norm(INS_P1 - INS_P2);
INS_DIST_P1P3 = norm(INS_P1 - INS_P3);

%% Results for 180 deg turn test
% compute mid-points for INS and GNSS using P1 and P4.
GNSS_MID = 0.5*(GNSS_P1 + GNSS_P4);
INS_MID = 0.5*(INS_P1 + INS_P4);

% the distance between INS and GNSS P1, P4
DIST_P1 = norm(GNSS_P1 - INS_P1);
DIST_P4 = norm(GNSS_P4 - INS_P4);

% distance between two mid-points
DIST_MID = norm(GNSS_MID - INS_MID);

% select the vehicle center for P1 and P4
VEHICLE_CENTER = INS_MID;

% center P1, P4 and the mid-points w.r.t. selected vehicle center
GNSS_MID_CENTER = GNSS_MID - VEHICLE_CENTER;
INS_MID_CENTER = INS_MID - VEHICLE_CENTER;
GNSS_P1_CENTER = GNSS_P1 - VEHICLE_CENTER;
GNSS_P4_CENTER = GNSS_P4 - VEHICLE_CENTER;
INS_P1_CENTER = INS_P1 - VEHICLE_CENTER;
INS_P4_CENTER = INS_P4 - VEHICLE_CENTER;

% plot results
figure('Units','inches','PaperUnits', 'inches','Position', [1 1 pwidth pheight], ...
    'PaperPositionMode','Auto','PaperSize',[pwidth pheight]);
hold on;
plot([GNSS_P1_CENTER(1) GNSS_P4_CENTER(1)] ,[GNSS_P1_CENTER(2) GNSS_P4_CENTER(2)],'s-','linewidth',2,'DisplayName','GNSS Points');
plot(GNSS_MID_CENTER(1),GNSS_MID_CENTER(2),'s','linewidth',2,'DisplayName','GNSS Mid-point');
plot([INS_P1_CENTER(1) INS_P4_CENTER(1)],[INS_P1_CENTER(2) INS_P4_CENTER(2)],'d-','linewidth',2,'DisplayName','INS Points');
plot(INS_MID_CENTER(1),INS_MID_CENTER(2),'d','linewidth',2,'DisplayName','INS Mid-point');
hold off; grid on; axis tight;
xlabel('Easting'); ylabel('Northing');
legend('Location','best');
str = sprintf('Mid-points at %c = %0.3f m', char(916), DIST_MID);
title(str);

% rotate the P1 and P4 with respect to selected mid-point
GNSS_P1_ROT = Rz(yaw_P1)*GNSS_P1_CENTER;
GNSS_P4_ROT = Rz(yaw_P4)*GNSS_P4_CENTER;
INS_P1_ROT = Rz(yaw_P1)*INS_P1_CENTER;
INS_P4_ROT = Rz(yaw_P4)*INS_P4_CENTER;

OFFSET_P1 = INS_P1_ROT - GNSS_P1_ROT;
OFFSET_P4 = INS_P4_ROT - GNSS_P4_ROT;
AVG_OFFSET = 0.5*(OFFSET_P1 + OFFSET_P4);
fprintf('Average Offset between INS and GNSS is (%3.3f, %3.3f) m\n', AVG_OFFSET);
% plot results
figure('Units','inches','PaperUnits', 'inches','Position', [1 1 pwidth pheight], ...
    'PaperPositionMode','Auto','PaperSize',[pwidth pheight]);
hold on;
plot([GNSS_P1_ROT(1) GNSS_P4_ROT(1)] ,[GNSS_P1_ROT(2) GNSS_P4_ROT(2)],'s-','linewidth',2,'DisplayName','GNSS Points');
plot(GNSS_MID_CENTER(1),GNSS_MID_CENTER(2),'s','linewidth',2,'DisplayName','GNSS Mid-point');
plot([INS_P1_ROT(1) INS_P4_ROT(1)],[INS_P1_ROT(2) INS_P4_ROT(2)],'d-','linewidth',2,'DisplayName','INS Points');
plot(INS_MID_CENTER(1),INS_MID_CENTER(2),'d','linewidth',2,'DisplayName','INS Mid-point');
hold off; grid on; axis tight;
xlabel('Easting'); ylabel('Northing');
legend('Location','best');
str = sprintf('Avg. Offset = (%3.3f, %3.3f) m', AVG_OFFSET);
title(str);

%% Results for (X; Y) lever arm length tests
INITIAL_POISTION = GNSS_P1;
GNSS_P2_P1 = GNSS_P2 - INITIAL_POISTION;
GNSS_P3_P1 = GNSS_P3 - INITIAL_POISTION;
INS_P2_P1 = INS_P2 - INITIAL_POISTION;
INS_P3_P1 = INS_P3 - INITIAL_POISTION;

% GNSS_P2_ROT = Rz(yaw_P2)*GNSS_P2_CENTER;
% GNSS_P3_ROT = Rz(yaw_P3)*GNSS_P3_CENTER;
% INS_P2_ROT  = Rz(yaw_P2)*INS_P2_CENTER;
% INS_P3_ROT  = Rz(yaw_P3)*INS_P3_CENTER;
% DIST_P1P2 = GNSS_P2 - GNSS_P1;
% DIST_P1P3 = GNSS_P3 - GNSS_P1;
% INS_DIST_P1P2 = INS_P1 - INS_P2;
% INS_DIST_P1P3 = INS_P1 - INS_P3;

figure('Units','inches','PaperUnits', 'inches','Position', [1 1 pwidth pheight], ...
    'PaperPositionMode','Auto','PaperSize',[pwidth pheight]);
subplot(211);
hold on;
plot(GNSS_P1(1),GNSS_P1(2),'ko','linewidth',2,'DisplayName','GNSS P1');
plot(GNSS_P2(1),GNSS_P2(2),'k>','linewidth',2,'DisplayName','GNSS P2');
str = sprintf('GNSS: P1 - P2 =  (%3.3f, %3.3f) m', GNSS_P2_P1);
title(str);
hold off; grid on; axis tight;
xlabel('Easting'); ylabel('Northing');
legend('Location','best');
subplot(212);
hold on;
plot(GNSS_P1(1),GNSS_P1(2),'ko','linewidth',2,'DisplayName','GNSS P1');
plot(GNSS_P3(1),GNSS_P3(2),'kv','linewidth',2,'DisplayName','GNSS P3');
str = sprintf('GNSS: P1 - P3 = (%3.3f, %3.3f) m', GNSS_P3_P1);
title(str);
hold off; grid on; axis tight;
xlabel('Easting'); ylabel('Northing');
legend('Location','best');


figure('Units','inches','PaperUnits', 'inches','Position', [1 1 pwidth pheight], ...
    'PaperPositionMode','Auto','PaperSize',[pwidth pheight]);
subplot(211);
hold on;
plot(INS_P1(1),INS_P1(2),'go','linewidth',2,'DisplayName','INS P1');
plot(INS_P2(1),INS_P2(2),'g>','linewidth',2,'DisplayName','INS P2');
hold off; grid on; axis tight;
str = sprintf('INS: P1 - P2 =  (%3.3f, %3.3f) m', INS_P2_P1);
title(str);
legend('Location','best');
xlabel('Easting'); ylabel('Northing');
subplot(212);
hold on;
plot(INS_P1(1),INS_P1(1),'go','linewidth',2,'DisplayName','INS P1');
plot(INS_P3(1),INS_P3(2),'gv','linewidth',2,'DisplayName','INS P3');
hold off; grid on; axis tight;
xlabel('Easting'); ylabel('Northing');
legend('Location','best');
str = sprintf('INS: P1 - P3 = (%3.3f, %3.3f) m', INS_P3_P1);
title(str);