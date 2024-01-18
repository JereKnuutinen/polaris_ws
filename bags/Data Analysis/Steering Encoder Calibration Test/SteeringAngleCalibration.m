% Important flag to do steering encoder calibration
close all; clear; clc;
save_coefficients = 0;
filename = 'AA_CAN_01.bag';
bagInfo = rosbag('info',filename);
bag = rosbag(filename);

DecodeINSPVAData;
DecodeEPECData;
DecodeIMUData;

start_time = 0.0;
Time_INS = linspace(start_time, LogTime_INS, length(msgINS));
Time_SODO = linspace(start_time, LogTime_SODO, length(msgSODO));
Time_IMU  = linspace(start_time, LogTime_IMU,  length(msgIMU));

% Define car parameters and constants
L = 1.83; % Wheel base of Polaris -- m
W = 1.44; % Polaris width -- m

% rotation matrix around X-axis and its derivatives
Rx = @(roll)[1 0 0;0 cos(roll) -sin(roll); 0 sin(roll) cos(roll)];
DRx = @(roll, droll)[0 0 0;0 -sin(roll) -cos(roll);...
    0 cos(roll) -sin(roll)].*droll;

% Rotation matrix around Y-axis and its derivatives
Ry = @(pitch)[cos(pitch) 0 sin(pitch); 0 1 0;-sin(pitch) 0 cos(pitch)];
DRy = @(pitch, dpitch)[-sin(pitch) 0 cos(pitch); 0 0 0;...
    -cos(pitch) 0 -sin(pitch)].*dpitch;

% Rotation matrix around Z-axis and its derivatives
Rz = @(yaw)[cos(yaw) -sin(yaw) 0;sin(yaw) cos(yaw) 0; 0 0 1];
DRz = @(yaw, dyaw) [-sin(yaw) -cos(yaw) 0;cos(yaw) -sin(yaw) 0;...
    0 0 0].*dyaw;

% reassign Euler angles from /novatel/odom_map topic
phi = deg2rad(roll_INS); theta = deg2rad(pitch_INS); psi = deg2rad(yaw_INS);
% reassign body rates from IMU data /novatel/imu
p = deg2rad(roll_rate); q = deg2rad(pitch_rate); r = deg2rad(yaw_rate);
% reassign vehicle position in local ENU coordinates
X_g = ENU(1,:); Y_g = ENU(2,:); Z_g = ENU(3,:);
% reassign vehicle's East, North, and Up velocity from GPS
dotE = XLinVel; dotN = YLinVel; dotU = ZLinVel;
dotENU = [dotE; dotN; dotU];
% reassing vehicle body accelerations
a_x = Xacc; a_y = Yacc; a_z = Zacc;

% Convert the inertial velocities to body velocities
dotUVW = zeros(size(dotENU));
for i = 2 : length(phi)
    C = Rz(psi(i))*Ry(theta(i))*Rx(phi(i)); % body to global (CG)
    dotUVW(:,i) = C.'*dotENU(:,i);
end

% Extract body velocities and position
u = dotUVW(1,:)'; v = dotUVW(2,:)'; w = dotUVW(3,:)';

% compute curvature of the path obtained from GPS
Xvk = [X_g',Y_g'];
[Lk,Rk,Kv] = curvature(Xvk);
Kf = 1./Rk; % path curvature of front body

% compute curvature of the dynamic trajectory of CG using yaw rate
Vt = sqrt(u.^2 + v.^2 + w.^2);
idx = find(Vt > 0);
Ky(idx) = r(idx)./Vt(idx);Ky = Ky';
stg = atan(L*Ky); % Estimated steering angle
Kcl = Ky(idx)./(1-0.5*W*Ky(idx)); % Curvature Left Wheel
Kcr = Ky(idx)./(1+0.5*W*Ky(idx)); % Curvature Left Wheel
stgL = atan(L*Kcl);
stgR = atan(L*Kcr);

% 3 Point data fit for steering encoder using ./Bags/AA_CAN_01.bag file.
Enc_Val = [485621; 477331; 468920];
Stg_Val = [-31.53; 0.0; 31.68];
y = Stg_Val; A = [Enc_Val ones(length(Enc_Val), 1)];
K_T = (A'*A)^-1*A'*y; % LS Estimate

if save_coefficients
    save('StrEncConvCoeff.mat', "K_T");
else
    load StrEncConvCoeff.mat;
end
stg_enc = K_T(1) * steering_encode_position + K_T(2);

%% Plot important results
pwidth = 3;
pheight = 3;
figure('Units','inches','PaperUnits', 'inches','Position', [1 1 pwidth pheight], ...
    'PaperPositionMode','Auto','PaperSize',[pwidth pheight]);
plot(X_g, Y_g, 'LineWidth', 2, 'DisplayName', 'Vehicle Path');
hold on;
plot(X_g(1), X_g(1), 'rx', 'LineWidth',2, 'DisplayName', 'Initial Point');
plot(X_g(end), X_g(end), 'go', 'LineWidth',2, 'DisplayName', 'Final Point');
hold off;
xlabel('Easting -- m');ylabel('Northing -- m');
grid on; axis tight;legend('Location','best');

figure('Units','inches','PaperUnits', 'inches','Position', [1 1 pwidth pheight], ...
    'PaperPositionMode','Auto','PaperSize',[pwidth pheight]);
plot(Time_INS,Kf,...
    'LineWidth',1.5,'DisplayName','$K_f$ -- Path Curvature');hold on;
plot(Time_INS,Ky,...
    'LineWidth',2,'DisplayName','$K_y = r/V_T$');
hold off;
grid on;
axis ([0 150 -5 5]);
legend('Location','best','Interpreter','latex');
xlabel('Time - s');
title('Curvature -- 1/m');

figure('Units','inches','PaperUnits', 'inches','Position', [1 1 pwidth pheight], ...
    'PaperPositionMode','Auto','PaperSize',[pwidth pheight]);
plot(Time_INS,u,...
    'LineWidth',1.5,'DisplayName','u');hold on;
plot(Time_INS,v,...
    'LineWidth',1.5,'DisplayName','v');
plot(Time_INS,w,...
    'LineWidth',1.5,'DisplayName','w');
hold off;
grid on;
axis tight;
legend('Location','best','Interpreter','latex');
xlabel('Time - s');
title('Body Velocities');

figure('Units','inches','PaperUnits', 'inches','Position', [1 1 pwidth pheight], ...
    'PaperPositionMode','Auto','PaperSize',[pwidth pheight]);
subplot(211);
plot(Time_SODO, steering_encode_position, 'LineWidth',2);
axis ([0 150 -inf inf]); xlabel('Time - s'); ylabel('Absoulte Units')
grid on;
title('Steering Encoder Value');
subplot(212);
plot(Time_INS, rad2deg(stg), 'LineWidth',2);
axis ([0 150 -inf inf]); xlabel('Time - s'); ylabel('Degrees')
grid on;
title('Steering Angle');

figure('Units','inches','PaperUnits', 'inches','Position', [1 1 pwidth pheight], ...
    'PaperPositionMode','Auto','PaperSize',[pwidth pheight]);
plot(Time_SODO, stg_enc, 'LineWidth',2,'DisplayName','\delta_{E}'); hold on;
plot(Time_INS, rad2deg(stg), 'LineWidth',2,'DisplayName','\delta = tan^{-1}(L*K)'); hold off;
axis ([0 150 -inf inf]); xlabel('Time - s'); ylabel('Steering Angle -- deg'); grid on; legend('Location','best');
title('Calibration Results for steering Encoder')