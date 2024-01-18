% Important flag to do steering encoder calibration
ExtractingData;

% position vectors for right_{Front, Rear} and left_{Front, Rear} corners
% of the vehicle with respect to CG assumed at {0,0,0}
prF = [ l/2; -t/2; -h/2];
prR = [-l/2; -t/2; -h/2];
plF = [ l/2;  t/2; -h/2];
plR = [-l/2;  t/2; -h/2];

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

% Assemble data from each sensor into matrices for post-processing.
INS = [roll_INS pitch_INS yaw_INS ENU' XLinVel' YLinVel' ZLinVel'];
IMU = [roll_rate pitch_rate yaw_rate Xacc' Yacc' Zacc'];
WSHS = [x_FR x_FL x_RL x_RR];

% resample the WSHS values to 20 Hz
[WSHS, Time] = resample(WSHS, TV_WSHS, Fs);
Time = Time - Time(1); % uniformly spaced time vector with t0 = 0.0.

% assign wheel speed and steering angle
V_ODOM = wheel_ground_speed;
stg = stg_enc;

% Check the number of arrays and append the numbers at end
N = max([length(INS) length(IMU) length(V_ODOM) length(stg) length(WSHS)]);
nn = N - ([length(INS) length(IMU) length(V_ODOM) length(stg) length(WSHS)]);
% Repear the last values till the length of longest vector
INS = cat(1, INS, repmat(INS(end,:), nn(1), 1) );
IMU = cat(1, IMU, repmat(IMU(end,:), nn(2), 1) );
V_ODOM = cat(1, V_ODOM, repmat(V_ODOM(end,:), nn(3), 1) );
stg = cat(1, stg, repmat(stg(end,:), nn(4), 1) );
WSHS = cat(1, WSHS, repmat(WSHS(end,:), nn(5), 1) );
% Append Time vector w.r.t. Fs
if nn(5) > 0
    Time = cat(1, Time, Time(end)+1/Fs:1/Fs:Time(end)+nn(5)/Fs);
end

% extract Euler angles from time adjusted data matrices
phi = deg2rad(INS(:,1)); theta = deg2rad(INS(:,2)); psi = deg2rad(INS(:,3));
% reassign vehicle position in local ENU coordinates
X_g = INS(:,4); Y_g = INS(:,5); Z_g = INS(:,6);
% reassign vehicle's East, North, and Up velocity from INS
dotE = INS(:,7); dotN = INS(:,8); dotU = INS(:,9);
dotENU = [dotE dotN dotU];

% extract body rates from resampled data matrix
p = deg2rad(IMU(:,1)); q = deg2rad(IMU(:,2)); r = deg2rad(IMU(:,3));
% extract vehicle body accelerations from resampled data matrix
a_x = IMU(:,4); a_y = IMU(:,5); a_z = IMU(:,6);
% compute total ground speed from INS
V_INS = sqrt(dotE.^2 + dotN.^2 + dotU.^2);

% Convert the inertial velocities to body velocities
dotUVW = zeros(size(dotENU));
for i = 1 : length(dotENU)
    C = Rz(psi(i))*Ry(theta(i))*Rx(phi(i)); % body to global (CG)
    dotUVW(i,:) = C.'*(dotENU(i,:)');
end
% Extract body velocities
u = dotUVW(:,1); v = dotUVW(:,2); w = dotUVW(:,3);

% Wheel displacement variables -- [x_FR x_FL x_RL x_RR];
dx_FR = WSHS(:,1);dx_FL = WSHS(:,2);
dx_RL = WSHS(:,3);dx_RR = WSHS(:,4);

%% Detect and remove any trends in height data
% % LT = trenddecomp(Z_g);
% bp = 0;
% Z_detrend = detrend(Z_g,1,bp,'SamplePoints',Time_Odom,'Continuous',false);
% figure;
% % subplot(211);
% % plot(Time_Odom,Z_g,Time_Odom,LT);grid on; axis tight;
% % legend("Data","Long-term");
% % subplot(212);
% plot(Time_Odom, Z_g, Time_Odom, Z_detrend, Time_Odom, Z_g-Z_detrend,'LineWidth',1.5);grid on; axis tight;
% legend("Actual", "Detrend", "Trend");
%
% [valleyValues, indexes] = findpeaks(-y);
% yBaseline = interp1(x(indexes), -valueValues, x);
% yCorrected = y - yBaseline;

%% Plot important results
% plot integration of displacement readings
figure('Units','inches','PaperUnits', 'inches','Position', [1 1 pwidth pheight], ...
    'PaperPositionMode','Auto','PaperSize',[pwidth pheight]);
subplot(221);
plot(Time, dx_FL, 'LineWidth',2);hold on;
grid on; axis tight; xlabel('sec'); ylabel('m')
legend off;%('Location','best');
title('FL: \int \Deltax_{FL} dt');
subplot(222);
plot(Time, dx_FR,'r', 'LineWidth',2);hold on;
grid on; axis tight; xlabel('sec'); ylabel('m')
legend off;%('Location','best');
title('FR: \int \Deltax_{FR} dt');
subplot(223);
plot(Time, dx_RL,'g', 'LineWidth',2);hold on;
grid on; axis tight; xlabel('sec'); ylabel('m')
legend off;%('Location','best');
title('RL: \int \Deltax_{RL} dt');
subplot(224);
plot(Time, dx_RR,'c','LineWidth',2);hold on;
grid on; axis tight; xlabel('sec'); ylabel('m')
legend off;%('Location','best');
title('RR: \int \Deltax_{RR} dt');

% plot position data in local ENU frame
figure('Units','inches','PaperUnits', 'inches','Position', [1 1 pwidth pheight], ...
    'PaperPositionMode','Auto','PaperSize',[pwidth pheight]);
plot(X_g, Y_g, 'LineWidth', 2, 'DisplayName', 'INS Path');hold on;
plot(X_g(1), Y_g(1), 'rx', 'LineWidth',2, 'DisplayName', 'Initial Point');
plot(X_g(end), Y_g(end), 'go', 'LineWidth',2, 'DisplayName', 'Final Point');hold off;
xlabel('Easting -- m');ylabel('Northing -- m');
grid on; axis tight;legend('Location','best');
title('Position Data');

% plot altitude data -- a comparison
figure('Units','inches','PaperUnits', 'inches','Position', [1 1 pwidth pheight], ...
    'PaperPositionMode','Auto','PaperSize',[pwidth pheight]);
hold on;
plot(Time, Z_g, 'LineWidth', 2, 'DisplayName', 'H_{INS}');
hold off;
grid on; axis tight; xlabel('sec'); ylabel('m')
legend('Location','best');
title('Height Data');

figure('Units','inches','PaperUnits', 'inches','Position', [1 1 pwidth pheight], ...
    'PaperPositionMode','Auto','PaperSize',[pwidth pheight]);
plot(Time,dotE,...
    'LineWidth',1.5,'DisplayName','$\dot{E}$');hold on;
plot(Time,dotN,...
    'LineWidth',1.5,'DisplayName','$\dot{N}$');
plot(Time,dotU,...
    'LineWidth',1.5,'DisplayName','$\dot{U}$');
hold off;
grid on;
axis tight;
legend('Location','best','Interpreter','latex');
xlabel('Time - s');
title('Inertial Velocities');

figure('Units','inches','PaperUnits', 'inches','Position', [1 1 pwidth pheight], ...
    'PaperPositionMode','Auto','PaperSize',[pwidth pheight]);
plot(Time,u,...
    'LineWidth',1.5,'DisplayName','u');hold on;
plot(Time,v,...
    'LineWidth',1.5,'DisplayName','v');
plot(Time,w,...
    'LineWidth',1.5,'DisplayName','w');
hold off;
grid on;
axis tight;
legend('Location','best','Interpreter','latex');
xlabel('Time - s');
title('Body Velocities');

% plot ground speeds from different sources
figure('Units','inches','PaperUnits', 'inches','Position', [1 1 pwidth pheight], ...
    'PaperPositionMode','Auto','PaperSize',[pwidth pheight]);
hold on;
plot(Time, V_ODOM, 'DisplayName','V_{ODOM}','LineWidth',2);
plot(Time, u, 'DisplayName','u','LineWidth',2);
hold off;
axis tight; grid on; legend('Location','best'); xlabel('Time -- s'); ylabel('m/s')
title('Speed Data');

% plot Euler angles from INS data
figure('Units','inches','PaperUnits', 'inches','Position', [1 1 pwidth pheight], ...
    'PaperPositionMode','Auto','PaperSize',[pwidth pheight]);
subplot(211);
hold on;
plot(Time, rad2deg(phi), 'LineWidth', 2, 'DisplayName', '\phi');
hold off;
grid on; axis tight; legend('Location','best');
xlabel('Time (sec)'); ylabel('degrees');
title('Roll');
subplot(212);
hold on;
plot(Time, rad2deg(theta), 'LineWidth', 2, 'DisplayName', '\theta');
hold off;
grid on; axis tight; legend('Location','best');
xlabel('Time (sec)'); ylabel('degrees');
title('Pitch');

% plot yaw angle
figure('Units','inches','PaperUnits', 'inches','Position', [1 1 pwidth pheight], ...
    'PaperPositionMode','Auto','PaperSize',[pwidth pheight]);
hold on;
plot(Time, rad2deg(psi), 'LineWidth', 2, 'DisplayName', '\psi');
hold off;
grid on; axis tight; legend('Location','best');
xlabel('Time (sec)'); ylabel('degrees');
title('Heading');

% plot steering angle
figure('Units','inches','PaperUnits', 'inches','Position', [1 1 pwidth pheight], ...
    'PaperPositionMode','Auto','PaperSize',[pwidth pheight]);
plot(Time, stg, 'LineWidth',2);
axis tight; grid on; xlabel('Time -- s'); ylabel('degrees')
title('Steering');

% To illustrate relative positioning w.r.t gnss (antenna) and ins (span) center
figure('Units','inches','PaperUnits', 'inches','Position', [1 1 pwidth pheight], ...
    'PaperPositionMode','Auto','PaperSize',[pwidth pheight]);
plot(ENU(1,:), ENU(2,:), 'LineWidth', 2, 'DisplayName', 'INS Path');
hold on;
plot(ENU(1,1), ENU(2,1), 'rx', 'LineWidth',2, 'DisplayName', 'INS Initial Point');
plot(ENU(1,end), ENU(2,end), 'go', 'LineWidth',2, 'DisplayName', 'INS Final Point');
plot(ENU_utm(1,:), ENU_utm(2,:), 'LineWidth', 2, 'DisplayName', 'GNSS Path');
plot(ENU_utm(1,1), ENU_utm(2,1), 'kx', 'LineWidth',2, 'DisplayName', 'GNSS Initial Point');
plot(ENU_utm(1,end), ENU_utm(2,end), 'co', 'LineWidth',2, 'DisplayName', 'GNSS Final Point');
hold off;
xlabel('Easting -- m');ylabel('Northing -- m');
grid on; axis tight;legend('Location','best');