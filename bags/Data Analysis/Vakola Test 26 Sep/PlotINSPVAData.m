%% Plot Extracted Data
% Plot INS Status
figure('Name','INSPVA: Status','Units','inches','PaperUnits', 'inches','Position', [1 1 pwidth pheight], ...
    'PaperPositionMode','Auto','PaperSize',[pwidth pheight]);
plot(Time_INS, INS_Status,'r', 'LineStyle','none', 'Marker','x','LineWidth', 2);
title('INS Status')
grid on; axis tight;
legend({ strcat("1 = Aligning", string(newline),...
    "2 = Bad", string(newline), ...
    "3 = Good", string(newline), ...    
    "7 = Aligned" )},'Location','best'); 

% North and East Velocities -- Map Odometry Data
figure('Name','INSPVA: Velocities','Units','inches','PaperUnits', 'inches','Position', [1 1 pwidth pheight], ...
    'PaperPositionMode','Auto','PaperSize',[pwidth pheight]);
plot(Time_INS, XLinVel, 'LineWidth', 2, 'DisplayName', 'V_E');
hold on;
plot(Time_INS, YLinVel, 'LineWidth', 2, 'DisplayName', 'V_N');
plot(Time_INS, ZLinVel, 'LineWidth', 2, 'DisplayName', 'V_U');
hold off;
xlabel('Time (sec)');ylabel('m/s');
title('Inertial Velocities')
grid on; axis tight;
legend ('Location','best');

% Ground Speed
figure('Name','INSPVA: Ground Speed','Units','inches','PaperUnits', 'inches','Position', [1 1 pwidth pheight], ...
    'PaperPositionMode','Auto','PaperSize',[pwidth pheight]);
plot(Time_INS, GSpeed_INS, 'LineWidth', 2, 'DisplayName', 'Ground Speed');
xlabel('Time (sec)');ylabel('m/s');
title('Ground Speed','Interpreter','latex')
grid on; axis tight;
legend off;%('Location','best');

% Course Angle -- Correct course angle is in RPY angle information
figure('Name','INSPVA: Heading','Units','inches','PaperUnits', 'inches','Position', [1 1 pwidth pheight], ...
    'PaperPositionMode','Auto','PaperSize',[pwidth pheight]);
plot(Time_INS, yaw_INS, 'LineWidth', 2, 'DisplayName', 'Azimuth');
xlabel('Time (sec)'); ylabel('deg');
title('Heading');
hold off;grid on; axis tight;
legend off;%('Location','best');

% Roll and Pitch Angle -- Correct roll and pitch angle is in RPY angle information
figure('Name','INSPVA: Roll and Pitch','Units','inches','PaperUnits', 'inches','Position', [1 1 pwidth pheight], ...
    'PaperPositionMode','Auto','PaperSize',[pwidth pheight]);
subplot(211);
plot(Time_INS, roll_INS, 'LineWidth', 2, 'DisplayName', 'Roll (\phi)');
grid on; axis tight; legend off;%('Location','best');
xlabel('Time (sec)'); ylabel('deg');
title('Roll Angle')
subplot(212);
plot(Time_INS, pitch_INS, 'LineWidth', 2, 'DisplayName', 'Pitch (\theta)');
grid on; axis tight; legend off;%('Location','best');
xlabel('Time (sec)'); ylabel('deg');
title('Pitch Angle')
% sgtitle('Roll and Pitch Angle','Fontsize',12);

% Plot INS Position -- WGS-84
figure('Name','INSPVA: WGS-84','Units','inches','PaperUnits', 'inches','Position', [1 1 pwidth pheight], ...
    'PaperPositionMode','Auto','PaperSize',[pwidth pheight]);
subplot(211);
plot(long_PVA, lat_PVA, 'LineWidth', 2,'HandleVisibility','off');hold on;
plot(long_PVA(1), lat_PVA(1), 'kx', 'LineWidth',2, 'DisplayName', 'Start');
plot(long_PVA(end), lat_PVA(end), 'go', 'LineWidth',2, 'DisplayName', 'End');hold off;
xlabel('Long (deg)');ylabel('Lat (deg)');
title('Vehicle Path')
grid on; axis tight;legend ('Location','northeast');
subplot(212);
plot(Time_PVA, alt_PVA, 'LineWidth', 2, 'DisplayName', 'Altitude GPS');hold on;
xlabel('Time (sec)');ylabel('m');
title('Altitude')
grid on; axis tight;
legend off;%('Location','best');

% Plot integrated path data in local ENU
figure('Name','INSPVA: UTM','Units','inches','PaperUnits', 'inches','Position', [1 1 pwidth pheight], ...
    'PaperPositionMode','Auto','PaperSize',[pwidth pheight]);
subplot(211);
plot(UTM_INS(1,:), UTM_INS(2,:), 'LineWidth', 2, 'HandleVisibility','off');hold on;
plot(UTM_INS(1,1), UTM_INS(2,1), 'rx', 'LineWidth',2, 'DisplayName', 'Start');
plot(UTM_INS(1,end), UTM_INS(2,end), 'go', 'LineWidth',2, 'DisplayName', 'End');hold off;
xlabel('Easting (m)');ylabel('Northing (m)');
grid on; axis tight; title('Vehicle Path')
legend ('Location','best');
subplot(212);
plot(Time_INS, UTM_INS(3,:), 'LineWidth', 2, 'DisplayName', 'Altitude');
xlabel('Time (sec)');
ylabel('m');
title('Altitude');
grid on; axis tight; legend off;%('Location','best');

% Lat/Lon/Alt Position Covariances
figure('Name','INSCOV: Position Covariances','Units','inches','PaperUnits', 'inches','Position', [1 1 pwidth pheight], ...
    'PaperPositionMode','Auto','PaperSize',[pwidth pheight]);
plot(Time_INS, Pos_Cov_PVA(1,:),'LineWidth',2,'DisplayName','\sigma^2_{LAT}'); hold on;
plot(Time_INS, Pos_Cov_PVA(5,:),'LineWidth',2,'DisplayName','\sigma^2_{LON}'); 
plot(Time_INS, Pos_Cov_PVA(9,:),'LineWidth',2,'DisplayName','\sigma^2_{ALT}'); 
hold off; grid on; axis tight; legend('Location','best');
xlabel('Time (sec)');ylabel('m^2');
title('Position Covariances');

% Lat/Lon/Alt Position Covariances
figure('Name','INSCOV: Attitude Covariances','Units','inches','PaperUnits', 'inches','Position', [1 1 pwidth pheight], ...
    'PaperPositionMode','Auto','PaperSize',[pwidth pheight]);
plot(Time_INS, rad2deg(rad2deg(Attitude_Cov_PVA(1,:))),'LineWidth',2,'DisplayName','\sigma^2_{\phi}'); hold on;
plot(Time_INS, rad2deg(rad2deg(Attitude_Cov_PVA(5,:))),'LineWidth',2,'DisplayName','\sigma^2_{\theta}'); 
plot(Time_INS, rad2deg(rad2deg(Attitude_Cov_PVA(9,:))),'LineWidth',2,'DisplayName','\sigma^2_{\psi}'); 
hold off; grid on; axis tight; legend('Location','best');
xlabel('Time (sec)');ylabel('deg^2');
title('Attitude Covariances');

% Lat/Lon/Alt Position Covariances
figure('Name','INSCOV: Velocity Covariances','Units','inches','PaperUnits', 'inches','Position', [1 1 pwidth pheight], ...
    'PaperPositionMode','Auto','PaperSize',[pwidth pheight]);
plot(Time_INS, Vel_Cov_PVA(1,:),'LineWidth',2,'DisplayName','\sigma^2_{V_E}'); hold on;
plot(Time_INS, Vel_Cov_PVA(5,:),'LineWidth',2,'DisplayName','\sigma^2_{V_N}'); 
plot(Time_INS, Vel_Cov_PVA(9,:),'LineWidth',2,'DisplayName','\sigma^2_{V_U}'); 
hold off; grid on; axis tight; legend('Location','best');
xlabel('Time (sec)');ylabel('(m/s)^2');
title('Velocity Covariances');