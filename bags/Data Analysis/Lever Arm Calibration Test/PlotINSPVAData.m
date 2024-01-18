%% Plot Extracted Data
% North and East Velocities -- Map Odometry Data
figure('Name','INSPVA Data','Units','inches','PaperUnits', 'inches','Position', [1 1 pwidth pheight], ...
    'PaperPositionMode','Auto','PaperSize',[pwidth pheight]);
plot(Time_INS, XLinVel, 'LineWidth', 2, 'DisplayName', 'V_E');
hold on;
plot(Time_INS, YLinVel, 'LineWidth', 2, 'DisplayName', 'V_N');
hold off;
xlabel('Time (sec)');ylabel('m/s');
title('Inertial Velocities')
grid on; axis tight;
legend ('Location','best');

% Up Velocity -- Map Odometry Data
figure('Name','INSPVA Data','Units','inches','PaperUnits', 'inches','Position', [1 1 pwidth pheight], ...
    'PaperPositionMode','Auto','PaperSize',[pwidth pheight]);
plot(Time_INS, ZLinVel, 'LineWidth', 2, 'DisplayName', 'V_U');
xlabel('Time (sec)');ylabel('m/s');
title('Up Velocity');
grid on; axis tight;
legend off;%('Location','best');

figure('Name','INSPVA Data','Units','inches','PaperUnits', 'inches','Position', [1 1 pwidth pheight], ...
    'PaperPositionMode','Auto','PaperSize',[pwidth pheight]);
plot(Time_INS, GSpeed_INS, 'LineWidth', 2, 'DisplayName', 'Ground Speed');
xlabel('Time (sec)');ylabel('m/s');
title('Ground Speed = $\sqrt{V_E^2 + V_N^2}$','Interpreter','latex')
grid on; axis tight;
legend off;%('Location','best');

% Course Angle -- Correct course angle is in RPY angle information
figure('Name','INSPVA Data','Units','inches','PaperUnits', 'inches','Position', [1 1 pwidth pheight], ...
    'PaperPositionMode','Auto','PaperSize',[pwidth pheight]);
plot(Time_INS, yaw_INS, 'LineWidth', 2, 'DisplayName', 'Azimuth');
xlabel('Time (sec)'); ylabel('deg');
title('Heading');
hold off;grid on; axis tight;
legend off;%('Location','best');

% Roll and Pitch Angle -- Correct roll and pitch angle is in RPY angle information
figure('Name','INSPVA Data','Units','inches','PaperUnits', 'inches','Position', [1 1 pwidth pheight], ...
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
figure('Name','INSPVA Data','Units','inches','PaperUnits', 'inches','Position', [1 1 pwidth pheight], ...
    'PaperPositionMode','Auto','PaperSize',[pwidth pheight]);
subplot(211);
plot(long_PVA, lat_PVA, 'LineWidth', 2,'HandleVisibility','off');hold on;
plot(long_PVA(1), lat_PVA(1), 'kx', 'LineWidth',2, 'DisplayName', 'Initial Point');
plot(long_PVA(end), lat_PVA(end), 'go', 'LineWidth',2, 'DisplayName', 'Final Point');hold off;
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
figure('Name','INSPVA Data','Units','inches','PaperUnits', 'inches','Position', [1 1 pwidth pheight], ...
    'PaperPositionMode','Auto','PaperSize',[pwidth pheight]);
subplot(211);
plot(UTM_INS(1,:), UTM_INS(2,:), 'LineWidth', 2, 'HandleVisibility','off');hold on;
plot(UTM_INS(1,1), UTM_INS(2,1), 'rx', 'LineWidth',2, 'DisplayName', 'Initial Point');
plot(UTM_INS(1,end), UTM_INS(2,end), 'go', 'LineWidth',2, 'DisplayName', 'Final Point');hold off;
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
figure('Name','INSCOV Data','Units','inches','PaperUnits', 'inches','Position', [1 1 pwidth pheight], ...
    'PaperPositionMode','Auto','PaperSize',[pwidth pheight]);
plot(Time_INS, Cov_PVA(1,:),'LineWidth',2,'DisplayName','P_{LAT}'); hold on;
plot(Time_INS, Cov_PVA(5,:),'LineWidth',2,'DisplayName','P_{LON}'); 
plot(Time_INS, Cov_PVA(9,:),'LineWidth',2,'DisplayName','P_{ALT}'); 
hold off; grid on; axis tight; legend('Location','best');
title('INS Position Covariances');