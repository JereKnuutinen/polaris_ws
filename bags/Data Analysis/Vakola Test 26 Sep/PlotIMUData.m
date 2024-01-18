%% Plot IMU data.
% Angular Rates
figure('Name','INSPVA: Pitch','Units','inches','PaperUnits', 'inches','Position', [1 1 pwidth pheight], ...
    'PaperPositionMode','Auto','PaperSize',[pwidth pheight]);
plot(Time_IMU, pitch_rate, 'LineWidth', 2, 'DisplayName', 'q');hold on;
plot(Time_IMU, pitch_imu , 'LineWidth', 2, 'DisplayName', '\theta');hold off;
xlabel('Time (sec)'); ylabel('^\circ/s, deg');
title('Pitch Dynamics');
hold off;grid on; axis tight;legend show;

% Roll and Pitch Angle
figure('Name','INSPVA: Roll','Units','inches','PaperUnits', 'inches','Position', [1 1 pwidth pheight], ...
    'PaperPositionMode','Auto','PaperSize',[pwidth pheight]);
plot(Time_IMU, roll_rate, 'r', 'LineWidth', 2, 'DisplayName', 'p');hold on;
plot(Time_IMU, roll_imu,'g', 'LineWidth', 2, 'DisplayName', '\phi');hold off;
xlabel('Time (sec)'); ylabel('^\circ/s, deg');
title('Roll Dynamics');
hold off;grid on; axis tight;legend show;

% Yaw angle and rate
figure('Name','INSPVA: Yaw','Units','inches','PaperUnits', 'inches','Position', [1 1 pwidth pheight], ...
    'PaperPositionMode','Auto','PaperSize',[pwidth pheight]);
plot(Time_IMU, yaw_rate, 'LineWidth', 2, 'DisplayName', 'r');hold on;
plot(Time_IMU, yaw_imu,'LineWidth', 2, 'DisplayName', '\psi');hold off;
xlabel('Time (sec)'); ylabel('^\circ/s, deg');
title('Yaw Dynamics');
hold off;grid on; axis tight;legend('Location','best');

% Accelerations
figure('Name','INSPVA: Acceleration','Units','inches','PaperUnits', 'inches','Position', [1 1 pwidth pheight], ...
    'PaperPositionMode','Auto','PaperSize',[pwidth pheight]);
subplot(311);
plot(Time_IMU, Xacc,'r', 'LineWidth', 2, 'DisplayName', 'a_x');
grid on; axis tight; legend off;%('Location','best');
xlabel('Time (sec)'); ylabel('m/s^2');
title('a_x');
subplot(312);
plot(Time_IMU, Yacc,'g', 'LineWidth', 2, 'DisplayName', 'a_y');
grid on; axis tight; legend off;%('Location','best');
xlabel('Time (sec)'); ylabel('m/s^2');
title('a_y');
subplot(313);
plot(Time_IMU, Zacc,'b', 'LineWidth', 2, 'DisplayName', 'a_z');
title('a_z');
grid on; axis tight; legend off;%('Location','best');
xlabel('Time (sec)'); ylabel('m/s^2');
xlabel('Time (sec)'); 