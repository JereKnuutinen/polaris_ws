%% Plot WODO Data -- wheel odometry measurements
pwidth = 3;
pheight = 3;
figure('Units','inches','PaperUnits', 'inches','Position', [1 1 pwidth pheight], ...
    'PaperPositionMode','Auto','PaperSize',[pwidth pheight]);
subplot(211);
plot(Time_WODO, -speed_FL, 'DisplayName','FL','LineWidth',2); hold on;
plot(Time_WODO, speed_FR, 'DisplayName','FR','LineWidth',2); 
plot(Time_WODO, -speed_RL, 'DisplayName','RL','LineWidth',2); 
plot(Time_WODO, speed_RR, 'DisplayName','RR','LineWidth',2);hold off;
axis tight; grid on; legend('Location','best');xlabel('Time -- s'); ylabel('Absolute Units')
title('Wheel Encoder Values');
subplot(212)
hold on;
% plot(Time_WODO, wheel_ground_speed_rear, 'DisplayName','Rear Wheels Speed','LineWidth',2);
% plot(Time_WODO, wheel_ground_speed_front, 'DisplayName','Front Wheels Speed','LineWidth',2);
% plot(Time_WODO, wheel_ground_speed_left, 'DisplayName','Left Wheels Speed','LineWidth',2);
% plot(Time_WODO, wheel_ground_speed_right, 'DisplayName','Right Wheels Speed','LineWidth',2);
plot(Time_WODO, wheel_ground_speed, 'DisplayName','V_{ODOM}','LineWidth',2);
plot(Time_Odom, GSpeed, 'DisplayName','V_{INS}','LineWidth',2);
plot(Time_UTM, XLinVel_utm, 'DisplayName','V_{GPS}','LineWidth',2);hold off;
axis tight; grid on; legend('Location','best'); xlabel('Time -- s'); ylabel('m/s')
title('Wheel Speed Calibration');

%% PLot SODO Data -- steering measurements
figure('Units','inches','PaperUnits', 'inches','Position', [1 1 pwidth pheight], ...
    'PaperPositionMode','Auto','PaperSize',[pwidth pheight]);
subplot(211);
plot(Time_SODO, steering_encode_position, 'LineWidth',2);
axis tight; grid on; xlabel('Time -- s'); ylabel('Absolute Units')
title('Steering Encoder Values');
subplot(212);
plot(Time_SODO, stg_enc, 'LineWidth',2);
axis tight; grid on; xlabel('Time -- s'); ylabel('degrees')
title('Steering Angle');