close all; clear; clc;
save_coefficients = 0;
filename = 'ACT_TEST_02.bag';
bagInfo = rosbag('info',filename);
bag = rosbag(filename);

DecodeEPECData;

min_time = min([TimeGASC(1) TimeGASP(1) TimeSODO(1) TimeWODO(1) TimeMSS(1)]);
start_time_SODO = TimeSODO(1) - min_time;
start_time_WODO = TimeWODO(1) - min_time;
start_time_GASP = TimeGASP(1) - min_time;
start_time_MSS  = TimeMSS(1)  - min_time;
start_time_GASC = TimeGASC(1) - min_time;
Time_SODO = linspace(start_time_SODO, LogTime_SODO, length(msgSODO));
Time_WODO = linspace(start_time_WODO, LogTime_WODO, length(msgWODO));
Time_GASP = linspace(start_time_GASP, LogTime_GASP, length(msgGASP));
Time_MSS  = linspace(start_time_MSS, LogTime_MSS, length(msgMSS));
Time_GASC = linspace(start_time_GASC, LogTime_GASC, length(msgGASC));

%% compute steering angle conversion coefficients
% the order of the values below must not be changed
Enc_Val = [485621; 477331; 468920]; % EPEC uses these values
Stg_Val = [-31.53; 0.0; 31.68]; % ROS must use these values
Turn_Ref_Val = [64536; 32768; 1000]; % for ROS -> Turn_Ref_Val -> EPEC
% DO NOT change the order of the min -> max values above

y = Stg_Val; A = [Enc_Val ones(length(Enc_Val), 1)];
K_T = (A'*A)^-1*A'*y; % LS Estimate

y = Turn_Ref_Val; A_trn = [Enc_Val ones(length(Enc_Val), 1)];
K_trn = (A_trn'*A_trn)^-1*A_trn'*y; % LS Estimate
if save_coefficients
    save('StrEncConvCoeff.mat', "K_T");
    save('EncTrnRadConvCoeff.mat', 'K_trn');
end
stg_enc = K_T(1) * steering_encode_position + K_T(2);
stg_cmd = K_trn(1) * steering_encode_position + K_trn(2);

%% Plot important results
pwidth = 3;
pheight = 3;
figure('Units','inches','PaperUnits', 'inches','Position', [1 1 pwidth pheight], ...
    'PaperPositionMode','Auto','PaperSize',[pwidth pheight]);
subplot(311)
plot(Time_SODO, steering_encode_position,'b','LineWidth',2,'DisplayName','\delta_{enc,str}');
axis tight; xlabel('Time - s'); ylabel('Absoulte Units')
grid on;
title('Steering Encoder Value');
subplot(313);
plot(Time_SODO, stg_enc, 'b','LineWidth',2,'DisplayName','\delta_{str}'); 
hold on;
axis tight; xlabel('Time - s'); ylabel('Steering Angle -- deg'); 
grid on; legend('Location','best');
title('Steering Angle')
subplot(312);
plot(Time_SODO, stg_cmd, 'b','LineWidth',2,'DisplayName','\delta_{cmd,str}'); 
hold on;
axis tight; xlabel('Time - s'); ylabel('Steering Angle -- deg'); 
grid on; legend('Location','best');
title('Turn Radius Reference');

figure('Units','inches','PaperUnits', 'inches','Position', [1 1 pwidth pheight], ...
    'PaperPositionMode','Auto','PaperSize',[pwidth pheight]);
subplot(211);
plot(Time_WODO, speed_RL, 'b', 'LineWidth',2,'DisplayName','\delta_{ODO,RL}'); 
hold on;
plot(Time_WODO, speed_RR, 'r','LineWidth',2,'DisplayName','\delta_{ODO,RR}'); 
plot(Time_MSS, machine_selected_speed,'g--','LineWidth',2,'DisplayName','\delta_{ODO,MSS}'); 
axis tight; xlabel('Time - s'); ylabel('Absolute Units'); 
grid on; legend('Location','best','Orientation','horizontal');
title('Rear Wheel Encoder Values')
subplot(212);
plot(Time_WODO, wheel_ground_speed,'b','LineWidth',2,'DisplayName','V_{ODO}'); 
hold on;
axis tight; xlabel('Time - s'); ylabel('Wheel Speed -- m/s'); 
grid on; legend('Location','best');
title('Rear Wheel Speed')

figure('Units','inches','PaperUnits', 'inches','Position', [1 1 pwidth pheight], ...
    'PaperPositionMode','Auto','PaperSize',[pwidth pheight]);
% subplot(211);
plot(Time_GASP, pwm_ratio1, 'b', 'LineWidth',2,'DisplayName','\delta_{PWM,Ch(12)}'); 
hold on;
plot(Time_GASP, pwm_ratio2, 'r','LineWidth',2,'DisplayName','\delta_{PWM,Ch(34)}'); 
axis tight; xlabel('Time - s'); ylabel('Absolute Units'); 
grid on; legend('Location','best');
title('PWM Signal Values');

figure('Units','inches','PaperUnits', 'inches','Position', [1 1 pwidth pheight], ...
    'PaperPositionMode','Auto','PaperSize',[pwidth pheight]);
subplot(211);
plot(Time_GASC, motion_control_ref, 'LineWidth',2,'DisplayName','\delta_{v}'); 
hold on;
axis tight; xlabel('Time - s'); ylabel('Speed Reference -- mm/s'); 
grid on; legend('Location','best');
title('Motor Control Reference')
subplot(212);
plot(Time_GASC, turn_radius_ref, 'LineWidth',2,'DisplayName','\delta_{str}'); 
hold on;
axis tight; xlabel('Time - s'); ylabel('Turn Radius Reference'); 
grid on; legend('Location','best');
title('Turn Radius Reference')