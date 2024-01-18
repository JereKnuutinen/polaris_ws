clear;
close all; clc;

%% Check important flags.
M = 890; % mass of Polaris in Kgs.
g = 9.8; % m/s^2
do_analysis = 1;
show_rightside_loading_data = 1;
show_leftside_loading_data = 1;
plot_fit_data = 1;
save_coeffs = 0;

%% Spring Calibration of Front Right Tire
% Front Right Calibration Table.
M0_FR = 190.0; % default mass for front right.
h0_FR = 69/100.0; % in m
V0_FR = 4577; % in mV

% The weight on the rod slipped towards tires at TV_WSHS = 1745 sec. This
% caused a 1 cm height difference in the measurement tape readings, plus a
% 10 mV shift in voltage readings from that instance onwards. This is
% visually clear from voltage data recorded. This explains why a 10 mV
% of voltage is subtracted and 1 cm displacement is added to
% V_FR values during unloading.

% Loading points
M_FR_load = [M0_FR; 210; 230; 240; 260; 270; 290; 300; 310];
h_FR_load = [h0_FR*100.0; 68; 66.5; 65.5; 64; 63; 61.5; 61; 60];
V_FR_load = [V0_FR; 4523; 4476; 4411; 4344; 4275; 4197; 4175; 4138];

% % Unloading points
% M_FR_unload = [290; 270; 260; 240; 230; 210; M0_FR];
% h_FR_unload = [62; 63; 63.5; 65.5; 66.5; 67.5; 68.5];
% V_FR_unload = [4218; 4281; 4344; 4405; 4468; 4530; 4550];

% % Unloading points
M_FR_unload = [300; 300; 290; 270; 260; 240; 230; 210; M0_FR];
h_FR_unload = [60.5; 61.5; 62; 63; 63.5; 65.5; 66.5; 67.5; 68.5];
V_FR_unload = [4150; 4218; 4218; 4281; 4344; 4405; 4468; 4530; 4550];

% Gather points
h_FR = [h_FR_load; h_FR_unload]/100.0;
V_FR = [V_FR_load; V_FR_unload];
M_FR = [M_FR_load; M_FR_unload];

% H = height over ground measured by tape, where
% H = Kv * V + Cv and V = voltage read by RTP sensor.
% LS Estimates Kv and Cv when H = Kv * V + Cv.
y = h_FR; A = [V_FR ones(length(V_FR), 1)];
K1_FR = (A'*A)^-1*A'*y; % LS Estimate
h_FR_Fit = K1_FR' * A'; h_FR_Fit = h_FR_Fit';
% Fit Check
MSE_K1_FR = mean ( (h_FR_Fit - h_FR).^2 );

% Once Kv and Cv are found by LS fit,
% we have m = Kh*(H - Cv) + Ch, we mt = total mass measured by weighing
% scale with the weights on.
% LS Estimates for K_h and C_h when mt = Kh*(H - Cv) + Ch
y = M_FR; A = [h_FR ones(length(h_FR), 1)];
K2_FR = (A'*A)^-1*A'*y; % LS Estimate
M_FR_Fit = K2_FR(1) * (h_FR) + K2_FR(2);
% Fit Check
MSE_K2_FR = mean ( (M_FR_Fit - M_FR).^2 );
% fit the mass data using fitted height data
A = [h_FR_Fit ones(length(h_FR_Fit), 1)];
K2p_FR = (A'*A)^-1*A'*y; % LS Estimate
Mp_FR_Fit = K2p_FR(1) * (h_FR_Fit) + K2p_FR(2);
% Fit Check
MSE_K2p_FR = mean ( (Mp_FR_Fit - M_FR).^2 );

% The the idea is to find a spring coefficient following Hooke's law,
% where Fs = Ks*x, when x = H - H0 and Fs = mt*g
x_FR = K1_FR(1)*(V0_FR-V_FR);
F_FR = (Mp_FR_Fit-Mp_FR_Fit(1))*g;
Ks_FR = F_FR ./ x_FR;

%% Spring Calibration of Front Left Tire
% Front Left Calibration Table.
M0_FL = 200.0; % default mass for front right.
offset = 0; % in cm
h0_FL = 66.7/100.0; % in m
V0_FL = 993; % in mV

% Loading points
M_FL_load = [M0_FL; 220; 230; 250; 260; 280; 300; 315];
h_FL_load = [h0_FL*100.0; 66.2; 65; 64.5; 63.25; 62.25; 61.25; 60.01];
V_FL_load = [V0_FL; 1018; 1063; 1112; 1171; 1221; 1283; 1335];

% Unloading points
M_FL_unload = [295; 280; 270; 250; 240; 220; 200];
h_FL_unload = [60.25; 60.9; 62; 62.75; 64; 65; 66.5]; % about 1 cm offset
V_FL_unload = [1309; 1238; 1207; 1142; 1089; 1037; 1000];

% Gather points
h_FL = [h_FL_load; h_FL_unload]/100.0;
V_FL = [V_FL_load; V_FL_unload];
M_FL = [M_FL_load; M_FL_unload];

% H = height over ground measured by tape, where
% H = Kv * V + Cv and V = voltage read by RTP sensor.
% LS Estimates Kv and Cv when H = Kv * V + Cv.
y = h_FL; A = [V_FL ones(length(V_FL), 1)];
K1_FL = (A'*A)^-1*A'*y;
h_FL_Fit = K1_FL' * A'; h_FL_Fit = h_FL_Fit';
% Fit Check
MSE_K1_FL = mean ( (h_FL_Fit - h_FL).^2 );

% Once Kv and Cv are found by LS fit,
% we have m = Kh*(H - Cv) + Ch, we mt = total mass measured by weighing
% scale with the weights on.
% LS Estimates for K_h and C_h when mt = Kh*(H - Cv) + Ch.
y = M_FL; A = [h_FL ones(length(h_FL), 1)];
K2_FL = (A'*A)^-1*A'*y; % LS Estimate
M_FL_Fit = K2_FL(1) * (h_FL) + K2_FL(2);
% Fit Check
MSE_K2_FL = mean ( (M_FL_Fit - M_FL).^2 );
% fit the mass data using fitted height data
A = [h_FL_Fit ones(length(h_FL_Fit), 1)];
K2p_FL = (A'*A)^-1*A'*y; % LS Estimate
Mp_FL_Fit = K2p_FL(1) * (h_FL_Fit) + K2p_FL(2);
% Fit Check
MSE_K2p_FL = mean ( (Mp_FL_Fit - M_FL).^2 );

% The the idea is to find a spring coefficient following Hooke's law,
% where Fs = Ks*x, when x = H - H0 and Fs = mt*g
x_FL = K1_FL(1)*(V0_FL-V_FL);
F_FL = (Mp_FL_Fit-Mp_FL_Fit(1))*g;
Ks_FL =  F_FL ./ x_FL;

%% Spring Calibration of Rear Left Tire
% Rear Left Calibration Table.
M0_RL = 260.0; % default mass for front right.
h0_RL = 76/100.0; % in m
V0_RL = 1219; % in mV

% Loading points
M_RL_load = [M0_RL; 275; 295; 310; 335; 345; 360; 380; 395; 410];
h_RL_load = [h0_RL*100.0; 75.5; 74.5; 73.7; 73.0; 72.0; 71.0; 70.0; 69.0; 67.5];
V_RL_load = [V0_RL; 1228.7; 1266.5; 1307.8; 1345.6; 1395; 1444.1; 1491.4; 1544.8; 1615];

% Unloading points
M_RL_unload = [400; 380; 370; 350; 330; 310; 295; 280; 260];
h_RL_unload = [68; 68.5; 69.5; 70.4; 71.0; 72.0; 73.4; 74.0; 75.0]; % about 1 cm offset
V_RL_unload = [1605.5; 1577.2; 1531.6; 1483.1; 1438.1; 1387.2; 1353; 1284.6; 1234.5];

% Gather points
h_RL = [h_RL_load; h_RL_unload]/100.0;
V_RL = [V_RL_load; V_RL_unload];
M_RL = [M_RL_load; M_RL_unload];

% H = height over ground measured by tape, where
% H = Kv * V + Cv and V = voltage read by RTP sensor.
% LS Estimates Kv and Cv when H = Kv * V + Cv.
y = h_RL; A = [V_RL ones(length(V_RL), 1)];
K1_RL = (A'*A)^-1*A'*y; % LS Estimate
h_RL_Fit = K1_RL' * A';h_RL_Fit = h_RL_Fit';
% Fit Check
MSE_K1_RL = mean ( (h_RL_Fit - h_RL).^2 );

% Once Kv and Cv are found by LS fit,
% we have m = Kh*(H - Cv) + Ch, we mt = total mass measured by weighing
% scale with the weights on.
% LS Estimates for K_h and C_h when mt = Kh*H + Ch.
y = M_RL; A = [h_RL ones(length(h_RL), 1)];
K2_RL = (A'*A)^-1*A'*y; % LS Estimate
M_RL_Fit = K2_RL(1) * (h_RL) + K2_RL(2);
% Fit Check
MSE_K2_RL = mean ( (M_RL_Fit - M_RL).^2 );
% fit the mass data using fitted height data
A = [h_RL_Fit ones(length(h_RL_Fit), 1)];
K2p_RL = (A'*A)^-1*A'*y; % LS Estimate
Mp_RL_Fit = K2p_RL(1) * (h_RL_Fit) + K2p_RL(2);
% Fit Check
MSE_K2p_RL = mean ( (Mp_RL_Fit - M_RL).^2 );

% The the idea is to find a spring coefficient following Hooke's law,
% where Fs = Ks*x, when x = H - H0 and Fs = mt*g
x_RL = K1_RL(1)*(V0_RL-V_RL);
F_RL = (Mp_RL_Fit-Mp_RL_Fit(1))*g;
Ks_RL = F_RL ./ x_RL ;

%% Spring Calibration of Rear Right Tire
% Rear Right Calibration Table.
M0_RR = 250.0; % default mass for front right.
h0_RR = 77/100.0; % in m
V0_RR = 4426; % in mV

% The weight on the rod slipped towards tires at TV_WSHS = 1745 sec. This
% caused a 1 cm height difference in the measurement tape readings, plus a
% 10 mV shift in voltage readings from that instance onwards. This is
% visually clear from voltage data recorded. This explains why a 10 mV
% of voltage is subtracted and 1 cm displacement is added to
% V_FR values during unloading.

% Loading points
M_RR_load = [M0_RR; 270; 285; 300; 320; 335; 350; 370; 390; 410];
h_RR_load = [h0_RR*100.0; 76.5; 75.8; 75; 74; 73; 72; 71; 70; 68];
V_RR_load = [V0_RR; 4416; 4382.8; 4337.5; 4281.5; 4228.3; 4170.1; 4121; 4055.8; 3972];

% Unloading points
M_RR_unload = [350; 340; 320; 300; 290; 270; 250];
h_RR_unload = [70; 71; 72; 73; 74; 75; 76]; % about 1 cm offset
V_RR_unload = [4061; 4103; 4126.7; 4212.3; 4270.2; 4322.1; 4379.2];

% Gather points
h_RR = [h_RR_load; h_RR_unload]/100.0;
V_RR = [V_RR_load; V_RR_unload];
M_RR = [M_RR_load; M_RR_unload];

% H = height over ground measured by tape, where
% H = Kv * V + Cv and V = voltage read by RTP sensor.
% LS Estimates Kv and Cv when H = Kv * V + Cv.
y = h_RR; A = [V_RR ones(length(V_RR), 1)];
K1_RR = (A'*A)^-1*A'*y; % LS Estimate
h_RR_Fit = K1_RR' * A'; h_RR_Fit = h_RR_Fit';
% Fit Check
MSE_K1_RR = mean ( (h_RR_Fit - h_RR).^2 );

% Once Kv and Cv are found by LS fit,
% we have m = Kh*(H - Cv) + Ch, we mt = total mass measured by weighing
% scale with the weights on.
% LS Estimates for K_h and C_h when mt = Kh*(H - Cv) + Ch.
y = M_RR; A = [h_RR ones(length(h_RR), 1)];
K2_RR = (A'*A)^-1*A'*y; % LS Estimate
M_RR_Fit = K2_RR(1) * (h_RR) + K2_RR(2);
% Fit Check
MSE_K2_RR = mean ( (M_RR_Fit - M_RR).^2 );
% fit the mass data using fitted height data
A = [h_RR_Fit ones(length(h_RR_Fit), 1)];
K2p_RR = (A'*A)^-1*A'*y; % LS Estimate
Mp_RR_Fit = K2p_RR(1) * (h_RR_Fit) + K2p_RR(2);
% Fit Check
MSE_K2p_RR = mean ( (Mp_RR_Fit - M_RR).^2 );

% The the idea is to find a spring coefficient following Hooke's law,
% where Fs = Ks*x, when x = H - H0 and Fs = mt*g
x_RR = K1_RR(1)*(V0_RR-V_RR);
F_RR = (Mp_RR_Fit-Mp_RR_Fit(1))*g;
Ks_RR =  F_RR./x_RR;

if exist('do_analysis', 'var')
    % Plot Force versus Displacement
    figure;
    subplot(222);
    plot(F_FR, x_FR,'r-s','DisplayName','FR -- Fit');grid on; axis tight;
    hold on;
    plot((M_FR-M0_FR)*g, h0_FR - h_FR,'k', 'DisplayName','Obs.');grid on; axis tight;
    ylabel('x -- (m)'); xlabel('F -- (N)');
    legend ('Location','southeast');
    subplot(221);
    plot(F_FL, x_FL,'g-*','DisplayName','FL -- Fit');grid on; axis tight;
    hold on;
    plot((M_FL-M0_FL)*g, h0_FL - h_FL,'k', 'DisplayName','Obs.');grid on; axis tight;
    ylabel('x -- (m)'); xlabel('F -- (N)');
    legend ('Location','southeast');
    subplot(224);
    plot(F_RR, x_RR,'b-o','DisplayName','RR -- Fit');grid on; axis tight;
    hold on;
    plot((M_RR-M0_RR)*g, h0_RR - h_RR,'k', 'DisplayName','Obs.');grid on; axis tight;
    ylabel('x -- (m)'); xlabel('F -- (N)');
    legend ('Location','southeast');
    subplot(223);
    plot(F_RL, x_RL,'c-d','DisplayName','RL -- Fit');grid on; axis tight;
    hold on;
    plot((M_RL-M0_RL)*g, h0_RL - h_RL,'k', 'DisplayName','Obs.');grid on; axis tight;
    ylabel('x -- (m)'); xlabel('F -- (N)');
    legend ('Location','southeast');
    sgtitle('Applied force and Extension');

    figure;
    plot((M_FR-M0_FR)*g, Ks_FR,'rs','DisplayName','FR');hold on;
    plot((M_FL-M0_FL)*g, Ks_FL,'g*','DisplayName','FL');
    plot((M_RL-M0_RL)*g, Ks_RL,'bo','DisplayName','RL');
    plot((M_RR-M0_RR)*g, Ks_RR,'cd','DisplayName','RR');hold off;
    axis tight; grid on;legend ('Location','best');
    ylabel('N.m^{-1}'); xlabel('Applied Force -- (N)')
    title('K_S = F/\Deltax');
end

% compute mean values of each spring coefficient after removing NaN values
% from the data
Ks_FR = nanmean(Ks_FR);
Ks_FL = nanmean(Ks_FL);
Ks_RL = nanmean(Ks_RL);
Ks_RR = nanmean(Ks_RR);

if save_coeffs
    save("SpringCoeffs.mat", "Ks_RR", "Ks_RL", "Ks_FL", "Ks_FR");
    save("HgtVsVoltCoeffs.mat", "K1_RR", "K1_RL", "K1_FL", "K1_FR");
    save("MassVsHgtCoeffs.mat", "K2_RR", "K2_RL", "K2_FL", "K2_FR");
end

%% Compare original and Linear Fit data
if plot_fit_data
    figure;
    subplot(211);
    plot(V_FR, h_FR,'LineWidth',2,'Marker','+','DisplayName','Data Points');
    hold on;
    plot(V_FR, h_FR_Fit,'LineWidth',2,'Marker','o','DisplayName','Linear Fit');
    hold off;
    xlabel('millivolts'); ylabel('meters');
    title('Front Right -- Voltage versus Displacement')
    axis tight; grid on;legend('Location','best');
    subplot(212);
    plot(h_FR, M_FR, 'LineWidth',2,'Marker','+','DisplayName','Data Points');
    hold on;
    plot(h_FR, M_FR_Fit, 'LineWidth',2,'Marker','o','DisplayName','Linear Fit');
    hold off;
    xlabel('meters'); ylabel('Newtons');
    title('Front Right -- Displacement versus Force')
    axis tight; grid on;legend('Location','best');

    figure;
    subplot(211);
    plot(V_FL, h_FL,'LineWidth',2,'Marker','+','DisplayName','Data Points');
    hold on;
    plot(V_FL, h_FL_Fit,'LineWidth',2,'Marker','o','DisplayName','Linear Fit');
    hold off;
    xlabel('millivolts'); ylabel('meters');
    title('Front Left -- Voltage versus Height')
    axis tight; grid on;legend('Location','best');
    subplot(212);
    plot(h_FL, M_FL , 'LineWidth',2,'Marker','+','DisplayName','Data Points');
    hold on;
    plot(h_FL, M_FL_Fit, 'LineWidth',2,'Marker','o','DisplayName','Linear Fit');
    hold off;
    xlabel('meters'); ylabel('Newtons');
    title('Front Left -- Displacement versus Force')
    axis tight; grid on;legend('Location','best');

    figure;
    subplot(211);
    plot(V_RL, h_RL,'LineWidth',2,'Marker','+','DisplayName','Data Points');
    hold on;
    plot(V_RL, h_RL_Fit,'LineWidth',2,'Marker','o','DisplayName','Linear Fit');
    hold off;
    xlabel('millivolts'); ylabel('meters');
    title('Rear Left -- Voltage versus Height')
    axis tight; grid on;legend('Location','best');
    subplot(212);
    plot(h_RL, M_RL,'LineWidth',2,'Marker','+','DisplayName','Data Points');
    hold on;
    plot(h_RL, M_RL_Fit,'LineWidth',2,'Marker','o','DisplayName','Linear Fit');
    hold off;
    xlabel('meters'); ylabel('Newtons');
    title('Rear Left -- Displacement versus Force')
    axis tight; grid on;legend('Location','best');

    figure;
    subplot(211);
    plot(V_RR, h_RR,'LineWidth',2,'Marker','+','DisplayName','Data Points');
    hold on;
    plot(V_RR, h_RR_Fit,'LineWidth',2,'Marker','o','DisplayName','Linear Fit');
    hold off;
    xlabel('millivolts'); ylabel('centimeters');
    title('Rear Right -- Voltage versus Displacement')
    axis tight; grid on;legend('Location','best');
    subplot(212);
    plot(h_RR, M_RR , 'LineWidth',2,'Marker','+','DisplayName','Data Points');
    hold on;
    plot(h_RR, M_RR_Fit, 'LineWidth',2,'Marker','o','DisplayName','Linear Fit');
    hold off;
    xlabel('centimeters'); ylabel('Newtons');
    title('Rear Right -- Displacement versus Force')
    axis tight; grid on;legend('Location','best');
end

%% Use Coefficients to plot loading and unloading data.
% Two separates bag files were recorded one
% for right set of tires and other for left set of tyres.
if show_rightside_loading_data
    % 2022-06-13-11-34-02.bag contains data while front right and rear right
    % tyres were loaded and unload. Note that front one was loaded and
    % unloaded first.
    filename = './Bags/Calib02.bag' ;
    bagInfo_right = rosbag('info', filename);
    %     rosbag info 2022-06-13-11-34-02.bag;
    % Extract the bag file into a matlab structure.
    bag_rightside_loading = rosbag(filename);

    % 2022-06-13-11-34-02.bag contains data while front left and rear left
    % tyres were loaded and unload. Note that in this test rear one was
    % loaded and unloaded first.
    filename = './Bags/Calib01.bag';
    bagInfo_left = rosbag('info', filename);
    % Extract the bag file into a matlab structure.
    bag_leftside_loading = rosbag(filename);

    % extract only the wheel displacement measurement topic
    bWSHS_rightside_loading = select(bag_rightside_loading,'Topic','/atv_wheel_displacement_measurement');
    TimeWSHS_rightside_loading = bWSHS_rightside_loading.MessageList.Time;
    msgWSHS_rightside_loading = readMessages(bWSHS_rightside_loading,'DataFormat','struct');

    TV_WSHS_rightside_loading = zeros(length(msgWSHS_rightside_loading),1);
    for k = 1 : length(msgWSHS_rightside_loading)
        % Following time conversion is verified by toSec() function in ROS.
        TV_WSHS_rightside_loading(k) = double ( msgWSHS_rightside_loading{k,1}.TimeReceived.Sec ) + ...
            double ( msgWSHS_rightside_loading{k,1}.TimeReceived.Nsec ) * 1e-9;
    end
    TV_WSHS_rightside_loading = TV_WSHS_rightside_loading - TV_WSHS_rightside_loading(1);

    voltage_FL_rightside_loading = zeros(length(msgWSHS_rightside_loading), 1);
    voltage_FR_rightside_loading = zeros(length(msgWSHS_rightside_loading), 1);
    voltage_RL_rightside_loading = zeros(length(msgWSHS_rightside_loading), 1);
    voltage_RR_rightside_loading = zeros(length(msgWSHS_rightside_loading), 1);

    for k = 1 : length(msgWSHS_rightside_loading)
        voltage_FL_rightside_loading(k) = msgWSHS_rightside_loading{k,1}.FrontLeftHeight;
        voltage_FR_rightside_loading(k) = msgWSHS_rightside_loading{k,1}.FrontRightHeight;
        voltage_RL_rightside_loading(k) = msgWSHS_rightside_loading{k,1}.RearLeftHeight;
        voltage_RR_rightside_loading(k) = msgWSHS_rightside_loading{k,1}.RearRightHeight;
    end

    height_FR_rightside_loading = K1_FR(1) * voltage_FR_rightside_loading + K1_FR(2);
    height_FL_rightside_loading = K1_FL(1) * voltage_FL_rightside_loading + K1_FL(2);
    height_RL_rightside_loading = K1_RL(1) * voltage_RL_rightside_loading + K1_RL(2);
    height_RR_rightside_loading = K1_RR(1) * voltage_RR_rightside_loading + K1_RR(2);

    mass_FR_rightside_loading = K2_FR(1) * (height_FR_rightside_loading) + K2_FR(2);
    mass_FL_rightside_loading = K2_FL(1) * (height_FL_rightside_loading) + K2_FL(2);
    mass_RL_rightside_loading = K2_RL(1) * (height_RL_rightside_loading) + K2_RL(2);
    mass_RR_rightside_loading = K2_RR(1) * (height_RR_rightside_loading) + K2_RR(2);

    mass_FR_rightside_loading_fit = K2p_FR(1) * (height_FR_rightside_loading) + K2p_FR(2);
    mass_FL_rightside_loading_fit = K2p_FL(1) * (height_FL_rightside_loading) + K2p_FL(2);
    mass_RL_rightside_loading_fit = K2p_RL(1) * (height_RL_rightside_loading) + K2p_RL(2);
    mass_RR_rightside_loading_fit = K2p_RR(1) * (height_RR_rightside_loading) + K2p_RR(2);

    figure;
    subplot(211);
    plot(TV_WSHS_rightside_loading, mass_FR_rightside_loading, 'DisplayName','Using H data','LineWidth',2);hold on;
    plot(TV_WSHS_rightside_loading, mass_FR_rightside_loading_fit, 'DisplayName','Using H-Fit data','LineWidth',2);hold off;
    grid on; axis tight; xlabel('Time -- (sec)'); ylabel('mass -- (kg)')
    legend ('Location','best');
    subplot(212);
    plot(TV_WSHS_rightside_loading, height_FR_rightside_loading, 'DisplayName','FR -- mV','LineWidth',2);hold on;
    grid on; axis tight; xlabel('Time -- (sec)'); ylabel('height -- (m)')
    legend off;%('Location','best');
    sgtitle('FR Tire Profile -- Rightside Loading Test');

    figure;
    subplot(211);
    plot(TV_WSHS_rightside_loading, mass_FL_rightside_loading, 'DisplayName','Using H data','LineWidth',2);hold on;
    plot(TV_WSHS_rightside_loading, mass_FL_rightside_loading_fit, 'DisplayName','Using H-fit data','LineWidth',2);hold off;
    grid on; axis tight; xlabel('Time -- (sec)'); ylabel('mass -- (kg)')
    legend ('Location','best');
    subplot(212);
    plot(TV_WSHS_rightside_loading, height_FL_rightside_loading, 'DisplayName','RL -- mV','LineWidth',2);hold on;
    grid on; axis tight; xlabel('Time -- (sec)'); ylabel('height -- (m)')
    legend off;%('Location','best');
    sgtitle('FL Tire Profile -- Rightside Loading Test');

    figure;
    subplot(211);
    plot(TV_WSHS_rightside_loading, mass_RL_rightside_loading, 'DisplayName','Using H','LineWidth',2);hold on;
    plot(TV_WSHS_rightside_loading, mass_RL_rightside_loading_fit, 'DisplayName','Using H-fit data','LineWidth',2);hold off;
    grid on; axis tight; xlabel('Time -- (sec)'); ylabel('mass -- (kg)')
    legend ('Location','best');
    subplot(212);
    plot(TV_WSHS_rightside_loading, height_RL_rightside_loading, 'DisplayName','RL -- m','LineWidth',2);hold on;
    grid on; axis tight; xlabel('Time -- (sec)'); ylabel('height -- (m)')
    legend off;%('Location','best');
    sgtitle('RL Tire Profile  -- Rightside Loading Test');

    figure;
    subplot(211);
    plot(TV_WSHS_rightside_loading, mass_RR_rightside_loading, 'DisplayName','Using H data','LineWidth',2);hold on;
    plot(TV_WSHS_rightside_loading, mass_RR_rightside_loading_fit, 'DisplayName','Using H-fit data','LineWidth',2);hold off;
    grid on; axis tight; xlabel('Time -- (sec)'); ylabel('mass -- (kg)')
    legend ('Location','best');
    subplot(212);
    plot(TV_WSHS_rightside_loading, height_RR_rightside_loading, 'DisplayName','RL -- m','LineWidth',2);hold on;
    grid on; axis tight; xlabel('Time -- (sec)'); ylabel('height -- (m)')
    legend off;%('Location','best');
    sgtitle('RR Tire Profile  -- Rightside Loading Test');
end

%% Show data fitting for left side loading calibrations
if show_leftside_loading_data
    % extract only the wheel displacement measurement topic
    bWSHS_leftside_loading = select(bag_leftside_loading,'Topic','/atv_wheel_displacement_measurement');
    TimeWSHS_leftside_loading = bWSHS_leftside_loading.MessageList.Time;
    msgWSHS_leftside_loading = readMessages(bWSHS_leftside_loading,'DataFormat','struct');

    TV_WSHS_leftside_loading = zeros(length(msgWSHS_leftside_loading),1);
    for k = 1 : length(msgWSHS_leftside_loading)
        % Following time conversion is verified by toSec() function in ROS.
        TV_WSHS_leftside_loading(k) = double ( msgWSHS_leftside_loading{k,1}.TimeReceived.Sec ) + ...
            double ( msgWSHS_leftside_loading{k,1}.TimeReceived.Nsec ) * 1e-9;
    end
    TV_WSHS_leftside_loading = TV_WSHS_leftside_loading - TV_WSHS_leftside_loading(1);

    voltage_FL_leftside_loading = zeros(length(msgWSHS_leftside_loading), 1);
    voltage_FR_leftside_loading = zeros(length(msgWSHS_leftside_loading), 1);
    voltage_RL_leftside_loading = zeros(length(msgWSHS_leftside_loading), 1);
    voltage_RR_leftside_loading = zeros(length(msgWSHS_leftside_loading), 1);

    for k = 1 : length(msgWSHS_leftside_loading)
        voltage_FL_leftside_loading(k) = msgWSHS_leftside_loading{k,1}.FrontLeftHeight;
        voltage_FR_leftside_loading(k) = msgWSHS_leftside_loading{k,1}.FrontRightHeight;
        voltage_RL_leftside_loading(k) = msgWSHS_leftside_loading{k,1}.RearLeftHeight;
        voltage_RR_leftside_loading(k) = msgWSHS_leftside_loading{k,1}.RearRightHeight;
    end
    height_FR_leftside_loading = K1_FR(1) * voltage_FR_leftside_loading + K1_FR(2);
    height_FL_leftside_loading = K1_FL(1) * voltage_FL_leftside_loading + K1_FL(2);
    height_RL_leftside_loading = K1_RL(1) * voltage_RL_leftside_loading + K1_RL(2);
    height_RR_leftside_loading = K1_RR(1) * voltage_RR_leftside_loading + K1_RR(2);

    mass_FR_leftside_loading = K2_FR(1) * (height_FR_leftside_loading) + K2_FR(2);
    mass_FL_leftside_loading = K2_FL(1) * (height_FL_leftside_loading) + K2_FL(2);
    mass_RL_leftside_loading = K2_RL(1) * (height_RL_leftside_loading) + K2_RL(2);
    mass_RR_leftside_loading = K2_RR(1) * (height_RR_leftside_loading) + K2_RR(2);

    mass_FR_leftside_loading_fit = K2p_FR(1) * (height_FR_leftside_loading) + K2p_FR(2);
    mass_FL_leftside_loading_fit = K2p_FL(1) * (height_FL_leftside_loading) + K2p_FL(2);
    mass_RL_leftside_loading_fit = K2p_RL(1) * (height_RL_leftside_loading) + K2p_RL(2);
    mass_RR_leftside_loading_fit = K2p_RR(1) * (height_RR_leftside_loading) + K2p_RR(2);

    figure;
    subplot(211);
    plot(TV_WSHS_leftside_loading, mass_FR_leftside_loading, 'DisplayName','Using H data','LineWidth',2);hold on;
    plot(TV_WSHS_leftside_loading, mass_FR_leftside_loading_fit, 'DisplayName','Using H-fit data','LineWidth',2);hold off;
    grid on; axis tight; xlabel('Time -- (sec)'); ylabel('mass -- (kg)')
    legend ('Location','best');
    subplot(212);
    plot(TV_WSHS_leftside_loading, height_FR_leftside_loading, 'DisplayName','FR -- mV','LineWidth',2);hold on;
    grid on; axis tight; xlabel('Time -- (sec)'); ylabel('height -- (m)')
    legend off;%('Location','best');
    sgtitle('FR Tire Profile -- Leftside Loading Test');

    figure;
    subplot(211);
    plot(TV_WSHS_leftside_loading, mass_FL_leftside_loading, 'DisplayName','Using H data','LineWidth',2);hold on;
    plot(TV_WSHS_leftside_loading, mass_FL_leftside_loading_fit, 'DisplayName','Using H-fit data','LineWidth',2);hold off;
    grid on; axis tight; xlabel('Time -- (sec)'); ylabel('mass -- (kg)')
    legend ('Location','best');
    subplot(212);
    plot(TV_WSHS_leftside_loading, height_FL_leftside_loading, 'DisplayName','RL -- mV','LineWidth',2);hold on;
    grid on; axis tight; xlabel('Time -- (sec)'); ylabel('height -- (m)')
    legend off;%('Location','best');
    sgtitle('FL Tire Profile -- Leftside Loading Test');

    figure;
    subplot(211);
    plot(TV_WSHS_leftside_loading, mass_RL_leftside_loading, 'DisplayName','Using H data','LineWidth',2);hold on;
    plot(TV_WSHS_leftside_loading, mass_RL_leftside_loading_fit, 'DisplayName','Using H-fit data','LineWidth',2);hold off;
    grid on; axis tight; xlabel('Time -- (sec)'); ylabel('mass -- (kg)')
    legend ('Location','best');
    subplot(212);
    plot(TV_WSHS_leftside_loading, height_RL_leftside_loading, 'DisplayName','RL -- m','LineWidth',2);hold on;
    grid on; axis tight; xlabel('Time -- (sec)'); ylabel('height -- (m)')
    legend off;%('Location','best');
    sgtitle('RL Tire Profile  -- Leftside Loading Test');

    figure;
    subplot(211);
    plot(TV_WSHS_leftside_loading, mass_RR_leftside_loading, 'DisplayName','Using H data','LineWidth',2);hold on;
    plot(TV_WSHS_leftside_loading, mass_RR_leftside_loading_fit, 'DisplayName','Using H-fit data','LineWidth',2);hold on;
    grid on; axis tight; xlabel('Time -- (sec)'); ylabel('mass -- (kg)')
    legend off;%('Location','best');
    subplot(212);
    plot(TV_WSHS_leftside_loading, height_RR_leftside_loading, 'DisplayName','RL -- m','LineWidth',2);hold on;
    grid on; axis tight; xlabel('Time -- (sec)'); ylabel('height -- (m)')
    legend off;%('Location','best');
    sgtitle('RR Tire Profile  -- Leftside Loading Test');
end
