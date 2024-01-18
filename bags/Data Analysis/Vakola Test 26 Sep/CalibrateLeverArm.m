% Important flag to do steering encoder calibration
close all; clear; clc;
lever_arm_calib_test = 1;
positive_offsets = 0;
ExtractingData;

Rz = @(yaw)[cos(yaw) sin(yaw);-sin(yaw) cos(yaw)];
%% To illustrate positioning w.r.t gnss (antenna) and ins (span) center
figure('Units','inches','PaperUnits', 'inches','Position', [1 1 pwidth pheight], ...
    'PaperPositionMode','Auto','PaperSize',[pwidth pheight]);
subplot(211);
plot(Time_UTM, ENU_utm(1,:),'LineWidth', 2, 'DisplayName', 'Easting -- GNSS');hold on;
plot(Time_INS, ENU(1,:),'LineWidth', 2, 'DisplayName', 'Easting -- INS');hold off;
grid on; axis tight; legend('Location','best');
subplot(212);
plot(Time_UTM, ENU_utm(2,:),'LineWidth', 2, 'DisplayName', 'Northing -- GNSS');hold on;
plot(Time_INS, ENU(2,:),'LineWidth', 2, 'DisplayName', 'Northing -- INS');hold off;
grid on; axis tight; legend('Location','best');

if positive_offsets == 0
    % extract marked position for negative offset test
    gnss_init_east   = lever_arm_neg_off(8).Position(2);
    gnss_init_north  = lever_arm_neg_off(3).Position(2);
    gnss_final_east  = lever_arm_neg_off(5).Position(2);
    gnss_final_north = lever_arm_neg_off(2).Position(2);

    ins_init_east   = lever_arm_neg_off(7).Position(2);
    ins_init_north  = lever_arm_neg_off(4).Position(2);
    ins_final_east  = lever_arm_neg_off(6).Position(2);
    ins_final_north = lever_arm_neg_off(1).Position(2);
    
    yaw_P1 = deg2rad(81);
    yaw_P2 = deg2rad(-97);
else
    % point out the initial and final points of INS and GNSS (antenna)
    % for positive offset test
    gnss_init_east   = lever_arm_pos_off(8).Position(2);
    gnss_init_north  = lever_arm_pos_off(4).Position(2);
    gnss_final_east  = lever_arm_pos_off(7).Position(2);
    gnss_final_north = lever_arm_pos_off(1).Position(2);

    ins_init_east   = lever_arm_pos_off(6).Position(2);
    ins_init_north  = lever_arm_pos_off(2).Position(2);
    ins_final_east  = lever_arm_pos_off(5).Position(2);
    ins_final_north = lever_arm_pos_off(3).Position(2);

    yaw_P1 = deg2rad(-103);
    yaw_P2 = deg2rad(79);
end
gnss_init  = [gnss_init_east, gnss_init_north];
gnss_final = [gnss_final_east, gnss_final_north];
ins_init   = [ins_init_east, ins_init_north];
ins_final  = [ins_final_east, ins_final_north];

gnss_mid_point = (gnss_final + gnss_init)*0.5;
ins_mid_point = (ins_final + ins_init)*0.5;

dist_init = norm(gnss_init - ins_init);
dist_final = norm(gnss_final - ins_final);

dist_diff = abs(dist_init - dist_final);
delta_mid_points = norm(gnss_mid_point - ins_mid_point);

%% Plot results
figure('Units','inches','PaperUnits', 'inches','Position', [1 1 pwidth pheight], ...
    'PaperPositionMode','Auto','PaperSize',[pwidth pheight]);
hold on;
plot([gnss_init(1) gnss_final(1)] ,[gnss_init(2) gnss_final(2)],'s-','linewidth',2,'DisplayName','GNSS Points');
plot(gnss_mid_point(1),gnss_mid_point(2),'s-','linewidth',2,'DisplayName','GNSS Mid-point');
plot([ins_init(1) ins_final(1)],[ins_init(2) ins_final(2)],'d-','linewidth',2,'DisplayName','INS Points');
plot(ins_mid_point(1),ins_mid_point(2),'d-','linewidth',2,'DisplayName','INS Mid-point');
hold off; grid on; axis tight;
legend('Location','best');
str = sprintf('Mid-points at %c = %0.3f m', char(916), delta_mid_points);
title(str);

% Roll and Pitch Angle -- Correct roll and pitch angle is in RPY angle information
figure('Units','inches','PaperUnits', 'inches','Position', [1 1 pwidth pheight], ...
    'PaperPositionMode','Auto','PaperSize',[pwidth pheight]);
subplot(311);
plot(Time_INS, roll_INS, 'LineWidth', 2, 'DisplayName', 'Roll (\phi)');
grid on; axis tight; legend('Location','best');
xlabel('Time (sec)'); ylabel('degrees');
subplot(312);
plot(Time_INS, pitch_INS, 'LineWidth', 2, 'DisplayName', 'Pitch (\theta)');
grid on; axis tight; legend('Location','best');
xlabel('Time (sec)'); ylabel('degrees');
sgtitle('Novatel Intergrated Data','Fontsize',12);
subplot(313)
plot(Time_INS, rad2deg( eul_INS(:,1) ),'LineWidth', 2, 'DisplayName', 'Heading (\psi)');
xlabel('Time (sec)'); ylabel('degrees');
title('Heading');
hold off;grid on; axis tight;legend('Location','best');

% Measure the actual distance from mid-point and
% rotate the init and final points with respect to the mid-point.
% Rotation matrix around Z-axis and its derivatives


GNSS_P1 = (gnss_init - ins_mid_point)';
GNSS_P2 = (gnss_final - ins_mid_point)';
% yaw_gnss_p1 = atan2(GNSS_P1(2),GNSS_P1(1));
% yaw_gnss_p2 = atan2(GNSS_P2(2),GNSS_P2(1));
INS_P1 = (ins_init - ins_mid_point)';
INS_P2 = (ins_final - ins_mid_point)';
% yaw_ins_p1 = atan2(INS_P1(2),INS_P1(1));
% yaw_ins_p2 = atan2(INS_P2(2),INS_P2(1));
GNSS_MID = gnss_mid_point - ins_mid_point;
INS_MID = ins_mid_point - ins_mid_point;

figure('Units','inches','PaperUnits', 'inches','Position', [1 1 pwidth pheight], ...
    'PaperPositionMode','Auto','PaperSize',[pwidth pheight]);
hold on;
plot([GNSS_P1(1) GNSS_P2(1)] ,[GNSS_P1(2) GNSS_P2(2)],'s-','linewidth',2,'DisplayName','GNSS Points');
plot(GNSS_MID(1),GNSS_MID(2),'s-','linewidth',2,'DisplayName','GNSS Mid-point');
plot([INS_P1(1) INS_P2(1)],[INS_P1(2) INS_P2(2)],'d-','linewidth',2,'DisplayName','INS Points');
plot(INS_MID(1),INS_MID(2),'d-','linewidth',2,'DisplayName','INS Mid-point');
hold off; grid on; axis tight;
legend('Location','best');
str = sprintf('Distances with respect to mid-point');
title(str);

GNSS_P1 = Rz(yaw_P1)*GNSS_P1;
GNSS_P2 = Rz(yaw_P2)*GNSS_P2;

INS_P1 = Rz(yaw_P1)*INS_P1;
INS_P2 = Rz(yaw_P2)*INS_P2;

OFFSET_P1 = INS_P1 - GNSS_P1;
OFFSET_P2 = INS_P2 - GNSS_P2;

AVG_OFFSET = 0.5*(OFFSET_P1 + OFFSET_P2);

figure('Units','inches','PaperUnits', 'inches','Position', [1 1 pwidth pheight], ...
    'PaperPositionMode','Auto','PaperSize',[pwidth pheight]);
hold on;
plot([GNSS_P1(1) GNSS_P2(1)] ,[GNSS_P1(2) GNSS_P2(2)],'s-','linewidth',2,'DisplayName','GNSS Points');
plot(GNSS_MID(1),GNSS_MID(2),'s-','linewidth',2,'DisplayName','GNSS Mid-point');
plot([INS_P1(1) INS_P2(1)],[INS_P1(2) INS_P2(2)],'d-','linewidth',2,'DisplayName','INS Points');
plot(INS_MID(1),INS_MID(2),'d-','linewidth',2,'DisplayName','INS Mid-point');
hold off; grid on; axis tight;
legend('Location','best');
str = sprintf('INS-GNSS Offset: x = %0.3f m, y = %0.3f m', AVG_OFFSET(1), AVG_OFFSET(2));
title(str);
