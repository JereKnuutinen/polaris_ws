%% Plot WSHS -- wheel displacement measurements
figure('Name','WSHS: Heights','Units','inches','PaperUnits', 'inches','Position', [1 1 pwidth pheight], ...
    'PaperPositionMode','Auto','PaperSize',[pwidth pheight]);
subplot(221);
plot(Time_WSHS, height_FL, 'LineWidth',2);hold on;
grid on; axis tight; xlabel('sec'); ylabel('m')
legend off;%('Location','best');
title('FL: Height');
subplot(222);
plot(Time_WSHS, height_FR,'r', 'LineWidth',2);hold on;
grid on; axis tight; xlabel('sec'); ylabel('m')
legend off;%('Location','best');
title('FR: Height');
subplot(223);
plot(Time_WSHS, height_RL,'g', 'LineWidth',2);hold on;
grid on; axis tight; xlabel('sec'); ylabel('m')
legend off;%('Location','best');
title('RL: Height');
subplot(224);
plot(Time_WSHS, height_RR,'c','LineWidth',2);hold on;
grid on; axis tight; xlabel('sec'); ylabel('m')
legend off;%('Location','best');
title('RR: Height');

figure('Name','WSHS: Displacements','Units','inches','PaperUnits', 'inches','Position', [1 1 pwidth pheight], ...
    'PaperPositionMode','Auto','PaperSize',[pwidth pheight]);
subplot(221);
plot(Time_WSHS, x_FL, 'LineWidth',2);hold on;
grid on; axis tight; xlabel('sec'); ylabel('m')
legend off;%('Location','best');
title('FL: \Deltax');
subplot(222);
plot(Time_WSHS, x_FR,'r', 'LineWidth',2);hold on;
grid on; axis tight; xlabel('sec'); ylabel('m')
legend off;%('Location','best');
title('FR: \Deltax');
subplot(223);
plot(Time_WSHS, x_RL,'g', 'LineWidth',2);hold on;
grid on; axis tight; xlabel('sec'); ylabel('m')
legend off;%('Location','best');
title('RL: \Deltax');
subplot(224);
plot(Time_WSHS, x_RR,'c','LineWidth',2);hold on;
grid on; axis tight; xlabel('sec'); ylabel('m')
legend off;%('Location','best');
title('RR: \Deltax');

figure('Name','WSHS: Loads','Units','inches','PaperUnits', 'inches','Position', [1 1 pwidth pheight], ...
    'PaperPositionMode','Auto','PaperSize',[pwidth pheight]);
subplot(221);
plot(Time_WSHS, W_FL./g, 'LineWidth',2);hold on;
grid on; axis tight; xlabel('sec'); ylabel('Newton')
legend off;%('Location','best');
title('FL: Load');
subplot(222);
plot(Time_WSHS, W_FR./g,'r', 'LineWidth',2);hold on;
grid on; axis tight; xlabel('sec'); ylabel('Newton')
legend off;%('Location','best');
title('FR: Load');
subplot(223);
plot(Time_WSHS, W_RL./g,'g', 'LineWidth',2);hold on;
grid on; axis tight; xlabel('sec'); ylabel('Newton')
legend off;%('Location','best');
title('RL: Load');
subplot(224);
plot(Time_WSHS, W_RR./g,'c','LineWidth',2);hold on;
grid on; axis tight; xlabel('sec'); ylabel('Newton')
legend off;%('Location','best');
title('RR: Load');
