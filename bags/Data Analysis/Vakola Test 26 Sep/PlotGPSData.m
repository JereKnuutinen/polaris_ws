%% Plot Extracted Data
pwidth = 3;
pheight = 3;
% Plot GPS Status
figure('Name','BESTPOS: GPS Status','Units','inches','PaperUnits', 'inches','Position', [1 1 pwidth pheight], ...
    'PaperPositionMode','Auto','PaperSize',[pwidth pheight]);
plot(Time_GPS, GPS_Status,'r', 'LineStyle','none', 'Marker','x','LineWidth', 2);
title('Status')
grid on; axis tight;
legend({ strcat("-1 = No Fix,", string(newline),...
    "0 = Fix", string(newline), "2 = Diff. Fix")},'Location','best'); 

% plot differential_age
figure('Name','PSRPOS: Diff. Age','Units','inches','PaperUnits', 'inches','Position', [1 1 pwidth pheight], ...
    'PaperPositionMode','Auto','PaperSize',[pwidth pheight]);
plot(Time_PSR, differential_age,'r', 'LineStyle','none', 'Marker','x','LineWidth', 2);
title('Differential Age')
grid on; axis tight;
% legend("Diff Age",'Location','best'); 

% plot solution age
figure('Name','PSRPOS: Solution Age','Units','inches','PaperUnits', 'inches','Position', [1 1 pwidth pheight], ...
    'PaperPositionMode','Auto','PaperSize',[pwidth pheight]);
plot(Time_PSR, solution_age,'r', 'LineStyle','none', 'Marker','x','LineWidth', 2);
title('Solution Age')
grid on; axis tight;

% plot position type 
figure('Name','PSRPOS: Position Type','Units','inches','PaperUnits', 'inches','Position', [1 1 pwidth pheight], ...
    'PaperPositionMode','Auto','PaperSize',[pwidth pheight]);
plot(Time_PSR, position_type,'r', 'LineStyle','none', 'Marker','x','LineWidth', 2);
title('Position Type')
grid on; axis tight;
legend({ strcat("0 = NONE,", string(newline),...
    "16 = SINGLE", string(newline), "17 = PSRDIFF")},'Location','best');  

% plot number of satellites age
figure('Name','PSRPOS: Satellites','Units','inches','PaperUnits', 'inches','Position', [1 1 pwidth pheight], ...
    'PaperPositionMode','Auto','PaperSize',[pwidth pheight]);
plot(Time_PSR, number_of_satellites,'r', 'LineStyle','none', 'Marker','x','LineWidth', 2);
title('Number of Satellites')
grid on; axis tight;
% legend("Diff Age",'Location','best'); 

% Plot Satellite Status
figure('Name','BESTPOS: Sat. Status','Units','inches','PaperUnits', 'inches','Position', [1 1 pwidth pheight], ...
    'PaperPositionMode','Auto','PaperSize',[pwidth pheight]);
plot(Time_GPS, Sat_Status,'r', 'LineStyle','none', 'Marker','x','LineWidth', 2);
title('Satellite Status')
grid on; axis tight;
legend({ strcat("1 = GPS,", string(newline),...
    "2 = GLONASS", string(newline), "4 = COMPASS/BeiDou",...
    string(newline), "8 = GALILEO") }, 'Location','best');  

% Plot GPS data from BESTPOS
figure('Name','BESTPOS: WGS-84','Units','inches','PaperUnits', 'inches','Position', [1 1 pwidth pheight], ...
    'PaperPositionMode','Auto','PaperSize',[pwidth pheight]);
subplot(211);
plot(long, lat, 'LineWidth', 2 ,'HandleVisibility','off');hold on;
plot(long(1), lat(1), 'kx', 'LineWidth',2, 'DisplayName', 'Start');
plot(long(end), lat(end), 'go', 'LineWidth',2, 'DisplayName', 'End');hold off;
xlabel('Long (deg)');ylabel('Lat (deg)');
title('Vehicle Path')
grid on; axis tight;legend ('Location','best');
subplot(212);
plot(Time_GPS, alt, 'LineWidth', 2, 'DisplayName', 'Altitude');hold on;
xlabel('Time (sec)');ylabel('m');
title('Altitude')
grid on; axis tight;legend off;%('Location','best');

% Plot GPS data from PSRPOS
figure('Name','PSRPOS: WGS-84','Units','inches','PaperUnits', 'inches','Position', [1 1 pwidth pheight], ...
    'PaperPositionMode','Auto','PaperSize',[pwidth pheight]);
subplot(211);
plot(long_PSR, lat_PSR, 'LineWidth', 2 ,'HandleVisibility','off');hold on;
plot(long_PSR(1), lat_PSR(1), 'kx', 'LineWidth',2, 'DisplayName', 'Start');
plot(long_PSR(end), lat_PSR(end), 'go', 'LineWidth',2, 'DisplayName', 'End');hold off;
xlabel('Long (deg)');ylabel('Lat (deg)');
title('Vehicle Path')
grid on; axis tight;legend ('Location','best');
subplot(212);
plot(Time_PSR, alt_PSR, 'LineWidth', 2, 'DisplayName', 'Altitude');hold on;
xlabel('Time (sec)');ylabel('m');
title('Altitude')
grid on; axis tight;legend off;%('Location','best');

% Plot GPS data -- UTM 
figure('Name','BESTPOS: UTM','Units','inches','PaperUnits', 'inches','Position', [1 1 pwidth pheight], ...
    'PaperPositionMode','Auto','PaperSize',[pwidth pheight]);
subplot(211);
plot(UTM_GPS(1,:), UTM_GPS(2,:), 'LineWidth', 2,'HandleVisibility','off');hold on;
plot(UTM_GPS(1,1), UTM_GPS(2,1), 'rx', 'LineWidth',2, 'DisplayName', 'Start');
plot(UTM_GPS(1,end), UTM_GPS(2,end), 'go', 'LineWidth',2, 'DisplayName', 'End');hold off;
xlabel('Easting (m)');ylabel('Northing (m)');title('Vehicle Path')
grid on; axis tight;legend ('Location','best');
subplot(212);
plot(Time_GPS, UTM_GPS(3,:), 'LineWidth', 2, 'DisplayName', 'Altitude');
xlabel('Time (sec)');
ylabel('m');
grid on; axis tight;legend off;%('Location','best');
title('Altitude');

% GPS Track Angle -- copy description from novatel oem manual
figure('Name','BESTVEL: Track Angle','Units','inches','PaperUnits', 'inches','Position', [1 1 pwidth pheight], ...
    'PaperPositionMode','Auto','PaperSize',[pwidth pheight]);
plot(Time_UTM, Track_Angle,'LineWidth', 2, 'DisplayName', 'Track Angle');
xlabel('Time (sec)'); ylabel('deg');
title('Track Angle');
hold off;grid on; axis tight;legend off;%('Location','best');

% Lat/Lon/Alt Position Covariances
figure('Name','BESTPOS Data','Units','inches','PaperUnits', 'inches','Position', [1 1 pwidth pheight], ...
    'PaperPositionMode','Auto','PaperSize',[pwidth pheight]);
plot(Time_GPS, PosCov(1,:),'LineWidth',2,'DisplayName','\sigma^2_{LAT}'); hold on;
plot(Time_GPS, PosCov(2,:),'LineWidth',2,'DisplayName','\sigma^2_{LON}'); 
plot(Time_GPS, PosCov(3,:),'LineWidth',2,'DisplayName','\sigma^2_{ALT}'); 
hold off; grid on; axis tight; legend('Location','best');
title('Position Covariances');