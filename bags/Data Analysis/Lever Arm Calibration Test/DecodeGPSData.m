%% Extract Data from UTMHandler routine of Novatel Node (ROS).
% /novatel/odom_gps contains position information in UTM coordinates
% computed by the GNSS solution inside Novatel IGM. Other important information
% inlcudes track over ground (zero northwards and positive clockwise 
% in range 0 to 360 degrees), and horizontal and vertical GPS speeds.
bUTM = select(bag,'Topic','/novatel/odom_gps'); 
TimeUTM = bUTM.MessageList.Time;
msgUTM = readMessages(bUTM,'DataFormat','struct');

TV_UTM = zeros(length(msgUTM),1);
for k = 1 : length(msgUTM)
    % Following time conversion is verified by toSec() function in ROS.
    TV_UTM(k) = double ( msgUTM{k,1}.Header.Stamp.Sec ) + ...
        double ( msgUTM{k,1}.Header.Stamp.Nsec ) * 1e-9;
end
% compute log time for /novatel/odom_gps topic
LogTime_UTM = TimeUTM(end) - TimeUTM(1);
% compute sampling frequency from data
Fs_UTM = length(msgUTM)/LogTime_UTM;

pos_utm = cellfun(@(m) struct(m.Pose),msgUTM);
twist_utm = cellfun(@(m) struct(m.Twist),msgUTM);

Xpos_utm = zeros(1,length(pos_utm));
Ypos_utm = Xpos_utm;Zpos_utm = Xpos_utm;
XangVel_utm = Xpos_utm;YangVel_utm = Xpos_utm;ZangVel_utm = Xpos_utm;
XLinVel_utm = Xpos_utm;YLinVel_utm = Xpos_utm;ZLinVel_utm = Xpos_utm;
q0_utm=Xpos_utm; q1_utm=Xpos_utm; q2_utm=Xpos_utm; q3_utm=Xpos_utm;
Track_angle = Xpos_utm;
for k = 1:length(pos_utm)
    Xpos_utm(k) = pos_utm(k).Pose.Position.X; % easting position
    Ypos_utm(k) = pos_utm(k).Pose.Position.Y; % northing position
    Zpos_utm(k) = pos_utm(k).Pose.Position.Z; % height

    % Following three fields are not assigned in the code.
    XangVel_utm(k) = twist_utm(k).Twist.Angular.X;
    YangVel_utm(k) = twist_utm(k).Twist.Angular.Y;
    ZangVel_utm(k) = twist_utm(k).Twist.Angular.Z;

    XLinVel_utm(k) = twist_utm(k).Twist.Linear.X; % horizontal speed
    YLinVel_utm(k) = twist_utm(k).Twist.Linear.Y; % sideways speed
    ZLinVel_utm(k) = twist_utm(k).Twist.Linear.Z; % vertical speed

    q0_utm(k) = pos_utm(k).Pose.Orientation.X;
    q1_utm(k) = pos_utm(k).Pose.Orientation.Y;
    q2_utm(k) = pos_utm(k).Pose.Orientation.Z;
    q3_utm(k) = pos_utm(k).Pose.Orientation.W;
end
% Extract RPY angles from quarternion -- do not change the sequence
eul_utm = quat2eul([q3_utm', q0_utm', q1_utm', q3_utm'], 'ZYX');
track = rad2deg( eul_utm(:,1) );
% for i = 1 : length(Track_angle)
%     track_utm(i) = track2yaw(+90.0 - Track_angle(i));
% end

% pack the position coordinates in UTM matrix
UTM_GPS = [Xpos_utm; Ypos_utm; Zpos_utm];

% Extract diagonal covariance values from the BESTUTM data
PosCov = zeros(3,length(msgUTM));
for k = 1 : length(msgUTM)
    % extract diagonal values corresponding to BESTUTM lat, lon, alt
    % squared standard deviations.
    PosCov(:,k) = msgUTM{k,1}.Pose.Covariance([1 8 15]);
end

% /novatel/gps_fix_psr topic contains the LLA coordinates which are
% obtained obtained purely from gps. Other important information
% includes GNSS status, satellite types, covariance type, position
% covariances, number of satellites, differential age, etc.
bGPS = select(bag,'Topic','/novatel/gps_fix_psr');
TimeGPS = bGPS.MessageList.Time;
msgGPS = readMessages(bGPS,'DataFormat','struct');

TV_GPS = zeros(length(msgGPS),1);
for k = 1 : length(msgGPS)
    % Following time conversion is verified by toSec() function in ROS.
    TV_GPS(k) = double ( msgGPS{k,1}.Header.Stamp.Sec ) + ...
        double ( msgGPS{k,1}.Header.Stamp.Nsec ) * 1e-9;
end
% compute log time for /novatel/gps/fix topic
LogTime_GPS = TimeGPS(end) - TimeGPS(1);
% compute sampling frequency from data
Fs_GPS = length(msgGPS)/LogTime_GPS;

% Extract other data from /novate/gps_fix_utm topic
lat = cellfun(@(m) double(m.Latitude),msgGPS);
long = cellfun(@(m) double(m.Longitude),msgGPS);
alt = cellfun(@(m) double(m.Altitude),msgGPS);

% Covariance type defined in the NavSatFix message in ROS has four values
% 0 = COVARIANCE_TYPE_UNKNOWN
% 1 = COVARIANCE_TYPE_APPROXIMATED
% 2 = COVARIANCE_TYPE_DIAGONAL_KNOWN
% 3 = COVARIANCE_TYPE_KNOWN=3
CovType = cellfun(@(m) double(m.PositionCovarianceType),msgGPS);
% Extract values from the covariance field in PSRPOS data
TempPosCov = zeros(4,length(msgGPS));
for k = 1 : length(msgGPS)
    % extract diagonal values corresponding to differential age, 
    % solution age, position type and number of satellites.
    TempPosCov(:,k) = msgGPS{k,1}.PositionCovariance([2 3 4 6]);
end

% extract important information.
differential_age = TempPosCov(1,:);
solution_age = TempPosCov(2,:);
position_type = TempPosCov(3,:);
number_of_satellites = TempPosCov(4,:);

% Extract GPS Status and reset the messages by omitting invalid GPS data.
GPS_Status = zeros(length(msgGPS),1);
for k = 1 : length(msgGPS)
    GPS_Status(k) = msgGPS{k,1}.Status.Status;
end

% Find the first entry where GPS was locked. (if GPS was not locked at start)
% Note that the NavSatFix message in ROS has four values for GPS
% Status:
% -1 = unable to fix position, 
%  0 = unaugmented fix, 
%  1 = fix with satellite augmentation,
%  2 = with ground-based augmentation.
index_fail =  find(GPS_Status == -1);
index =  find(GPS_Status == 0 | GPS_Status == 1);
index_diff = find(GPS_Status == 2);

% Extract Satellite Status from GPS.
% Sat_Status defined in the NavSatFix message in ROS has four values
% 1 = SERVICE_GPS     
% 2 = SERVICE_GLONASS 
% 4 = SERVICE_COMPASS/BeiDou.
% 8 = SERVICE_GALILEO 
Sat_Status = zeros(length(msgGPS),1);
for k = 1 : length(msgGPS)
    Sat_Status(k) = msgGPS{k,1}.Status.Service;
end

% GPS Data in Local ENU coordinates
[ENU_GPS, ECEF_GPS] = lla2enu(lat, long, alt);
