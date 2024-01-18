%% Extract data from INSPVAHandler of Novatel node (ROS).
% This topic is published in InsPvaHandler file in ROS. It converts
% the LLA data to UTM format along with recording the ENU velocities and
% RPY angles after converting to quarternions.
bINS = select(bag,'Topic','/novatel/odom_map');
TimeINS = bINS.MessageList.Time;
msgINS = readMessages(bINS,'DataFormat','struct');

TV_INS = zeros(length(msgINS),1);
for k = 1 : length(msgINS)
    % Following time conversion is verified by toSec() function in ROS.
    TV_INS(k) = double ( msgINS{k,1}.Header.Stamp.Sec ) + ...
        double ( msgINS{k,1}.Header.Stamp.Nsec ) * 1e-9;
end
% compute log time for /novatel/odom_gps topic
LogTime_INS = TimeINS(end) - TimeINS(1);
% compute sampling frequency from data
Fs_INS = length(msgINS)/LogTime_INS;

pos = cellfun(@(m) struct(m.Pose),msgINS);
twist = cellfun(@(m) struct(m.Twist),msgINS);

Xpos = zeros(1,length(pos));
Ypos = Xpos;Zpos = Xpos;
XangVel = Xpos;YangVel = Xpos;ZangVel = Xpos;
XLinVel = Xpos;YLinVel = Xpos;ZLinVel = Xpos;
q0=Xpos; q1=Xpos; q2=Xpos; q3=Xpos;

for k = 1:length(pos)
    Xpos(k) = pos(k).Pose.Position.X; % easting position
    Ypos(k) = pos(k).Pose.Position.Y; % northing position
    Zpos(k) = pos(k).Pose.Position.Z; % height

    % Following three fields are not assigned in the code.
    XangVel(k) = twist(k).Twist.Angular.X;
    YangVel(k) = twist(k).Twist.Angular.Y;
    ZangVel(k) = twist(k).Twist.Angular.Z;

    XLinVel(k) = twist(k).Twist.Linear.X; % east velocity
    YLinVel(k) = twist(k).Twist.Linear.Y; % north velocity
    ZLinVel(k) = twist(k).Twist.Linear.Z; % up velocity

    q0(k) = pos(k).Pose.Orientation.X;
    q1(k) = pos(k).Pose.Orientation.Y;
    q2(k) = pos(k).Pose.Orientation.Z;
    q3(k) = pos(k).Pose.Orientation.W;
end
% Extract RPY angles from quarternion -- do not change the sequence
eul_INS = quat2eul([q3', q0', q1', q2'], 'ZYX');
roll_INS = rad2deg( eul_INS(:,3) );
pitch_INS = rad2deg( eul_INS(:,2) );
% Inertial Azimuth (zero East +ve CCW, -pi to pi)
yaw_INS = rad2deg( eul_INS(:,1) ); 

% Extract position covariance values from the INSPVA data
Pos_Cov_PVA = zeros(9,length(msgINS));
for k = 1 : length(msgINS)
    % extract covariance values corresponding to position 
    % squared meters units.
    Pos_Cov_PVA(:,k) = msgINS{k,1}.Pose.Covariance([1 2 3 7 8 9 13 14 15]);
end

% Extract position covariance values from the INSPVA data
Attitude_Cov_PVA = zeros(9,length(msgINS));
for k = 1 : length(msgINS)
    % extract covariance values corresponding to position 
    % squared meters units.
    Attitude_Cov_PVA(:,k) = msgINS{k,1}.Pose.Covariance([22 23 24 28 29 30 34 35 36]);
end

% Extract position covariance values from the INSPVA data
Vel_Cov_PVA = zeros(9,length(msgINS));
for k = 1 : length(msgINS)
    % extract covariance values corresponding to position 
    % squared meters units.
    Vel_Cov_PVA(:,k) = msgINS{k,1}.Twist.Covariance([1 2 3 7 8 9 13 14 15]);
end

% Ground Speed.
GSpeed_INS = sqrt(XLinVel.^2 + YLinVel.^2 + ZLinVel.^2);
% Heading From Velocities -- arctan(North Velocity / East Velocity)
Course_INS = rad2deg( atan2(YLinVel, XLinVel) );

% pack position coordinates
UTM_INS = [Xpos; Ypos; Zpos];

% /novatel/gps_fix_utm (is mis-named) topic contains the LLA coordinates which are
% obtained obtained inspva at 20 Hz. 
bPVA = select(bag,'Topic','/novatel/gps_fix_inspva');
TimePVA = bPVA.MessageList.Time;
msgPVA = readMessages(bPVA,'DataFormat','struct');

TV_PVA = zeros(length(msgPVA),1);
for k = 1 : length(msgPVA)
    % Following time conversion is verified by toSec() function in ROS.
    TV_PVA(k) = double ( msgPVA{k,1}.Header.Stamp.Sec ) + ...
        double ( msgPVA{k,1}.Header.Stamp.Nsec ) * 1e-9;
end
% compute log time for /novatel/gps/fix topic
LogTime_PVA = TimePVA(end) - TimePVA(1);
% compute sampling frequency from data
Fs_PVA = length(msgPVA)/LogTime_PVA;

% Extract other data from /novate/gps_fix_utm topic
lat_PVA = cellfun(@(m) double(m.Latitude),msgPVA);
long_PVA = cellfun(@(m) double(m.Longitude),msgPVA);
alt_PVA = cellfun(@(m) double(m.Altitude),msgPVA);

% Covariance type defined in the NavSatFix message in ROS has four values
% 0 = COVARIANCE_TYPE_UNKNOWN
% 1 = COVARIANCE_TYPE_APPROXIMATED
% 2 = COVARIANCE_TYPE_DIAGONAL_KNOWN
% 3 = COVARIANCE_TYPE_KNOWN=3
CovType_PVA = cellfun(@(m) double(m.PositionCovarianceType),msgPVA);