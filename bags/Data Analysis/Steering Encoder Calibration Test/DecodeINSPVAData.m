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
yaw_INS = -rad2deg( eul_INS(:,1) ); % Azimuth (-pi to pi)
temp = zeros(size(yaw_INS));
for k = 1 : length(yaw_INS)
    temp(k) = yaw2track(yaw_INS(k));
    yaw_INS(k) = track2yaw( -90.0 - temp(k) ); % Heading
end
% Ground Speed.
GSpeed_INS = sqrt(XLinVel.^2 + YLinVel.^2 + ZLinVel.^2);
% Heading From Velocities -- arctan(North Velocity / East Velocity)
Course_INS = rad2deg( atan2(YLinVel, XLinVel) );

ENU = [Xpos; Ypos; Zpos];% - [Xpos(1); Ypos(1); Zpos(1)];
% ENU = ENU';