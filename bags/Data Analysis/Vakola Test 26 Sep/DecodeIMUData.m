%% Extract data from Novatel's CORRIMU and INSPVA message.
% /novatel/imu topic contains quarternions, rates and 
% accelerations collected from CORRIMU and INSPVA sentences.
bIMU = select(bag,'Topic','/novatel/imu');
TimeIMU = bIMU.MessageList.Time;
msgIMU = readMessages(bIMU,'DataFormat','struct');

TV_IMU = zeros(length(msgIMU),1);
for k = 1 : length(msgIMU)
    % Following time conversion is verified by toSec() function in ROS.
    TV_IMU(k) = double ( msgIMU{k,1}.Header.Stamp.Sec ) + ...
        double ( msgIMU{k,1}.Header.Stamp.Nsec ) * 1e-9;
end
% compute log time for /novatel/odom_gps topic
LogTime_IMU = TimeIMU(end) - TimeIMU(1);
% compute sampling frequency from data
Fs_IMU = length(msgIMU)/LogTime_IMU;

vel = cellfun(@(m) struct(m.AngularVelocity),msgIMU);
acc = cellfun(@(m) struct(m.LinearAcceleration),msgIMU);
orientation = cellfun(@(m) struct(m.Orientation),msgIMU);

qx = [orientation(1:end).X]';
qy = [orientation(1:end).Y]';
qz = [orientation(1:end).Z]';
qw = [orientation(1:end).W]';

% Extract RPY angles from quarternion -- do not change the sequence
eul = quat2eul([qw, qx, qy, qz], 'ZYX');
roll_imu = qx; %rad2deg( eul(:,3) );
pitch_imu = qy; %rad2deg( eul(:,2) );
yaw_imu = qz; %rad2deg( eul(:,1) );

% extract ins status
INS_Status = qw;

roll_rate = rad2deg( [vel(1:end).X]' ); % around body X-axis
pitch_rate = rad2deg( [vel(1:end).Y]' ); % around body Y-axis
yaw_rate = rad2deg( [vel(1:end).Z]' ); % around body Z-axis

Xacc = [acc(1:end).X]; % longitudinal acceleration
Yacc = [acc(1:end).Y]; % lateral acceleration
Zacc = [acc(1:end).Z]; % verticle acceleration