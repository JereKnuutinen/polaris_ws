function y = track2yaw(x)

if x > 180.0
    y = x - 360.0;
elseif x < -180.0
    y = x + 360.0;
else
    y = x;
end