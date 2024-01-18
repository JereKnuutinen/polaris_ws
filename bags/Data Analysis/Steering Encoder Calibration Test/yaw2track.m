function y = yaw2track(x)

if x < 0.0
    y = x + 360.0;
elseif x > 360.0
    y = x - 360.0;
else
    y = x;
end
