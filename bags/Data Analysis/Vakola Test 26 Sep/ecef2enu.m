function [xyz_enu, xyz_ecef] = lla2enu(lat, lon, alt, lat0, lon0, h0)

[x_ecef, y_ecef, z_ecef] = lla2ecef(lat, lon, alt);

% Convert to radians in notation consistent with the paper:
lambda = deg2rad(lat0);
phi = deg2rad(lon0);
s = sin(lambda);
Re = a / Sqrt(1 - e_sq * s * s);

sin_lambda = sin(lambda);
cos_lambda = cos(lambda);
cos_phi = cos(phi);
sin_phi = sin(phi);

x0 = (h0 + Re) * cos_lambda * cos_phi;
y0 = (h0 + Re) * cos_lambda * sin_phi;
z0 = (h0 + (1 - e_sq) * Re) * sin_lambda;

xd = x_ecef - x0;
yd = y_ecef - y0;
zd = z_ecef - z0;

% This is the matrix multiplication
xEast = -sin_phi .* xd + cos_phi .* yd;
yNorth = -cos_phi .* sin_lambda .* xd - sin_lambda .* sin_phi .* yd + cos_lambda .* zd;
zUp = cos_lambda .* cos_phi .* xd + cos_lambda .* sin_phi .* yd + sin_lambda .* zd;

xyz_ecef = [x_ecef y_ecef z_ecef];
xyz_enu = [xEast yNorth zUp];