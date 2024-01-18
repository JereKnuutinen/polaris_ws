function [xyz_ned xyz_ecef] = lla2ned(lat, lon, alt, cen, xyz0_ned)

[x_ecef y_ecef z_ecef] = lla2ecef(lat, lon, alt);


% [cen x0_ned y0_ned z0_ned] = InitializeNav(lat, lon, alt);

x0_ned = xyz0_ned(1);
y0_ned = xyz0_ned(2);
z0_ned = xyz0_ned(3);

x1_ned = cen(1,1)*x_ecef + cen(1,2)*y_ecef + cen(1,3)*z_ecef;
y1_ned = cen(2,1)*x_ecef + cen(2,2)*y_ecef + cen(2,3)*z_ecef;
z1_ned = cen(3,1)*x_ecef + cen(3,2)*y_ecef + cen(3,3)*z_ecef;
    
% Translate coordinates by target coordinates to set target as reference

x_ned = x1_ned - x0_ned;
y_ned = y1_ned - y0_ned;
z_ned = z1_ned - z0_ned;

% outputs

xyz_ecef = [x_ecef y_ecef z_ecef];

xyz_ned = [x_ned y_ned z_ned];
