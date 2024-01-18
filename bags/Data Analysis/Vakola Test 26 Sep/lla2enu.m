function [xyz_enu, xyz_ecef] = lla2enu(lat, lon, alt)

%---------------------------------------------------%
%   LLA to ECEF Transformation for any Point        %
%---------------------------------------------------%

%   Equatorial Radius WGS84 a = 6378137.0000 m 	    %
%   Polar Radius WGS84      b = 6356752.3142 m 		%
%   e = 1 - (b * b)/(a * a) = 1 - 0.993395         	%
%   Re = a/(sqrt(1-e * sin(lat) * sin(lat)))       	%
%   x_ecef  = (Re + alt) * cos(lat) * cos(lon)      %
%   y_ecef  = (Re + alt) * cos(lat) * sin(lon)		%
%   z_ecef  = (Re * (1-e) + alt) * sin(lat)  		%
% --------------------------------------------------%
a = 6378137.0000; % WGS-84 Earth semimajor axis (m)
b = 6356752.3142; % Derived Earth semiminor axis (m)
e = 1 - (b * b)/(a * a); % Square of Eccentricity

Lat = deg2rad(lat);    % Latitude in radians
Lon = deg2rad(lon);    % Longitude in radians
sinLat = sin(Lat); cosLat = cos(Lat);
sinLon = sin(Lon); cosLon = cos(Lon);
Re = a./(sqrt(1- e.* sinLat.* sinLat));

% Compute ECEF Coordinates for each point
x_ecef = (Re + alt).*cosLat.* cosLon;
y_ecef = (Re + alt).*cosLat.* sinLon;
z_ecef = (Re * (1-e) + alt).* sinLat;

% Take the first point as initial point
x0 = x_ecef(1);
y0 = y_ecef(1);
z0 = z_ecef(1);

% Translate the ECEF coordintes w.r.t initial point
xd = x_ecef - x0;
yd = y_ecef - y0;
zd = z_ecef - z0;

% ECEF Axis Sytem */
% --------------- */
% Origin at center of earth, X Axis extends towards the equator  */
% and passes thru the Greenwich meridian. Z Axis passes thru the */
% the North pole and the Y axis completes the right hand system  */

% ENU Axis System */
% --------------- */
% Origin at the center of the earth. Z axis is rotated by the    */
% longitude angle to get an intermediate axis system [x1,y1,z1]  */

% | x1 |   | cos(lon) sin(lon) 0.0  || X |                       */
% | y1 | = |-sin(lon) cos(lon) 0.0  || Y |                       */
% | z1 |   |      0.0      0.0 1.0  || Z |                       */
   
% This intermediate axis system [x1 y1 z1] is rotated about the  */
% the y1 axis by the latitude angle(lat) to give [x2 y2 z2]      */

% | x2 |   | cos(lat) 0.0 sin(lat)|| x1 |                        */
% | y2 | = |      0.0 1.0     0.0 || y1 |                        */
% | z2 |   |-sin(lat) 0.0 cos(lat)|| z1 |                        */

% Now [x2 y2 z2] = [U E N] */
% The above two transformation matrices would give */

% | -D |   | x2 |  |     | |     || X |    */
% |  E | = | y2 | =| 3x3 | | 3x3 || Y |    */
% |  N |   | z2 |  |     | |     || Z |    */

% Implement the matrix computations 
East = -sinLon .* xd + cosLon .* yd;
North = -cosLon .* sinLat .* xd - sinLat .* sinLon .* yd + cosLat .* zd;
Up = cosLat .* cosLon .* xd + cosLat .* sinLon .* yd + sinLat .* zd;

% outputs
xyz_ecef = [x_ecef y_ecef z_ecef];
xyz_enu = [East North Up];