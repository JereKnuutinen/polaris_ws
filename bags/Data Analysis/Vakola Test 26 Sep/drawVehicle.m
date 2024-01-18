function drawVehicle(y1,y2,FwrF,FwlF,FwrR,FwlR,const,AlpharF,AlphalF,AlpharR,AlphalR)

persistent skipCount
if isempty(skipCount)
    skipCount = 0;
end

skipCount = skipCount + 1;
if skipCount < 100
    return
end
skipCount = 0;

xp = y1(1); yp = y1(2); zp = y1(3); % position of CG
roll = y1(7); pitch = y1(8); yaw = y1(9); % Euler angles
u = y1(4);  % CG velocities in body frame
v = y1(5);
w = y1(6); 
p = y1(10); q = y1(11); r = y1(12); % body rate about CG

v_in = y2(4); % input velocity in body x-axis
curv = y2(5); % curvature

g = const(5);
l = const(1); h = const(2); t = const(3);
Ca = const(15);

Kcl = curv/(1-(t/2)*curv); Kcr = curv/(1+(t/2)*curv);
% understeer gradient as function of lateral tyre forces
% times normalized lateral acceleration
V = sqrt(u^2 + v^2);
etaR = (FwrF(3)/Ca-FwrR(3)/Ca)*((V^2*Kcr)/g); 
etaL = (FwlF(3)/Ca-FwlR(3)/Ca)*((V^2*Kcl)/g);
% total steering angle is the sum of Ackermann steering angle 
% and a correction factor governed by understeer index eta.
STL1 = atan(l*Kcl);% + etaL; % steering angle on left tire
STR1 = atan(l*Kcr);% + etaR; % steering angle for right wheel
% understeer gradient as function of differences in front tyre and rear
% tyre slip angles times normalized lateral acceleration
% UGr = (AlpharF - AlpharR)*((u^2*Kcr)/g);
% UGl = (AlphalF - AlphalR)*((u^2*Kcl)/g);
% STL2 = atan(l*Kcl) + UGl;
% STR2 = atan(l*Kcr) + UGr;

% vectors for right_{Front, Rear} and left_{Front, Rear} corners
% of the vehicle in (Xb, Yb, Zb)
prF = [ l/2; -t/2; -h/2];
prR = [-l/2; -t/2; -h/2];
plF = [ l/2;  t/2; -h/2];
plR = [-l/2;  t/2; -h/2];

Rx = @(roll)[1 0 0;0 cos(roll) -sin(roll); 0 sin(roll) cos(roll)];
Ry = @(pitch)[cos(pitch) 0 sin(pitch); 0 1 0;-sin(pitch) 0 cos(pitch)];
Rz = @(yaw)[cos(yaw) -sin(yaw) 0;sin(yaw) cos(yaw) 0; 0 0 1];

windowsize = 10;
scale = 1000;

xmin = xp - windowsize;
xmax = xp + windowsize;
ymin = yp - windowsize;
ymax = yp + windowsize;
zmin = zp - windowsize;
zmax = zp + windowsize;

figure(10);
clf;
axis([xmin xmax ymin ymax zmin zmax]);
view(45,25);
hold on;

C = Rz(yaw)*Ry(pitch)*Rx(roll);% body to global (CG)
WrF = C*prF + [xp; yp; zp];
WrR = C*prR + [xp; yp; zp];
WlF = C*plF + [xp; yp; zp];
WlR = C*plR + [xp; yp; zp];

FrF = C*FwrF;
FlF = C*FwlF;
FrR = C*FwrR;
FlR = C*FwrR;

Rf = (FwrF(3) - FwlF(3))/(FwrF(3) + FwlF(3));
Rr = (FwrR(3) - FwlR(3))/(FwrR(3) + FwlR(3));

plot3([WrF(1), WrR(1), WlR(1), WlF(1), WrF(1)],[WrF(2), WrR(2), WlR(2), WlF(2), WrF(2)],[WrF(3), WrR(3), WlR(3), WlF(3), WrF(3)]);
plot3(xp, yp, zp, 'ro', 'DisplayName','CG');
plot3([WrF(1), WrF(1)+FrF(1)*scale],[WrF(2), WrF(2)+FrF(2)*scale],[WrF(3), WrF(3)+FrF(3)*scale],'DisplayName','rF');
plot3([WrR(1), WrR(1)+FrR(1)*scale],[WrR(2), WrR(2)+FrR(2)*scale],[WrR(3), WrR(3)+FrR(3)*scale],'DisplayName','rR');
plot3([WlF(1), WlF(1)+FlF(1)*scale],[WlF(2), WlF(2)+FlF(2)*scale],[WlF(3), WlF(3)+FlF(3)*scale],'DisplayName','lF');
plot3([WlR(1), WlR(1)+FlR(1)*scale],[WlR(2), WlR(2)+FlR(2)*scale],[WlR(3), WlR(3)+FlR(3)*scale],'DisplayName','lR');
legend('Location','best');
str = sprintf('\\delta_r = %3.1f, \\eta_r = %3.1f',STL1*57.3, etaR);
text(WrF(1)+1,WrF(2),WrF(3),str);
str = sprintf('Front LTR = %f', Rf);
text(WrF(1),WrF(2),WrF(3)+11,str);
str = sprintf('\\delta_l = %3.1f, \\eta_l = %3.1f',-STR1*57.3, etaL);
text(WlR(1)+1,WlR(2),WlR(3),str);
str = sprintf('Rear LTR = %f', Rr);
text(WlR(1),WlR(2),WlR(3)+11,str);
drawnow;
pause(0.1);
