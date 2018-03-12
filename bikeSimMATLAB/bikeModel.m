% ----------------------------------------
% --------Bike Simulation Main--------
% ----------------------------------------
clc,clear
close all
buildBike; % load bike graphic
v = 1; % velocity
sf = -pi/8; % steering angle
dt = 0.01;
tf = 6;
xvel(1) = 0;
yvel(1) = 0;
phi_dot(1) = 0;
phi(1) = 0;
xdis(1) = 0;
ydis(1) = 0;
t(1) = 0;
worldSize = 100;
ctr = 2;

for ii=1:dt:tf
% while true
%     sf = sf - pi/500;
    if ctr > 300
        sf = pi/8;
    end
    beta = atan(lr / (lr+lf) * tan(sf));
    
    Px = xdis(ctr-1);
    Py = ydis(ctr-1);
    pos_phi = wrapToPi(phi(ctr-1));
    trajectoryUpdate;
      
    [xdis(ctr), ydis(ctr), phi(ctr), t(ctr)] = rk4Solver(xdis(ctr-1), ydis(ctr-1), phi(ctr-1),dt,t(ctr-1),v,beta,lr);
    
%     % update velocities  
%     xvel(ctr) = v * cos(phi(ctr-1) + beta);
%     yvel(ctr) = v * sin(phi(ctr-1) + beta);
%     phi_dot(ctr) = v/lr * sin(beta);
% 
%     % integrate velocities
%     dphi(ctr) = dt/2 * (phi_dot(ctr) + phi_dot(ctr-1));
%     dx(ctr) = dt/2 * (xvel(ctr) + xvel(ctr-1));
%     dy(ctr) = dt/2 * (yvel(ctr) + yvel(ctr-1));
% 
%     % update position
%     xdis(ctr) = xdis(ctr-1) + dx(ctr);
%     ydis(ctr) = ydis(ctr-1) + dy(ctr);
%     phi(ctr) = phi(ctr-1) + dphi(ctr);

    ctr = ctr + 1;
end

% figure(2)
% hold on
% plot(t,xdis)
% 
% y0 = [0 0 0];
% tspan = [0 3];
% [t2,y] = ode45(@myODE,tspan,y0);
% a = y(:,3);
% x = y(:,1);
% yd = y(:,2);
% plot(t2,x)
% 
% figure(3)
% hold on 
% plot(t,ydis)
% plot(t2,yd)
% 
% % figure(4)
% % hold on
% % plot(t,phi)
% % plot(t2,a)






