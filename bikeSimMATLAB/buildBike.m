% ----------------------------------------
% --------Create Bike and World--------
% ----------------------------------------
figure(1)
lf = .15; % length from COG to front steering axis
lr = .15; % length from COG to rear axle
cl = lf + lr; % chassis length
cw = 0.04*2; % chassis width
wl = .1; % wheel length
ww = .03; % wheel width
bikeFrame = [-cl/2  -cl/2  cl/2  cl/2  -cl/2; -cw/2  cw/2  cw/2  -cw/2  -cw/2;  1 1 1 1 1];
Wheel = [-wl/2  -wl/2  wl/2  wl/2  -wl/2; -ww/2  ww/2  ww/2  -ww/2  -ww/2; 1 1 1 1 1];
h = animatedline(bikeFrame(1,:),bikeFrame(2,:));
% rear_axle = [-cl/2;0;1];
% traj_r = animatedline(rear_axle(1,:),rear_axle(2,:),'Color','b');
COG = [0;0;01]; % center of gravity 
traj_COG = animatedline(COG(1,:),COG(2,:),'Color','r');

% position wheels on frame
transformFrontWheel = [1 0 cl/2; 0 1 0; 0 0 1]*Wheel;
frontWheel = animatedline(transformFrontWheel(1,:),transformFrontWheel(2,:))

transformRearWheel = [1 0 -cl/2; 0 1 0; 0 0 1]*Wheel;
rearWheel = animatedline(transformRearWheel(1,:),transformRearWheel(2,:))

worldsize = 1;
% axis([-worldsize, worldsize, -worldsize, worldsize])
axis equal


