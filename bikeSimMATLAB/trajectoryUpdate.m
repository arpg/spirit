% ----------------------------------------
% ----------Update Animation----------
% ----------------------------------------

% homogenous matrix translation in XY plane, followed by rotation about
% Z axis
H = [cos(pos_phi) -sin(pos_phi) Px; sin(pos_phi) cos(pos_phi) Py; 0 0 1];
pos = H*bikeFrame;
pos_COG = H*COG;
Hfr = H*[cos(sf) -sin(sf) cl/2; sin(sf) cos(sf) 0; 0 0 1]*Wheel;
Hrw = H*transformRearWheel;

clearpoints(rearWheel);
clearpoints(frontWheel);
clearpoints(h);
addpoints(h,pos(1,:),pos(2,:));
addpoints(traj_COG,pos_COG(1,:),pos_COG(2,:));
addpoints(frontWheel, Hfr(1,:), Hfr(2,:));
addpoints(rearWheel, Hrw(1,:), Hrw(2,:));

% axis([-worldsize, 2*worldsize, -worldsize, 2*worldsize])
drawnow