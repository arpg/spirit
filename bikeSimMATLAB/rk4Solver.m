function [xdis,ydis,phi,t] = rk4Solver(xdis,ydis,phi,dt,t,v,beta,lr)
  xvel = @(phi) v * cos(phi + beta);
  yvel = @(phi) v * sin(phi + beta);
  phi_dot = v/lr * sin(beta);
  
  l1 = dt*xvel(phi);
  l2 = dt*xvel(phi + l1/2);
  l3 = dt*xvel(phi + l2/2);
  l4 = dt*xvel(phi+l3);
  
  z1 = dt*yvel(phi);
  z2 = dt*yvel(phi + z1/2);
  z3 = dt*yvel(phi + z2/2);
  z4 = dt*yvel(phi+z3);
  
  k1 = dt*phi_dot;
  k2 = dt*phi_dot;
  k3 = dt*phi_dot;
  k4 = dt*phi_dot;
  
  xdis = xdis + l1/6 + l2/3 + l3/3 + l4/6;
  ydis = ydis + z1/6 + z2/3 + z3/3 + z4/6;
  phi = phi + k1/6 + k2/3 + k3/3 + k4/6;
  t = t + dt;
end

