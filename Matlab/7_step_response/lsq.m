function xopt = lsq(vel)
% This function returns steady states for any carousel speed given as input
options = optimoptions('lsqnonlin', 'TolFun', 1e-25,'TolX', 1e-25);
xopt = lsqnonlin(@(x)function_residual(x, vel), [-pi/2,-0.058*pi/180, 0], [], [], options);

end


function res = function_residual(dv, vel)
% lsqnonlin Residual function for least-squares error minimisation
ddelta_arm = vel;
alpha = dv(1);
beta = dv(2);
dddelta_arm = dv(3);
u=vel;
% Computation of derivative and minimisation against steady state
x = [ddelta_arm; alpha; beta; dddelta_arm; 0; 0];
xdot = model(x,u) - [0; 0; 0; 0; 0; 0];
res = xdot;

end