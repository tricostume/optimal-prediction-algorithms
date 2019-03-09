function xss = lsq_alpha(alphass)
% This function returns steady states for any carousel speed given as input
options = optimoptions('lsqnonlin', 'TolFun', 1e-25,'TolX', 1e-25);
xopt = lsqnonlin(@(x)function_residual(x, alphass), [1.4,-0.058*pi/180, 0, 1], [], [], options);
xss=[xopt(1), alphass, xopt(2), xopt(3), 0, 0, xopt(1),0]';
end


function res = function_residual(dv, alphass)
% lsqnonlin Residual function for least-squares error minimisation
ddelta_arm = dv(1);
alpha = alphass;
beta = dv(2);
dddelta_arm = dv(3);
ddelta_motor_sp = dv(1);
u=0;
% Computation of derivative and minimisation against steady state
x = [ddelta_arm; alpha; beta; dddelta_arm; 0; 0; ddelta_motor_sp ;0];
xdot = model_lin_cont(x,u) - [0; 0; 0; 0; 0; 0; 0; 0];
res = xdot;

end