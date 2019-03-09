function xopt = solve_steady_state_lsq(alpha_ref,l_t_ref)

%This function solves for steady states by solving a least squares problem.

options = optimoptions('lsqnonlin', 'TolFun', 1e-25,'TolX', 1e-25);
xopt = lsqnonlin(@(x)g(x, alpha_ref, l_t_ref), [1.35, -0.04851, 0, 1.35, 0], [], [], options);

end


function ret = g(dv, alpha_ref, l_t_ref)


ddelta_arm = dv(1);
alpha = alpha_ref;
beta = dv(2);
dddelta_arm = dv(3);
ddelta_motor_sp = dv(4);
l_tether=l_t_ref;

u=[ddelta_motor_sp,0];

x = [ddelta_arm; alpha; beta; dddelta_arm; 0; 0; l_tether];
xdot = nmpc_carr_lagr(x,u) - [0; 0; 0; 0; 0; 0; 0];
ret = xdot;

end