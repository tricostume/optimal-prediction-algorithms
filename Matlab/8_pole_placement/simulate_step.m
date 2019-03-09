function [X] = simulate_step(x0,u)
% This function is in charge of receiving data packets and simulating with
% them, namely ONLY controls.

%1. Set sampling time and numer of samples
Ts = 0.1;
X = integrate(x0, u, Ts);
end
function xnext = integrate(x0, u0, dt)
%This is the ode15s integrator
options = odeset('RelTol', 5e-14);
[~, Y] = ode15s(@(t, x)model_lin_cont(x, u0), [0, dt], x0, options);
xnext = Y(end, :)';

end