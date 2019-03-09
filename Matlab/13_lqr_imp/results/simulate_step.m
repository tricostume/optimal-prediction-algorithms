function [X] = simulate_step(x0,u,mode)
% This function is in charge of receiving data packets and simulating with
% them, namely ONLY controls.
%1. Set sampling time and numer of samples
Ts = 0.1;
X = integrate(x0, u, Ts, mode);
end
function xnext = integrate(x0, u0, dt, mode)
%This is the ode15s integrator
options = odeset('RelTol', 5e-14);
if mode == 2
eval('[~, Y] = ode15s(@(t, x)model_simulator17(x, u0), [0, dt], x0, options);');
else if mode == 1
eval('[~, Y] = ode15s(@(t, x)model_simulator13(x, u0), [0, dt], x0, options);');
    end
end
xnext = Y(end, :)';

end