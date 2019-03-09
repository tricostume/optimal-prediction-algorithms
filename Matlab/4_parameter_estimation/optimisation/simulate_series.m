function [T,X,U] = simulate_series(u, time,theta)
% This function is in charge of receiving data packets and simulating with
% them, namely controls and parameters. Attention: This function receives
% also optimisation variables theta.

%1. Set sampling time and numer of samples
Ts = 0.1;
N = length(u);
offset=1;
%2. Initial guesses depending on the example used
% multisteps experiment
%xss0 = [1.054, -1.2896, -0.0411,8.68286935912251e-31,  0, 0]';
% Initial experiment
xss0 = [1.543, -0.9546, -0.0538, 0, 0, 0];
%xss0 = [0, -pi/2, -0.0538, 0, 0, 0];
%3. Matrices initialisation 
nx = length(xss0);
X = zeros(nx, N + 1);
X(:, 1) = xss0;
T = 0:Ts:((N-offset)*Ts);
U = u(offset:end - 1);
%4. Simulation loop
for k = 1:(N - offset)
    x = X(:, k);
    u = U(k);
    x = integrate(x, u, Ts,theta);
    X(:, k + 1) = x;
end

end

function xnext = integrate(x0, u0, dt,theta)
%This is the ode15s integrator

options = odeset('RelTol', 5e-14);
[~, Y] = ode15s(@(t, x)modelopt(x, u0,theta), [0, dt], x0, options);
xnext = Y(end, :)';

end