function [T,X,U] = simulate_series(u)
% This function is in charge of receiving data packets and simulating with
% them, namely ONLY controls.

%1. Set sampling time and numer of samples
Ts = 0.1;
N = length(u);
offset=1;
%2. Initial guess for this example
xss0 = [1.19, -1.2041, -0.0448, 0, 0, 0]';
%3. Matrices initialisation 
nx = length(xss0);
X = zeros(nx, N -offset);
X(:, 1) = xss0;
T = 0:Ts:((N-offset-1)*Ts);
U = u(offset:N - 1);
%4. Simulation loop
for k = 1:(N - offset-1)
    x = X(:, k);
    u = U(k);
    x = integrate(x, u, Ts);
    X(:, k + 1) = x;
end

end
function xnext = integrate(x0, u0, dt)
%This is the ode15s integrator
options = odeset('RelTol', 5e-14);
[~, Y] = ode15s(@(t, x)model(x, u0), [0, dt], x0, options);
xnext = Y(end, :)';

end