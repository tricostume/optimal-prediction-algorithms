function [A, B, C] = discretise_model(x0, u0)
%This script applies finite difference aproximations to
%the next state with respect to states and controls.

A = finite_differences(@(x)simulate_step15(x,u0), x0, 1e-6);
B = finite_differences(@(u)simulate_step15(x0,u), u0, 1e-6);
C = finite_differences(@output, x0, 1e-6);
end