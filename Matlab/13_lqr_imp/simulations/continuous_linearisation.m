function [A, B, C, D] = continuous_linearisation(xss, uss)
%Use of finite differences to obtain the matrices describing the state
%space representation of the system. The next formulation was used:
% x =[ddelta_arm alpha beta dddelta_arm dalpha dbeta]
% u = ddelta_motorsp
% y = [alpha]

A = finite_differences(@(x)model_lin_cont(x,uss), xss, 1e-7);
B = finite_differences(@(u)model_lin_cont(xss,u), uss, 1e-7);
C = finite_differences(@output, xss, 1e-7);
D = 0;

%Analysis of controllability 
Controllability = ctrb(A ,B);

if rank(Controllability) < rank(A)
    disp('System not controllable')
end

end

function [J] = finite_differences(f, var_i, h)
% Computation of the jacobian JF between f and a specified variable by 
% using finite difference approximations.

f0 = f(var_i);
[rows,columns] = size(f0);
J = zeros(rows, length(var_i));
e = eye(length(var_i));

for j = 1:length(var_i)
    J(:, j) = (f(var_i + h*e(:, j)) - f0)/h;
end    

end