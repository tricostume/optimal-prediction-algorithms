function [JF] = fingrad(F, x0, h)

%Numerical calculation of the Jacobian of function F(x), evaluated at x0 with accuracy h

F0 = F(x0);
[nrow,ncol] = size(F0);
assert(ncol == 1, 'need a column vector')
JF = zeros(nrow, length(x0));
e = eye(length(x0));

for j = 1:length(x0)
    JF(:, j) = (F(x0 + h*e(:, j)) - F0)/h;
end    

end