function [y] = output(x)
% With this function, a masking is done over the full state to only let 
% pass the sensed states.

y = x(2);
end