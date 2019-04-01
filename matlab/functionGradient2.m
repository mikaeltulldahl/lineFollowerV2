function [gradient, f0] = functionGradient2(fun, x, delta)
% syntax: fun(x(i), i)
N = length(x);
f0 = zeros(N,1);
gradient = zeros(N,1);
for i = 1:N
    fPos = fun(x(i) + delta, i);
    fNeg = fun(x(i) - delta, i);
    f0(i) = 0.5*(fPos + fNeg);
    gradient(i) = 0.5/delta*(fPos - fNeg);
end
end