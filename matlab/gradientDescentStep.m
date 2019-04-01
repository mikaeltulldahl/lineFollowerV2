function [xOptimal, f0] = gradientDescentStep(funIdx, funTot, initX, delta)
% syntax: cost(i) = funIdx(x, i)
% syntax: sum(cost) = funTot(x)
[gradient, f0] = functionGradient2(funIdx, initX, delta);
%line search best step length
gradientNorm = norm(gradient);
unitGradient = gradient/gradientNorm;
fun2 = @(x) funTot(initX - x*unitGradient);
stepLength = lineSearch(fun2, delta);
xOptimal = -stepLength*unitGradient;
end