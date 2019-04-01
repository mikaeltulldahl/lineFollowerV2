function [gradient, f0] = functionGradient(handle, x, delta)
N = length(x);
f0 = handle(x);
gradient = zeros(N,1);
for i = 1:N
%     xTemp = x;
%     xTemp(i) = xTemp(i) + delta;
%     gradient(i) = (handle(xTemp) - f0)/delta;
    xPos = x;
    xPos(i) = xPos(i) + delta;
    xNeg = x;
    xNeg(i) = xNeg(i) - delta;
    gradientPos = (handle(xPos) - f0)/delta;
    gradientNeg = -(handle(xNeg) - f0)/delta;
    gradient(i) = 0.5*(gradientPos + gradientNeg);
end
end