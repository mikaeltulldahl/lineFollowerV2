function p = getSplinePos(coeffs, t)
t2 = t^2;
t3 = t*t2;
p = coeffs(:,1)*t3 + coeffs(:,2)*t2 + coeffs(:,3)*t + coeffs(:,4);
end