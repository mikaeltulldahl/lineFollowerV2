function v = getSplineVel(coeffs, t)
v = 3*coeffs(:,1)*t^2 + 2*coeffs(:,2)*t + coeffs(:,3);
end