function a = getSplineAcc(coeffs, t)
a = 6*coeffs(:,1)*t + 2*coeffs(:,2);
end