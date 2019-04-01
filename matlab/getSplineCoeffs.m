function [coeffs, t, success] = getSplineCoeffs(p0, p1, v0, v1)
% cubic spline:
% x(t) = ax*t^3 + bx*t^2 + cx*t + dx
% y(t) = ay*t^3 + by*t^2 + cy*t + dy
% -> p(t) = coeffs(:,1)*t^3 + coeffs(:,2)*t^2 + coeffs(:,3)*t + coeffs(:,4)

% conditions:
% - p(t0) = p0
% - p'(t0) = v0
% - p(t1) = p1
% - p'(t1) = v1
% - constant acceleration in direction from p0 to p1

%x0 = dx
%vx0 = cx
%x1 = ax*t1^3 + bx*t1^2 + cx*t1 + dx
%vx1 = 3*ax*t1^2 + 2*bx*t1 + cx

%x1 = ax*t1^3 + bx*t1^2 + cx*t1 + dx
%vx1 = 3*ax*t1^2 + 2*bx*t1 + cx

%x1 - x0 - vx0*t1 = ax*t1^3 + bx*t1^2
%vx1 - vx0 = 3*ax*t1^2 + 2*bx*t1




epsilon = 1e-4;

t = 10;
success = false;
dP = p1 - p0;
dPNorm = norm(dP);
if dPNorm > epsilon
    unitdP = dP/dPNorm;
    meanVeldP = 0.5*unitdP'*(v1 + v0);
    if meanVeldP > epsilon
        t = dPNorm/meanVeldP;
        if t > 0
            t2 = t^2;
            t3 = t*t2;
            b = [(p1 - p0 - v0*t), (v1 - v0)]';
            A = [  t3,   t2;
                  3*t2, 2*t];
            coeffs = [(A\b)' v0, p0];
            success = true;
        else
            coeffs = ones(2,4);
            warning('only sollution is negative time -> velocity is in wrong direction');
        end
    else
        coeffs = ones(2,4);
        warning('too low mean velocity');
    end
else
    coeffs = ones(2,4);
    warning('too short spline length');
end
end