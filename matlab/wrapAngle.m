function out = wrapAngle(in)
%wraps angle to interval [-180, 180]
out = mod(in + 180, 2*180) - 180;
end