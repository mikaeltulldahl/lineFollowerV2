function twistRequest = controller(odometry, dt)
global path initPose

%calculate pose in map from mapPose and odometry

%find lateral error to nearest path segment to left/right using intersection(...)

%find path curvature

yawRateRequest = 100*lineMeasured;
speedRequest = max(0.1, 0.3 - 3*abs(lineMeasured));
twistRequest = struct('speed', max(0.1, 0.3 - 3*abs(lineMeasured)), 'yawRate', 100*lineMeasured);
end