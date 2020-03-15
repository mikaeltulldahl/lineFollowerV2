function twistRequest = simpleController(lineMeasured)
twistRequest = struct('speed', max(0.1, 0.3 - 3*abs(lineMeasured)), 'yawRate', 100*lineMeasured);
end