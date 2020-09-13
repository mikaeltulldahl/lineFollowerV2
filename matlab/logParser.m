function [time, dist, pos, heading, lineSensorPos] = logParser(shouldPlot, logName)
encScaling = 0.030 * pi / (9.96 * 12);

if nargin < 2
    logName = 'LOG009.TXT';
end
M = csvread(logName);
M(1:10,:) = []; %trim away first samples
M((end-10):end,:) = []; %trim away last samples

time = 0.001*M(:,1);
velocity = M(:,2);
heading = M(:,3);
pos = M(:,4:5);
dist = encScaling*M(:,6:7);
lineSensorState = M(:,8);
lineSensorValue = M(:,9);
lineSensorPos = M(:,10:11);

if shouldPlot
    figure(11);
    clf;
    title('robotPos');
    hold on
    grid on
    plot(pos(:,1),pos(:,2),'b')

    figure(14);
    clf;
    title('linePos');
    hold on
    grid on
    plot(lineSensorPos(:,1),lineSensorPos(:,2),'b')
end