function [time, dist, pos2, heading, lineSensorPos2] = logParser(shouldPlot, logName)
% clear all
% clc

wheelbase =1.046* 0.268; %0.96
lineSensorX = 0.085;
encScaling = 0.030 * pi / (9.96 * 12);

if nargin < 2
    logName = 'LOG009';
end
M = csvread([logName '.TXT']);
M(1:10,:) = []; %trim away first samples
M((end-10):end,:) = []; %trim away last samples
N = size(M,1);
time = 0.001*M(:,1);
velocity = M(:,2);
heading = M(:,3);
pos = M(:,4:5);
dist = encScaling*M(:,6:7);
lineSensorState = M(:,8);
lineSensorValue = M(:,9);
lineSensorPos = M(:,10:11);

meanDist = mean(dist,2);

pos2 = zeros(size(pos));
pos2(1,:) = pos(1,:);
heading2 = zeros(size(heading));
heading2(1) = heading(1);

for i = 2:N
    dt = time(i) - time(i-1);
    dDist = dist(i,:) - dist(i-1,:);
    avgDist = mean(dDist);
    heading2(i) = heading2(i-1) + (180/pi)*(dDist(1) - dDist(2))/wheelbase;
    pos2(i,:) = pos2(i-1,:) + avgDist*[cos(pi/180*heading(i)) sin(pi/180*heading(i))];
    
%     posX += micros2sec(timeDiff) * velocity * cosf(M_PI / 180.0f * heading);
%   posY += micros2sec(timeDiff) * velocity * sinf(M_PI / 180.0f * heading);
end


lineSensorPos2 = zeros(size(lineSensorPos));
for i = 1:N
    lineSensorPos2(i,:) = pos2(i,:) + (rotDeg(heading(i))*[lineSensorX; lineSensorValue(i)])';
end

if shouldPlot
    figure(4);
    plot(pos(:,1),pos(:,2),'b')
    plot(pos2(:,1),pos2(:,2),'r')
    
    figure(5);
    plot(heading,'b')
    plot(heading2,'r')
    % plot(25*(heading2 - heading),'k')
end