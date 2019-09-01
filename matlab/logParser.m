% clear all
% clc

wheelbase =1.046* 0.268; %0.96
encScaling = 0.030 * pi / (9.96 * 12);

M = csvread('LOG009.TXT');
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
    lineSensorPos2(i,:) = pos2(i,:) + (rot(pi/180*heading(i))*[0.085; lineSensorValue(i)])';
end
lineSensorPosSmooth = zeros(size(lineSensorPos2));
lineSensorPosSmooth(:,1) = smoothn(lineSensorPos2(:,1), 50,'robust');
lineSensorPosSmooth(:,2) = smoothn(lineSensorPos2(:,2), 50,'robust');
%TODO investigate using smoothn with 2D cell array input: smoothn({x,y});
lineSensorPosSmoothDiff = diff(lineSensorPosSmooth,1);

K = N - 1;

segmentLength = zeros(K,1);
for i = 1:K
    segmentLength(i) = norm(lineSensorPosSmoothDiff(i,:));
end

segmentLengthCumSum = [0; cumsum(segmentLength)];

oversamplingRatio = 1;
N2 = round(size(lineSensorPosSmooth,1)*oversamplingRatio);
lineSensorPosSmoothInterpX = interp1(segmentLengthCumSum,lineSensorPosSmooth(:,1),linspace(0,segmentLengthCumSum(end),N2),'spline')';
lineSensorPosSmoothInterpY = interp1(segmentLengthCumSum,lineSensorPosSmooth(:,2),linspace(0,segmentLengthCumSum(end),N2),'spline')';

lineSensorPosSmoothInterp = [lineSensorPosSmoothInterpX, lineSensorPosSmoothInterpY];

lineSensorPosSmoothInterpDiff = diff(lineSensorPosSmoothInterp,1);
K2 = N2 - 1;
segmentLengthInterp = zeros(K2,1);
for i = 1:K2
    segmentLengthInterp(i) = norm(lineSensorPosSmoothInterpDiff(i,:));
end

curveHeading = (180/pi)*atan2(lineSensorPosSmoothInterpDiff(:,2),lineSensorPosSmoothInterpDiff(:,1));
segmentLengthInterpMean = mean(segmentLengthInterp);
curvature = zeros(K2,1);
for i = 2:K2
    %curvature = dHeading/dDist
    dHeading = wrapAngle(curveHeading(i) - curveHeading(i-1));
    curvature(i) = dHeading/segmentLengthInterpMean;
end

[curvatureCorr2,lags2] = xcorr(curvature,'unbiased');
curvatureCorr2 = curvatureCorr2/max(curvatureCorr2);
minLineLength = 2;
minIdx = 500;%round(1 + minLineLength/segmentLengthInterp(1));
[maxCorr, maxCorrLags] = max(curvatureCorr2(N2-1+minIdx:end));
maxCorrLags = maxCorrLags + minIdx - 1;

disconuity = lineSensorPosSmoothInterp(maxCorrLags + 1,:) - lineSensorPosSmoothInterp(1,:);

figure(1)
clf
hold on
grid on
plot(segmentLengthInterp)
plot(segmentLength)

figure(2)
clf
hold on
grid on
plot(lags2,curvatureCorr2,'.b')

figure(4);
clf;
grid on
hold on
% axis equal
% plot(pos(:,1),pos(:,2),'b')
% plot(pos2(:,1),pos2(:,2),'r')
plot(lineSensorPos2(:,1),lineSensorPos2(:,2),'k')
% plot(lineSensorPosSmooth(:,1),lineSensorPosSmooth(:,2),'ob')
% plot(lineSensorPosSmoothInterp(:,1),lineSensorPosSmoothInterp(:,2),'xr')
plot(lineSensorPosSmoothInterp(1:maxCorrLags,1),lineSensorPosSmoothInterp(1:maxCorrLags,2),'b')
% plot(lineSensorPos(:,1),lineSensorPos(:,2),'k')

figure(5);
clf;
grid on
hold on
plot(heading,'b')
plot(heading2,'r')
plot(curveHeading,'g')

plot(25*(heading2 - heading),'k')

figure(6);
clf;
grid on
hold on
plot(curvature,'b')
