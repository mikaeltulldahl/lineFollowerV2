function [lineOut] = postProcessLine(line, shouldPlot)
N = size(line,1);

%smooth x and y coordinates
lineSmooth = zeros(size(line));
lineSmooth(:,1) = smoothn(line(:,1), 50,'robust');
lineSmooth(:,2) = smoothn(line(:,2), 50,'robust');
%TODO investigate using smoothn with 2D cell array input: smoothn({x,y});
lineSmoothDiff = diff(lineSmooth,1);

K = N - 1;

segmentLength = zeros(K,1);
for i = 1:K
    segmentLength(i) = norm(lineSmoothDiff(i,:));
end

segmentLengthCumSum = [0; cumsum(segmentLength)];

%interpolate to get constant segment length
oversamplingRatio = 1;
N2 = round(size(lineSmooth,1)*oversamplingRatio);
lineSmoothInterpX = interp1(segmentLengthCumSum,lineSmooth(:,1),linspace(0,segmentLengthCumSum(end),N2),'spline')';
lineSmoothInterpY = interp1(segmentLengthCumSum,lineSmooth(:,2),linspace(0,segmentLengthCumSum(end),N2),'spline')';
lineSmoothInterp = [lineSmoothInterpX, lineSmoothInterpY];

%calculate curvature
% lineSmoothInterpDiff = diff(lineSmoothInterp,1);
K2 = N2 - 1;
% segmentLengthInterp = zeros(K2,1);
% for i = 1:K2
%     segmentLengthInterp(i) = norm(lineSmoothInterpDiff(i,:));
% end
% curveHeadingRad = atan2(lineSmoothInterpDiff(:,2),lineSmoothInterpDiff(:,1));
[curveHeadingRad, segmentLengthInterp] = lineHeading(lineSmoothInterp');
segmentLengthInterpMean = mean(segmentLengthInterp);
curvature = zeros(K2,1);
for i = 2:K2
    %curvature = dHeading/dDist
    dHeading = wrapAngleRad(curveHeadingRad(i) - curveHeadingRad(i-1));
    curvature(i) = dHeading/segmentLengthInterpMean;
end

%find peak curvature autocorrelation -> find when does the line repeats

[curvatureCorr2,lags2] = xcorr(curvature,'unbiased');
curvatureCorr2 = curvatureCorr2/max(curvatureCorr2);
minLineLength = 2;
minIdx = 500;%round(1 + minLineLength/segmentLengthInterp(1));
[~, maxCorrLags] = max(curvatureCorr2(N2-1+minIdx:end));
maxCorrLags = maxCorrLags + minIdx - 2;

%close loop
mismatch = lineSmoothInterp((maxCorrLags+1),:) - lineSmoothInterp(1,:);
lineOut = zeros(maxCorrLags,2);
for i = 1:maxCorrLags
    lineOut(i,:) = lineSmoothInterp(i,:) - (i-1)/maxCorrLags*mismatch;
end

% outFile = [logName '_filtered.csv'];
% csvwrite(outFile, lineOut);

if shouldPlot
    figure(1)
    clf
    hold on
    grid on
    title('segment length');
    ylabel('length');
    plot(segmentLength)
    plot(segmentLengthInterp)
    legend('original', 'interpolated');
    
    figure(2)
    clf
    hold on
    grid on
    title('unbiased correlation');
    plot(lags2,curvatureCorr2,'.b')
    
    figure(4);
    clf;
    grid on
    hold on
    title('line');
    xlabel('x');
    ylabel('y');
    plot(line(:,1),line(:,2),'k')
    plot(lineSmooth(:,1),lineSmooth(:,2),'og')
    plot(lineSmoothInterp(1:maxCorrLags,1),lineSmoothInterp(1:maxCorrLags,2),'-xb')
    plot(lineOut(:,1),lineOut(:,2),'-xm')
    
    
    figure(5);
    clf;
    grid on
    hold on
    title('heading');
    plot((180/pi)*curveHeadingRad,'g')
    
    figure(6);
    clf;
    grid on
    hold on
    title('curvature');
    plot(curvature,'b')
end
end