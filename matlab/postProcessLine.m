function lineOut = postProcessLine(line, shouldPlot)
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
K2 = N2 - 1;
[curveHeadingRad, segmentLengthInterp] = lineHeading(lineSmoothInterp');
segmentLengthInterpMean = mean(segmentLengthInterp);
curvature = zeros(K2,1);
for i = 2:K2
    dHeading = wrapAngleRad(curveHeadingRad(i) - curveHeadingRad(i-1));
    curvature(i) = dHeading/segmentLengthInterpMean;
end

%find peak curvature using autocorrelation -> find when the curvature repeats
[curvatureCorr,lags] = xcorr(curvature,'unbiased');
curvatureCorr = curvatureCorr/max(curvatureCorr);
maxCorrIdxs = findPeaks(curvatureCorr, 0.95);
lapLengths = segmentLengthInterpMean*diff(maxCorrIdxs);
maxCorrLags = round(median(diff(maxCorrIdxs)));

%close loop
mismatch = lineSmoothInterp((maxCorrLags+1),:) - lineSmoothInterp(1,:);
lineOut = zeros(maxCorrLags,2);
for i = 1:maxCorrLags
    lineOut(i,:) = lineSmoothInterp(i,:) - (i-1)/maxCorrLags*mismatch;
end

% outFile = [logName '_filtered.csv'];
% csvwrite(outFile, lineOut);

if shouldPlot
    figure(21)
    clf
    hold on
    grid on
    title('segment length');
    ylabel('length');
    plot(segmentLength)
    plot(segmentLengthInterp)
    legend('original', 'interpolated');

    figure(22)
    clf
    hold on
    grid on
    title('unbiased correlation');
    plot(lags,curvatureCorr,'.b')
    plot(lags(maxCorrIdxs),curvatureCorr(maxCorrIdxs),'or')

    figure(23);
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
    plot(lineOut(1,1),lineOut(1,2),'om')

    figure(24);
    clf;
    grid on
    hold on
    title('heading');
    plot((180/pi)*curveHeadingRad,'g')

    figure(25);
    clf;
    grid on
    hold on
    title('curvature');
    plot(curvature,'b')
end
end