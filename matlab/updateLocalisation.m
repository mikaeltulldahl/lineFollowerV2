function updateLocalisation(odometry, lineMeasured, init, initPoseTrue, startIdxTrue)
global recording map initPose poseTemplate lineSensorX startIdx
persistent candidates cost

N = length(map.heading);
skips = 10;
K = ceil(N/skips);
headingQstd = 0.1*pi/180;% rad
posQstd = 0.001;%meter

%init persistent variables
if isempty(candidates) || init
    candidates = struct('pos', {}, 'heading', {});
    for k = 1:K
        candidates(k) = poseTemplate;
    end
    initPose = candidates(1);
    cost = zeros(K,1);
    startIdx = ones(K,1);
end

% record lineMeasured
% TODO handle multiple or no lineMeasured -> put NaN
recording.count = recording.count + 1;
recording.pos(:, recording.count) = transform(odometry, [lineSensorX; lineMeasured]);
if recording.count > 1
    [recording.heading(recording.count-1), recording.length(recording.count-1)] = lineHeading(recording.pos(:,(recording.count - 1 : recording.count)));
end

if recording.count == 2
    %init candidates
    recPose = struct('pos', recording.pos(:, 1), 'heading', recording.heading(1));
    recPoseInv = inversePose(recPose);
    for k = 1:K
        n = 1+ (k-1)*skips;
        candidates(k) = transformPose(struct('pos', map.pos(:,n), 'heading', map.heading(n)), recPoseInv);
        startIdx(k) = n;
    end
end

if recording.count > 2
    %move startIdx for all candidates
    lastSegmentLength = mean(recording.length([recording.count-2, recording.count-1]));
    mapSegmentLength = map.length(1);
    startIdx = startIdx + round(lastSegmentLength/mapSegmentLength);
end

if recording.count >= 2
%     [costTrue, tempIdx] = scanMatchCost(initPoseTrue, true, startIdxTrue);
%     plot(initPoseTrue.pos(1),initPoseTrue.pos(2),'ro');
    
    for k = 1:K
        %Add noise to candidate
        %TODO add noise around latestRecording instead
        latestRecording = struct('pos', recording.pos(:,recording.count), 'heading', recording.heading(recording.count-1));
        odom_mapFrame = transformPose(candidates(k), odometry);
        posNoise = posQstd*randn(2,1);
        headingNoise = headingQstd*randn;
        odom_mapFrame.pos = odom_mapFrame.pos + posNoise;
        odom_mapFrame.heading = odom_mapFrame.heading + headingNoise;
        candidates(k) = transformPose(odom_mapFrame, inversePose(odometry));

        %evaulate candidate        
        shouldPlot = false;%rand < 0.2;
        [costTemp, startIdx(k)] = scanMatchCost(candidates(k), shouldPlot, startIdx(k));
        cost(k) = 0.9*cost(k) + 0.1*costTemp;
    end

%     costSum = sum(cost);
%     for k = 1:K
%         % plot
%         odom_mapFrame = transformPose(candidates(k), odometry);
%         plotPatch(odom_mapFrame.pos, 0.0000001*costSum/cost(k));
%     end
%     [~,costIdxs] = sort(cost);
%     for i = costIdxs(1)'
%         [~, ~] = scanMatchCost(candidates(i), true, startIdx(i));
%         plot(candidates(i).pos(1),candidates(i).pos(2),'rx');
%     end
%     drawnow;

    [~, bestCandidateIdx] = min(cost);
    initPose = candidates(bestCandidateIdx);
    
    [~, ~] = scanMatchCost(candidates(bestCandidateIdx), true, startIdx(bestCandidateIdx));
    
%     posSum = [0;0];
%     dir = [0;0];
%     for i = costIdxs(1:5)'
%         posSum = posSum + candidates(i).pos;
%         dir = dir + [cos(candidates(i).heading);sin(candidates(i).heading)];
%     end
%     initPose = struct('pos', posSum/5, 'heading', atan2(dir(2),dir(1)));
%     plot(initPose.pos(1),initPose.pos(2),'mx');

    %resample
    weights = 1./max(0.001,cost);
    weights = weights/sum(weights);
    idxs = resample(weights);
    candidates = candidates(idxs);
    startIdx = startIdx(idxs);
    cost = zeros(K,1);
end
end

function plotPatch(p, s)
s = sqrt(s);
patch('XData',[p(1) - s, p(1) + s, p(1) + s, p(1) - s,],'YData',[p(2) - s, p(2) - s, p(2) + s, p(2) + s,]);
end

function [cost, mapStartIdxOut] = scanMatchCost(initPose, shouldPlot, mapStartIdx)
global recording map
R = rotDeg(90);
searchDist = 0.1;

finish = recording.count-1;
start = max(1, finish - 50);
recordingInMapFrame = transform(initPose, recording.pos(:,start:(finish + 1)));
if shouldPlot
    figure(1)
    plot(recordingInMapFrame(1,:),recordingInMapFrame(2,:),'-xb');
end
recordingInMapFrameDiff = diff(recordingInMapFrame, 1, 2);
recordingInMapFrameDiffRot = R*recordingInMapFrameDiff;
dist = zeros(finish - start + 1,1);
mapStartIdxOut = mapStartIdx;
for i = finish:-1:start
    if i ~= finish
        lastSegmentLength = 0.5*(recording.length(i) + recording.length(i + 1));%mean(recording.length([i, i+1]));
        mapSegmentLength = map.length(1);
        idxDiff = round(lastSegmentLength/mapSegmentLength);
        mapStartIdx = mapStartIdx - idxDiff;
    end
    k = i - start + 1;%equivalent idx in recordingInMapFrame
    segmentCenter = 0.5*(recordingInMapFrame(:,k) + recordingInMapFrame(:,k+1));%mean(recordingInMapFrame(:,[k, k+1]), 2);
    perpDir = (searchDist/recording.length(i))*recordingInMapFrameDiffRot(:,k);
    perpSegment = [segmentCenter - perpDir, segmentCenter + perpDir];
    
    %find all intersections in map
    [tempDist, hits, tempIdx] = measureLineSensorSingleHit(perpSegment, mapStartIdx);
    
    if hits == 1
        dist(k) = tempDist;
        mapStartIdx = tempIdx;
    else
%         dist(i:-1:1) = searchDist;
%         break;
        dist(k) = searchDist;
    end
    
    if i == finish
        mapStartIdxOut = mapStartIdx;
    end

%     if shouldPlot
%         temp = [segmentCenter, segmentCenter + dist(i)*perpDir/norm(perpDir)];
%         plot(temp(1,:),temp(2,:),'-b');
%         text(temp(1,1),temp(2,1),num2str(i));
%         text(temp(1,2),temp(2,2),num2str(mapStartIdx));
%         drawnow;
%     end
end

% cost = sqrt(dot(abs(dist).^2, recording.length(start:finish))/sum(recording.length(start:finish)));
errorArea = abs(dist).*recording.length(start:finish);
cost = sum(errorArea)/sum(recording.length(start:finish));
% weights = linspace(0.5, 1, 50);
% weights = weights((start + end - finish):end);
% weights = (finish - start + 1)*weights/sum(weights);
% cost = dot(errorArea, weights)/sum(recording.length(start:finish));
end