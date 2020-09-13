function lineFollowerTEST(inMap)
global adjustDirection lineIdxs map
clc
rng(124)
N = 201; %line points
ratio = 2;
M = (N-1)/ratio + 1; % waypoints
edgeLength = 0.03;
initVel = 1;
delta = 0.001;
if nargin == 0
    map = generateLine(N, edgeLength);
else
    map = inMap;
end

%init waypoints
waypoints = map(:,1:ratio:end);
waypointVel = zeros(size(waypoints));
for i = 1:M
    if i==1
        start = i;
        finish = i + 1;
    elseif i == M
        start = i - 1;
        finish = i;
    else
        start = i - 1;
        finish = i + 1;
    end
    deltaWaypoint = waypoints(:,finish) - waypoints(:,start);
    waypointVel(:,i) = initVel*deltaWaypoint/norm(deltaWaypoint);
end

%init plots
h = initPlots();
R = rotDeg(90);
figure(31)
clf
grid on
hold on
set(gca, 'YScale', 'log')
set(gca, 'XScale', 'log')
for i = 1:40
    %TODO
    % - cost:
    %   - there is only cost for segments, not vertices
    %   - time cost is time spent in segment
    %   - distanceToLineCost is only based on the furthest away point from the segment
    %   - slipRiskCost is based on centripetal acc (mean vel) and
    %   tangentialAcc
    %   - segments are assigned to line points. this assignment is
    %   done once 
    
%     [distancePrior, durationPrior] = getDistanceDuration(waypoints, waypointVel);
    [edgeDirection, ~] = getEdgeDirection(waypoints);
    adjustDirection = R*edgeDirection;
    assignLineIdxToWpIdx(waypoints);
    
    %adjust waypoints
%     funPath = @(x) costFunctionHelper(x, waypoints, waypointVel);
    funIdx = @(x, idx) wpCost(adjustPathIdx(waypoints, x, adjustDirection(:,idx), idx), waypointVel, idx);
    funTot = @(x) segmentCostTot(adjustPath(waypoints, x, adjustDirection), waypointVel);
    initX = zeros(M,1);
    [xOptimal, f0] = gradientDescentStep(funIdx, funTot, initX, delta); 
    % syntax: cost(i) = funIdx(x, i)
    % syntax: sum(cost) = funTot(x)
    waypoints = adjustPath(waypoints, xOptimal, adjustDirection);
    
    %adjust speed
%     funSpeed = @(x) costFunction(waypoints, [x(1:M)';x((M+1):end)']);
    funIdx = @(x, idx) wpCost(waypoints, adjustVel(waypointVel, x, idx), mod(idx-1,M)+1);
    funTot = @(x) segmentCostTot(waypoints, waypointVel + [x(1:M)';x((M+1):end)']);
    initX = zeros(2*M,1);%[waypointVel(1,:),waypointVel(2,:)]';
%     [xOptimal, f0] = gradientDescentStep(funSpeed, initX, delta);
    [xOptimal, f0] = gradientDescentStep(funIdx, funTot, initX, delta); 
    waypointVel = waypointVel + [xOptimal(1:M)';xOptimal((M+1):end)'];    
    
%     %remove too short segments
%     [~, edgeLength] = getEdgeDirection(waypoints);
%     idxToRemove = edgeLength < 0.01;
%     waypoints(:,idxToRemove) = [];
%     waypointVel(:,idxToRemove) = [];
%     M = M - sum(idxToRemove);
    
%     distToLine = max(getDistanceToLine(waypoints, map))
    
%     [distancePost, ~] = getDistanceDuration(targetPath, targetVel);
    if i ~= 1
        figure(31)
        plot(i, segmentDurationTot(waypoints, waypointVel),'or');
        plot(i,sum(f0),'ob');
    end
    updatePlots(h, waypoints, waypointVel);
end
end

function cost = costFunctionHelper(adjustment, oldPath, waypointVel)
global adjustDirection
cost = costFunction(adjustPath(oldPath, adjustment, adjustDirection), waypointVel);
end

% function cost = costFunction(waypoints, waypointVel)
% % Cost function= duration + slip risk + miss risk + top speed is impossible 
% robotWidth = 0.15;
% safetyDistance = 0.1;
% maxAcc = 10;
% % [~, duration] = getDistanceDuration(waypoints, waypointVel);
% distToLine = getDistanceToLine(waypoints);
% distanceToLineCost = mean(max(0, (distToLine - safetyDistance))/(robotWidth - safetyDistance));%mean((distToLine/robotWidth).^10);
% acc = getAccRequired(waypoints, waypointVel);
% slipRiskCost = mean((acc/maxAcc).^10); 
% cost = duration + distanceToLineCost + slipRiskCost;
% end

function cost = costFunction(waypoints, waypointVel)
M = size(waypoints,2);
K = M-1;
cost = 0;
for i = 1:K
    cost = cost + wpCost(waypoints, waypointVel, i);
end
end

function cost = wpCost(wps, wpVel, wpIdx)
M = size(wps,2);
if wpIdx==1
    [cost,~] = segmentCost(wps, wpVel, 1);
elseif wpIdx == M
    [cost,~] = segmentCost(wps, wpVel, M - 1);
else
    [cost1,~] = segmentCost(wps, wpVel, wpIdx - 1);
    [cost2,~] = segmentCost(wps, wpVel, wpIdx);
    cost = 0.5*(cost1 + cost2);
end
end

function duration = segmentDurationTot(wps, wpVel)
M = size(wps,2);
K = M - 1;
duration = 0;
for i = 1:K
    [~, tempDuration] = segmentCost(wps, wpVel, i);
    duration = duration + tempDuration;
end
end

function cost = segmentCostTot(wps, wpVel)
M = size(wps,2);
K = M - 1;
cost = 0;
for i = 1:K
    [tempCost, ~] = segmentCost(wps, wpVel, i);
    cost = cost + tempCost;
end
end

function [cost, duration] = segmentCost(wps, wpVel, segmentIdx)
global lineIdxs
robotWidth = 0.15;
% safetyDistance = 0.1;
maxAcc = 10;
startWp = segmentIdx;
finishWp = segmentIdx + 1;
[coeffs, duration, success] = getSplineCoeffs(wps(:,startWp),...
                                              wps(:,finishWp),...
                                              wpVel(:,startWp),...
                                              wpVel(:,finishWp));
if success
    distanceToLineCost = 0;
    if lineIdxs(segmentIdx,1) ~= 0
        start = lineIdxs(segmentIdx,1);
        finish = lineIdxs(segmentIdx,2);
        for n = start:finish
            tot = finish - start + 1;
            
            t = duration*(n - 1)/(tot + 1);
            temp = (n - start + 0.5)/tot;
            t = duration*(n - start + 0.5)/tot;
            tempDist = getDistanceToLine(n, t, coeffs);
            tempCost = (tempDist/robotWidth)^10;
            distanceToLineCost = distanceToLineCost + tempCost;
        end
    end
    accStart = norm(getSplineAcc(coeffs, 0));
    accFinish = norm(getSplineAcc(coeffs, duration));
    slipRiskCost = (accStart/maxAcc)^10 + (accFinish/maxAcc)^10;
    cost = duration + 0.0001*distanceToLineCost + 0.001*slipRiskCost;
else
    cost = 10;
end
end

function distance = getDistanceToLine(lineIdx, time, coeffs)
global map
%todo search for nearest point on spline
distance = norm(map(:,lineIdx) - getSplinePos(coeffs, time));
end

% function acc = getAccRequired(waypoints, vel)
% M = size(waypoints,2);
% acc = zeros(M,1); %TODO force per wheel
% for i = 2:(M - 1)
%     previous = i - 1;
%     next = i + 1;
%     [radius,~] = fit_circle_through_3_points(waypoints(:,[previous i next])');
%     normalAcc = vel(i)^2/abs(radius);
%     v1 = vel(previous);
%     v2 = vel(i);
%     v3 = vel(next);
%     length1 = norm(waypoints(:, i) - waypoints(:, previous));
%     length2 = norm(waypoints(:, next) - waypoints(:, i));
%     tangentialAcc1 = 0.5*(v2 - v1)*(v2 + v1)/length1;
%     tangentialAcc2 = 0.5*(v3 - v2)*(v3 + v2)/length2;
%     tangentialAcc = 0.5*(abs(tangentialAcc1) + abs(tangentialAcc2));
%     acc(i) = norm([normalAcc tangentialAcc]);
% end
% end


% function [distance, duration] = getDistanceDuration(waypoints, targetVel)
% M = size(waypoints,2);
% duration = 0;
% distance = 0;
% for i = 2:M
%     edge = waypoints(:,i) - waypoints(:,(i-1));
%     meanVel = (targetVel(i) + targetVel(i-1))/2;
%     edgeLength = norm(edge);
%     distance = distance + edgeLength;
%     duration = duration + edgeLength/meanVel;
% end
% end

function assignLineIdxToWpIdx(waypoints)
global lineIdxs map
N = size(map,2);
M = size(waypoints,2);
K = M - 1;
lineIdxs = zeros(K,2);
idx = 1;
for i = 1:N
    done = false;
    segmentCenter = 0.5*(waypoints(:,idx) + waypoints(:,idx + 1));
    dist = norm(map(:,i) - segmentCenter);
    while~done
        if (idx + 1) > K %idx is last segment
            done = true;
            if lineIdxs(idx,1) == 0
                lineIdxs(idx,1) = i;
            end 
            lineIdxs(idx,2) = i; %assign this linepoint to idx
        else %possible that next idx is nearer
            nextSegmentCenter = 0.5*(waypoints(:,idx+1) + waypoints(:,idx + 2));
            nextDist = norm(map(:,i) - nextSegmentCenter);
            if nextDist < dist % next segment is closer, increment idx
                dist = nextDist;
                idx = idx + 1;
            else % next segment further away, thus current idx is the nearest
                %TODO assign waypoints idx to linepoints and reuse when it's time to calculate
                %distToPath for specific segment
                done = true;
                if lineIdxs(idx,1) == 0 %first linepoint on this segment
                    lineIdxs(idx,1) = i;
                end
                lineIdxs(idx,2) = i;
            end
        end
    end
end
end

%keep
function [edgeDirection, edgeLength] = getEdgeDirection(waypoints)
M = size(waypoints,2);
edgeDirection = zeros(size(waypoints));
edgeLength = zeros(M,1);
for i = 1:M
    if i==1
        start = i;
        finish = i + 1;
    elseif i == M
        start = i - 1;
        finish = i;
    else
        start = i - 1;
        finish = i + 1;
    end
    edge = waypoints(:,finish) - waypoints(:,start);
    edgeNorm = norm(edge);
    edgeLength(i) = edgeNorm/(finish - start);
    edgeDirection(:,i) = edge/edgeNorm;
end
end

function newPath = adjustPathIdx(oldPath, adjustment, adjustDirection, idx)
newPath = oldPath;
newPath(:,idx) = newPath(:,idx) + adjustment*adjustDirection;
end

function newVel = adjustVel(oldVel, adjustment, idx)
newVel = oldVel;
M = size(oldVel,2);
if idx <= M %adjust X
    newVel(1,idx) = oldVel(1,idx) + adjustment;
else % adjust Y
    newVel(2,idx-M) = oldVel(2,idx-M) + adjustment;
end
end

function newPath = adjustPath(oldPath, adjustment, adjustDirection)
newPath = oldPath + repmat(adjustment',2,1).*adjustDirection;
end

function h = initPlots()
global map
f1 = figure(32);
clf
axis equal
grid on
hold on
xlabel('X  [meter]');
ylabel('Y  [meter]');
c = colorbar;
c.Label.String = 'Speed [m/s]';
h(1) = surface([],[],[],[],...
        'facecol','no',...
        'edgecol','interp',...
        'linew',2);
h(2) = plot(map(1,:),map(2,:),'ob');

f2 = figure(33);
clf
axis equal
grid on
hold on
xlabel('X  [meter]');
ylabel('Y  [meter]');
c = colorbar;
c.Label.String = 'Acc [m/s^2]';
h(3) = surface([],[],[],[],...
        'facecol','no',...
        'edgecol','interp',...
        'linew',2);
h(4) = plot(map(1,:),map(2,:),'ob');

linkaxes([f1.CurrentAxes f2.CurrentAxes]);

end

function updatePlots(h, wps, wpVel)
M = size(wps,2);
K = M - 1;
vel = zeros(1,M);
for i = 1:M
    vel(i) = norm(wpVel(:,i));
end
accStart = zeros(1,K);
accFinish = zeros(1,K);
for i = 1:K
    startWp = i;
    finishWp = i + 1;
    [coeffs, duration, ~] = getSplineCoeffs(wps(:,startWp),...
                                              wps(:,finishWp),...
                                              wpVel(:,startWp),...
                                              wpVel(:,finishWp));
    accStart(i) = norm(getSplineAcc(coeffs, 0));
    accFinish(i) = norm(getSplineAcc(coeffs, duration));
end

acc = zeros(1,K);
for i = 1:M
    if i==1
        acc(i) = accStart(i);
    elseif i == M
        acc(i) = accFinish(i-1);
    else
        acc(i) = max(accStart(i-1), accFinish(i));
    end
end


h(1).XData = [wps(1,:);wps(1,:)];
h(1).YData = [wps(2,:);wps(2,:)];
h(1).ZData = [zeros(2, size(wps, 2))];
h(1).CData = [vel;vel];

h(3).XData = [wps(1,:);wps(1,:)];
h(3).YData = [wps(2,:);wps(2,:)];
h(3).ZData = [zeros(2, size(wps, 2))];
h(3).CData = [acc;acc];
drawnow;
end

function [R,xcyc] = fit_circle_through_3_points(ABC)
    % FIT_CIRCLE_THROUGH_3_POINTS
    % Mathematical background is provided in http://www.regentsprep.org/regents/math/geometry/gcg6/RCir.htm
    %
    % Input:
    %
    %   ABC is a [3 x 2n] array. Each two columns represent a set of three points which lie on
    %       a circle. Example: [-1 2;2 5;1 1] represents the set of points (-1,2), (2,5) and (1,1) in Cartesian
    %       (x,y) coordinates.
    %
    % Outputs:
    %
    %   R     is a [1 x n] array of circle radii corresponding to each set of three points.
    %   xcyc  is an [2 x n] array of of the centers of the circles, where each column is [xc_i;yc_i] where i
    %         corresponds to the {A,B,C} set of points in the block [3 x 2i-1:2i] of ABC
    %
    % Author: Danylo Malyuta.
    % Version: v1.0 (June 2016)
    % ----------------------------------------------------------------------------------------------------------
    % Each set of points {A,B,C} lies on a circle. Question: what is the circles radius and center?
    % A: point with coordinates (x1,y1)
    % B: point with coordinates (x2,y2)
    % C: point with coordinates (x3,y3)
    % ============= Find the slopes of the chord A<-->B (mr) and of the chord B<-->C (mt)
    %   mt = (y3-y2)/(x3-x2)
    %   mr = (y2-y1)/(x2-x1)
    % /// Begin by generalizing xi and yi to arrays of individual xi and yi for each {A,B,C} set of points provided in ABC array
    x1 = ABC(1,1:2:end);
    x2 = ABC(2,1:2:end);
    x3 = ABC(3,1:2:end);
    y1 = ABC(1,2:2:end);
    y2 = ABC(2,2:2:end);
    y3 = ABC(3,2:2:end);
    % /// Now carry out operations as usual, using array operations
    mr = (y2-y1)./(x2-x1);
    mt = (y3-y2)./(x3-x2);
    % A couple of failure modes exist:
    %   (1) First chord is vertical       ==> mr==Inf
    %   (2) Second chord is vertical      ==> mt==Inf
    %   (3) Points are collinear          ==> mt==mr (NB: NaN==NaN here)
    %   (4) Two or more points coincident ==> mr==NaN || mt==NaN
    % Resolve these failure modes case-by-case.
    idf1 = isinf(mr); % Where failure mode (1) occurs
    idf2 = isinf(mt); % Where failure mode (2) occurs
    idf34 = isequaln(mr,mt) | isnan(mr) | isnan(mt); % Where failure modes (3) and (4) occur
    % ============= Compute xc, the circle center x-coordinate
    xcyc = (mr.*mt.*(y3-y1)+mr.*(x2+x3)-mt.*(x1+x2))./(2*(mr-mt));
    xcyc(idf1) = (mt(idf1).*(y3(idf1)-y1(idf1))+(x2(idf1)+x3(idf1)))/2; % Failure mode (1) ==> use limit case of mr==Inf
    xcyc(idf2) = ((x1(idf2)+x2(idf2))-mr(idf2).*(y3(idf2)-y1(idf2)))/2; % Failure mode (2) ==> use limit case of mt==Inf
    xcyc(idf34) = NaN; % Failure mode (3) or (4) ==> cannot determine center point, return NaN
    % ============= Compute yc, the circle center y-coordinate
    xcyc(2,:) = -1./mr.*(xcyc-(x1+x2)/2)+(y1+y2)/2;
    idmr0 = mr==0;
    xcyc(2,idmr0) = -1./mt(idmr0).*(xcyc(idmr0)-(x2(idmr0)+x3(idmr0))/2)+(y2(idmr0)+y3(idmr0))/2;
    xcyc(2,idf34) = NaN; % Failure mode (3) or (4) ==> cannot determine center point, return NaN
    % ============= Compute the circle radius
    R = sqrt((xcyc(1,:)-x1).^2+(xcyc(2,:)-y1).^2);
    R(idf34) = Inf; % Failure mode (3) or (4) ==> assume circle radius infinite for this case
end