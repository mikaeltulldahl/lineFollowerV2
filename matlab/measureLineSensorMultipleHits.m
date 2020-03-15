function [lineMeasured, hits, idx] = measureLineSensorMultipleHits(lineSensor, maxHits, lineStartIdx)
global map
%lineSensor =[xRight, xLeft;
%             yRight, yLeft]
lineMeasured = zeros(maxHits,1);
idx = zeros(maxHits,1);
sensorDir = lineSensor(:,2) - lineSensor(:,1);
sensorLength = norm(sensorDir);
N = size(map.pos,2);
hits = 0;
steps = 0;
idxHistory = [];
i = lineStartIdx;
while steps < N
    i = mod(i-1, N) + 1;
    iplus1= mod((i+1)-1, N) + 1;
    idxHistory(steps+1) = i;%temporary
    segment = map.pos(:,[i iplus1]);
    lineDir = segment(:,2) - segment(:,1);
    [dist, s] = intersection(lineSensor, sensorDir, sensorLength, segment, lineDir);
    if ~isnan(dist)
        hits = hits + 1;
        idx(hits) = i;
        lineMeasured(hits) = dist;
        if hits == maxHits
            break;
        end
    end
    if s < 1
        increment = 10;
    elseif s < 10 %[1 10]
        increment = max(1,floor(0.5*s)); % risk of overshooting
        %TODO use measureLineSensorSingleHit
    else
        increment = 5;
    end
    i = i + increment;
    steps = steps + increment;
end
end