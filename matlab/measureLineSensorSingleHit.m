function [lineMeasured, hits, idx] = measureLineSensorSingleHit(lineSensor, lineStartIdx)
global map
%line =[x1, x2,...;
%       y1, y2,...]
%lineSensor =[xRight, xLeft;
%             yRight, yLeft]
lineMeasured = NaN;
idx = NaN;
sensorDir = lineSensor(:,2) - lineSensor(:,1);
sensorLength = norm(sensorDir);
N = size(map.pos,2);
hits = 0;
steps = 0;
i = lineStartIdx;
stepsMax = 5;

while steps < stepsMax
    i = mod(i-1, N) + 1;
    iplus1= mod((i+1)-1, N) + 1;
    segment = map.pos(:,[i iplus1]);
    lineDir = segment(:,2) - segment(:,1);
    [dist, s] = intersection(lineSensor, sensorDir, sensorLength, segment, lineDir);
    if ~isnan(dist)
        hits = 1;
        idx = i;
        lineMeasured = dist;
        break;
    else

        if s > 0 && s < 1
            break;
%             [~, tempHits, tempIdx] = measureLineSensorMultipleHits(lineSensor, 10, 1);
%             if tempHits ~= 0
%                 plot(lineSensor(1,:),lineSensor(2,:),'m', 'LineWidth',2);
%                 for n = idxHistory
%                     segment = map.pos(:,n:(n+1));
%                     plot(segment(1,:),segment(2,:),'m', 'LineWidth',2);
%                     text(segment(1,1),segment(2,1),num2str(n));
%                 end
%                 drawnow
%                 derp = 1;
%                 %                     error('s should not be able to be in range [0 1]');
%                 i = tempIdx(1);
%             else
%                 break;
%             end
        else
            i = i + max(-10, min(10, floor(s)));%TODO use 10 instead
        end
        
    end
    steps = steps + 1;
end

% if hits == 0 && steps == stepsMax
%     [tempLineMeasured, tempHits, tempIdx] = measureLineSensorMultipleHits(lineSensor, 10, 1);
%     if tempHits ~= 0
%         plot(lineSensor(1,:),lineSensor(2,:),'m', 'LineWidth',2);
%         for n = idxHistory
%             segment = map.pos(:,n:(n+1));
%             plot(segment(1,:),segment(2,:),'m', 'LineWidth',2);
%             text(segment(1,1),segment(2,1),num2str(n));
%         end
%         drawnow
%         lineMeasured =tempLineMeasured(1);
%         hits =1;
%         idx = tempIdx(1);
%     end
% end
end