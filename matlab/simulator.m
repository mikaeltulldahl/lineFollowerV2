function simulator()
clc
clear all
global recording map initPose path poseTemplate lineSensorX
addpath(genpath(fullfile(fileparts(mfilename('fullpath')),'matlabUtils')))
%SETTINGS
N = 1000; % maximum samples of recorded line
rng(1);
lineRecorded = coder.nullcopy(zeros(2,N));
lineSensorX = 0.085;
lineSensorWidth = 0.2185;
dt = 0.1;
simulationDuration = 10;
T = round(simulationDuration/dt);%simulation tics

shouldGenerateLine = true;
shouldPlot = true;
manualDriving = false;

%INIT 
%struct definitions
poseTemplate = struct('pos', [0;0], 'heading', 0);
twistTemplate = struct('speed', 0, 'yawRate', 0);
mapTemplate = struct('pos', zeros(2,N), 'heading', zeros(N,1), 'length', zeros(N,1));
lineTemplate = struct('pos', zeros(2,N), 'heading', zeros(N,1), 'length', zeros(N,1), 'count', 0);
pathTemplate = struct('pos', zeros(2,N), 'heading', zeros(N,1), 'curvature', zeros(N,1), 'length', zeros(N,1), 'range', [1 1]);

map = mapTemplate;
if shouldGenerateLine
    map.pos = generateLine(201, 0.03);%TODO must be closed loop!
else
    [~, ~, ~, ~, lineRaw] = logParser(false);
    map.pos = postProcessLine(lineRaw, false)';
end
[map.heading, map.length] = lineHeading(map.pos(:, [1:end 1]));
plotLimits = [min(map.pos(1,:)),...
              max(map.pos(1,:)),...
              min(map.pos(2,:)),...
              max(map.pos(2,:))];

recording = lineTemplate;
path = pathTemplate;%TODO optimize

poseTrue = poseTemplate;
startIdx = randi(size(map.pos,2));%random starting position somewhere along map
poseTrue.pos = map.pos(:,startIdx) + 0.01*randn(2,1);
poseTrue.heading = map.heading(startIdx) + 5*pi/180*randn;
initPoseTrue = poseTrue;
odometry = poseTemplate;

twistTrue = twistTemplate;
twistMeasured = twistTemplate;
twistRequest= twistTemplate;
lineMeasured = 0;

%SIMULATE
for t = 1:T
    
    t
    %propagate true state
    twistTrue = twistRequest;%TODO add noise
    [poseTrue, velTrue] = propagateState(poseTrue, twistTrue, dt);
    
    %make line, yawRate & speed measurment
    lineSensor = transform(poseTrue, [lineSensorX, lineSensorX; -0.5*lineSensorWidth, 0.5*lineSensorWidth]);
    [lineMeasuredTemp, hits, startIdx] = measureLineSensorMultipleHits(lineSensor, 1, startIdx);%TODO add noise  
    if hits == 1
        lineMeasured = lineMeasuredTemp;
    end

    twistMeasured = twistTrue;%TODO add noise
    
    %update odometry
    [odometry, ~] = propagateState(odometry, twistMeasured, dt);
    
    %plot truth
    if shouldPlot
        figure(1)
        clf;
        hold on;
        grid on;
        axis equal;
        axis([plotLimits])
        plot(map.pos(1,[1:end 1]),map.pos(2,[1:end 1]),'k');
        plot(lineSensor(1,:),lineSensor(2,:),'-rx');
        lineMeasuredPos = transform(poseTrue, [lineSensorX; lineMeasured]);
        plot(lineMeasuredPos(1),lineMeasuredPos(2),'rx');
        plot(poseTrue.pos(1),poseTrue.pos(2),'r.');
        plot([poseTrue.pos(1), poseTrue.pos(1) + dt*velTrue(1)],[poseTrue.pos(2), poseTrue.pos(2) + dt*velTrue(2)],'-r');
    end
    
    tic
    updateLocalisation(odometry, lineMeasured, t==1, initPoseTrue, startIdx);
    clc
    elapsedTime = toc
%     controlType = 'random';
    controlType = 'simple';
%     controlType = 'pathFollower';
    
    %decide on action
    switch controlType
        case 'random'
            twistRequest.speed = 0.9*twistRequest.speed + abs(dt*0.1*randn);
            twistRequest.yawRate = 0.7*twistRequest.yawRate + 30*pi/180*randn;
        case 'simple'
            twistRequest = simpleController(lineMeasured);
        case 'pathFollower'
            twistRequest = controller(odometry, dt);
    end
    
    %plot estimate    
    if shouldPlot        
        if ~isempty(initPose)
            posError(t) = 100*norm(initPose.pos - initPoseTrue.pos);
            headingError(t) = abs(180/pi*wrapAngleRad(initPose.heading - initPoseTrue.heading));
            poseEstimated =  transformPose(initPose, odometry);
            velEstimated = polar2cartesian(poseEstimated.heading,twistMeasured.speed);
            plot(poseEstimated.pos(1),poseEstimated.pos(2),'gx');
            plot([poseEstimated.pos(1), poseEstimated.pos(1) + velEstimated(1)],[poseEstimated.pos(2), poseEstimated.pos(2) + velEstimated(2)],'-r');
            
%             figure(11)
%             clf
%             semilogy(posError,'b');
%             hold on
%             grid on
%             semilogy(headingError,'r');
%             legend('posError [cm]', 'headingError[deg]');
        end
        drawnow;
%         pause(dt);
    end
end
end
