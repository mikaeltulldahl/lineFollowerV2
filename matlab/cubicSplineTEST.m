clear all
clc
epsilon = 1e-4;
p0 = randn(2,1);
v0 = randn(2,1);
p1 = v0*3 + randn(2,1);
v1 = v0 + 0.5*randn(2,1);

[coeffs, t1, ~] = getSplineCoeffs(p0, p1, v0, v1);
time = linspace(0, t1);
p = [];
v = [];
v2 = [];
for i = 1:length(time)
    p(:,i) = getSplinePos(coeffs, time(i));
    v(:,i) = getSplineVel(coeffs, time(i));
    a(:,i) = getSplineAcc(coeffs, time(i));
    normAcc(i) = norm(a(:,i));
    if i == 1
        v2(:,i) = v0;
    else
        v2(:,i) = (p(:,i) - p(:,i-1))/(time(i) - time(i-1));
    end
end
algebraicMaxAcc = max(norm(getSplineAcc(coeffs, 0)),norm(getSplineAcc(coeffs, t1)))

% PLOTTING
figure(1)
clf
grid on
hold on
title('POSITION');
xlabel('time');
ylabel('pos');
plot(time,p(1,:))
plot(time,p(2,:))
legend('X','Y')

figure(2)
clf
grid on
hold on
title('VELOCITY');
xlabel('time');
ylabel('vel');
plot(time,v(1,:))
plot(time,v(2,:))

unitdP = (p1 - p0)/norm(p1 - p0);
Rtemp = rot(90*pi/180);
R = [unitdP, Rtemp*unitdP];
temp = R'*v;
plot(time,temp(1,:))
legend('X','Y','along Dir')

plot(time,v2(1,:))
plot(time,v2(2,:))

figure(3)
clf
grid on
hold on
xlabel('X');
ylabel('Y');
plot(p(1,:),p(2,:))
text(p(1,1),p(2,1),'start')
text(p(1,end),p(2,end),'finish')

figure(4)
clf
grid on
% view(3);
hold on
title('ACCELERATION');
xlabel('X');
ylabel('Y');
zlabel('Time');
plot3(a(1,:),a(2,:), time,'b');
plot3([0 0],[0 0], time([1 end]),'r');
text(a(1,1),a(2,1), time(1),'start')
text(a(1,end),a(2,end), time(end),'finish')

