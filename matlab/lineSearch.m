function stepLengthN = lineSearch(fun, delta)
N = 3;
% figure(3);
% clf
% hold on
% grid on

stepLength = delta*ones(N,1);
% stepLength(1) = 0;
f = zeros(N,1);
f(1) = fun(stepLength(1));
for i = 2:N
    fa = fun(stepLength(i-1) - delta);
    fb = fun(stepLength(i-1) + delta);
    firstOrder1 = (fb - f(i-1))/delta;
    firstOrder2 = -(fa - f(i-1))/delta;
    firstOrder = 0.5*(firstOrder1 + firstOrder2);
    secondOrder = (firstOrder1 - firstOrder2)/delta;
    if abs(secondOrder) < 1e-20
        stepLength(i) = stepLength(i-1);
    else
        stepLength(i) = stepLength(i-1) - 0.7*firstOrder/secondOrder;
    end
    f(i) = fun(stepLength(i));
    cnt = 0;
    while f(i) > f(i-1) && cnt < 5  %backtrack if newton method failed
        stepLength(i) = stepLength(i-1) + 0.5*(stepLength(i) - stepLength(i-1));
        f(i) = fun(stepLength(i));
        cnt = cnt + 1;
    end
end
stepLengthN = stepLength(N);
% x = linspace(min(stepLength), max(stepLength), 7);
% y = zeros(size(x));
% for i = 1:length(x)
%     y(i) = fun(x(i));
% end
% plot(x,y);
% plot(stepLength(1:(end-1)),f(1:(end-1)),'rx');
% plot(stepLength(N),f(N),'ro');
% drawnow
end