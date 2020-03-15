function line = generateLine(N, edgeLength)
line = zeros(2,N);
direction = 0;%rand*2*pi;
for i = 2:N
    direction = direction + 10*pi/180*randn;
    line(:,i) = transform(struct('pos', line(:,i-1), 'heading', direction),  [edgeLength; 0]);    
    
end