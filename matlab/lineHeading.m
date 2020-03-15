function [lineHeadings, lineLengths] = lineHeading(line)
lineDiff = diff(line, 1, 2);
lineLengths = vecnorm(lineDiff);
lineHeadings = atan2(lineDiff(2,:), lineDiff(1,:));
end