%view log
addpath(genpath(fullfile(fileparts(mfilename('fullpath')),'matlabUtils')))
clear all
clc
file = uigetfile('*.TXT');
fprintf('loaded log %s\n',file);
[~, ~, ~, ~, lineRaw] = logParser(true, file);
postProcessLine(lineRaw, true);