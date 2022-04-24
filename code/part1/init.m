function [sampledData, sampledVicon, sampledTime] = init(dataNum)
%RESAMPLEDATA Resample the given Vicon Data
addpath('../data') 
data1 = 'studentdata1.mat';
data4 = 'studentdata4.mat';
data9 = 'studentdata9.mat';

switch dataNum
    case 1
        allData = load(data1);
    case 4
        allData = load(data4);
    case 9
        allData = load(data9);
end
        
dataTimes = vertcat(allData.data(:).t); % Extract all values of t field from data
matchingIndices = ismember(allData.time, dataTimes); % Obtain incdices of matches
sampledTime = allData.time(matchingIndices); % Keep only 1 indices
%sampledTime = downsample(sampledTime', 2); % Downsample time
%sampledData = downsample(allData.data, 2); % Downsample data
sampledData = allData.data;
sampledVicon = allData.vicon(:, matchingIndices); % Keep only 1 indices
%sampledVicon = downsample(sampledVicon', 2)'; % Downsample Vicon
end