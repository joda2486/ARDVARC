clc; close all


%% Initialize
% Read File
bag = rosbag('ardvarc_sim_bag_001_100.bag');

rosbag info 'filename.bag' % display information

%{

% Initilize Variables
times = select(bag,'TimeStamp');
% repeat w/ rgv ID, azimuth, elevations
bluetooth = [times,IDs,azimuths,elevations];

cam = select(bag,'Camera'); % camera files 

pose = select(bag,'Topic','/ardvarc/pose');
posestruct = readMessages(pose,'DataFormat','Struct');


%% Frame Transformations
% run transformation function on bluetooth data

frames = bag.AvailableFrames; % list of available frame transformations
tf = getTransform(bag,'world',frames{1}); % latest frame transform


%% Computer Vision
% read image, identify bounding box
img = rosReadImage(msg);

%% Poses

% plot path
xPoints = cellfun(@(m) double(m.X),posestruct);
yPoints = cellfun(@(m) double(m.Y),posestruct);
zPoints = cellfun(@(m) double(m.Z),posestruct);
plot(xPoints,yPoints,zPoints)



%% Output



%}
%% Functions




